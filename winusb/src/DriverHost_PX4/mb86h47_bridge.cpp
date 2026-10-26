// mb86h47_bridge.cpp

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "mb86h47_bridge.hpp"

#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <algorithm>

#include "zd1_firmware.h"

namespace px4 {

namespace {

constexpr std::uint8_t kReqMemory = 0xBC;
constexpr std::uint8_t kEpStreamIn = 0x81;

constexpr std::uint32_t kCore0Base = 0x1600000;
constexpr std::uint32_t kCore1Base = 0x1700000;
constexpr std::uint32_t kCoreCode = 0xA0000;
constexpr std::uint32_t kCoreResetVector = 0xFFFFC;
constexpr std::uint32_t kCoreHold = 0xFFFFFFFF;
constexpr std::uint32_t kCoreRun = 0xFFFCFFFF;

/* core 1 のデータ RAM (コア内アドレス)。zd1fw_gen.py の変数配置と一致させる */
constexpr std::uint32_t kVarBase = 0xA3C00;
constexpr std::uint32_t kVarExpect = kVarBase + 0x00;	/* 次に実行するコマンドパケットのシーケンス */
constexpr std::uint32_t kVarSlot = kVarBase + 0x04;	/* 次に実行する受信スロットの先頭 */
constexpr std::uint32_t kVarFirstErr = kVarBase + 0x0C;
constexpr std::uint32_t kVarAnswers = kVarBase + 0x10;
constexpr std::uint32_t kVarSlave = kVarBase + 0x60;	/* A (2 バイト番号) << 24 | B (1 バイト番号) << 16 */
constexpr std::uint32_t kMboxLive = kVarBase + 0x7C;
constexpr std::uint32_t kMboxAnswer = kVarBase + 0x80;
constexpr std::uint32_t kMboxRead = kVarBase + 0x84;
constexpr std::uint32_t kMboxReq = kVarBase + 0xC0;
constexpr std::uint32_t kMboxWrite = kVarBase + 0xC4;

constexpr std::uint32_t kCmdWindow = 0x82100000;
constexpr std::uint8_t kEpCmdOut = 0x02;
constexpr std::uint8_t kEpCmdIn = 0x83;
constexpr std::size_t kCmdSlots = 16;
constexpr std::size_t kCmdSlotSize = 512;
constexpr std::size_t kCmdMaxWords = 125;

/* 要求ハンドラ (SRAM 上の常駐コード) のパッチ位置 */
constexpr std::uint32_t kHandlerReadChk0 = 0xE9AC0;
constexpr std::uint32_t kHandlerReadChk1 = 0xE9AC4;
constexpr std::uint32_t kHandlerWriteChk0 = 0xE9E0C;
constexpr std::uint32_t kHandlerWriteChk1 = 0xE9E10;
constexpr std::uint32_t kHandlerShift = 0xE9D94;
constexpr std::uint32_t kRoutineReadBase = 0xE1000;
constexpr std::uint32_t kRoutineWriteBase = 0xE1400;

constexpr std::uint32_t LdiJmp(std::uint32_t target)
{
	/* ldi:20 target, r12 の 2 半語 */
	return (0x9B0Cu | (((target >> 16) & 0xF) << 4)) << 16 | (target & 0xFFFF);
}

/* 値はすべてビッグエンディアン */
inline std::uint32_t LoadBe(const std::uint8_t *p)
{
	return static_cast<std::uint32_t>(p[0]) << 24 | static_cast<std::uint32_t>(p[1]) << 16 |
		static_cast<std::uint32_t>(p[2]) << 8 | p[3];
}

inline void StoreBe(std::uint8_t *p, std::uint32_t v)
{
	p[0] = static_cast<std::uint8_t>(v >> 24);
	p[1] = static_cast<std::uint8_t>(v >> 16);
	p[2] = static_cast<std::uint8_t>(v >> 8);
	p[3] = static_cast<std::uint8_t>(v);
}

#define BR_TRY(expr) do { int r_ = (expr); if (r_) return r_; } while (0)

} // namespace

Mb86h47Bridge::Mb86h47Bridge(WINUSB_INTERFACE_HANDLE winusb, LogFn log)
	: winusb_(winusb),
	log_(log),
	cmd_ok_(false),
	cmd_seq_(0),
	cmd_slot_(0),
	started_(false),
	patched_(false),
	core0_loaded_(false),
	mbox_seq_(0),
	stream_run_(false)
{
	ULONG timeout = 1000;

	WinUsb_SetPipePolicy(winusb_, 0x00, PIPE_TRANSFER_TIMEOUT, sizeof(timeout), &timeout);
}

Mb86h47Bridge::~Mb86h47Bridge()
{
	StreamStop();
}

void Mb86h47Bridge::Log(const char *fmt, ...) const
{
	if (!log_)
		return;

	char buf[512];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	log_("%s", buf);
}

int Mb86h47Bridge::Control(bool read, std::uint32_t addr, void *buf, std::size_t len)
{
	WINUSB_SETUP_PACKET sp = {};
	ULONG xfer = 0;

	sp.RequestType = read ? 0xC0 : 0x40;
	sp.Request = kReqMemory;
	sp.Value = static_cast<USHORT>(((addr >> 8) & 0xFF00) | (addr >> 24));
	sp.Index = static_cast<USHORT>(addr & 0xFFFF);
	sp.Length = static_cast<USHORT>(len);

	if (!WinUsb_ControlTransfer(winusb_, sp, static_cast<PUCHAR>(buf), static_cast<ULONG>(len), &xfer, nullptr)) {
		Log("control transfer failed: %s addr=0x%08X len=%zu (error %lu)\n", read ? "read" : "write", addr, len, GetLastError());
		return -EIO;
	}

	return (xfer == len) ? 0 : -EIO;
}

int Mb86h47Bridge::Read32(std::uint32_t addr, std::uint32_t &value)
{
	std::uint8_t b[4];
	int ret = Control(true, addr, b, 4);

	if (!ret)
		value = LoadBe(b);

	return ret;
}

int Mb86h47Bridge::Write32(std::uint32_t addr, std::uint32_t value)
{
	std::uint8_t b[4];

	StoreBe(b, value);
	return Control(false, addr, b, 4);
}

int Mb86h47Bridge::Read16(std::uint32_t addr, std::uint16_t &value)
{
	std::uint32_t v;
	int ret = Read32(addr, v);

	if (!ret)
		value = static_cast<std::uint16_t>(v >> 16);

	return ret;
}

int Mb86h47Bridge::Write16(std::uint32_t addr, std::uint16_t value)
{
	std::uint32_t v;
	int ret = Read32(addr, v);

	if (ret)
		return ret;

	return Write32(addr, (v & 0x0000FFFF) | static_cast<std::uint32_t>(value) << 16);
}

int Mb86h47Bridge::Modify32(std::uint32_t addr, std::uint32_t clear, std::uint32_t set)
{
	std::uint32_t v;
	int ret = Read32(addr, v);

	if (ret)
		return ret;

	return Write32(addr, (v & ~clear) | set);
}

int Mb86h47Bridge::ReadBlock(std::uint32_t addr, void *buf, std::size_t len)
{
	if (!len || (len % 4) || len > 64)
		return -EINVAL;

	return Control(true, addr, buf, len);
}

int Mb86h47Bridge::WriteBlock(std::uint32_t addr, const void *buf, std::size_t len)
{
	if (!len || (len % 4) || len > 64)
		return -EINVAL;

	/* パッチ前は先頭ワードしか通らないため、1 ワードずつ書く */
	if (!patched_ && len > 4) {
		const std::uint8_t *p = static_cast<const std::uint8_t *>(buf);

		for (std::size_t i = 0; i < len; i += 4)
			BR_TRY(Control(false, addr + static_cast<std::uint32_t>(i), const_cast<std::uint8_t *>(p) + i, 4));

		return 0;
	}

	return Control(false, addr, const_cast<void *>(buf), len);
}

int Mb86h47Bridge::WriteRoutine(std::uint32_t base, const std::uint32_t *words, std::size_t n)
{
	for (std::size_t i = 0; i < n; i++)
		BR_TRY(Write32(base + static_cast<std::uint32_t>(i * 4), words[i]));

	/* ジャンプを入れる前に、書いた内容を読み直して確かめる */
	for (std::size_t i = 0; i < n; i++) {
		std::uint32_t v;

		BR_TRY(Read32(base + static_cast<std::uint32_t>(i * 4), v));
		if (v != words[i]) {
			Log("routine verify failed at 0x%08X: 0x%08X != 0x%08X\n", base + static_cast<std::uint32_t>(i * 4), v, words[i]);
			return -EIO;
		}
	}

	return 0;
}

int Mb86h47Bridge::PatchHandler()
{
	struct Site {
		std::uint32_t addr;
		std::uint32_t original;
	};
	static const Site sites[] = {
		{ kHandlerReadChk0, 0x9B800000 },
		{ kHandlerReadChk1, 0xAA04E40F },
		{ kHandlerWriteChk0, 0x9B800000 },
		{ kHandlerWriteChk1, 0xAA04E40D },
		{ kHandlerShift, 0xB4279F8D },
	};
	const std::uint32_t patched_words[] = {
		LdiJmp(kRoutineReadBase), 0x970C9FA0, LdiJmp(kRoutineWriteBase), 0x970C9FA0, 0x9FA09F8D,
	};

	std::size_t already = 0;

	for (std::size_t i = 0; i < 5; i++) {
		std::uint32_t v;

		BR_TRY(Read32(sites[i].addr, v));
		if (v == patched_words[i]) {
			already++;
		} else if (v != sites[i].original) {
			Log("unknown request handler: 0x%08X = 0x%08X (expected 0x%08X)\n", sites[i].addr, v, sites[i].original);
			return -ENODEV;
		}
	}

	if (already == 5) {
		/* 前回の起動でパッチ済み。ルーチンが同じ内容のときだけ再利用する */
		patched_ = true;

		for (std::size_t i = 0; i < sizeof(zd1fw::kZd1FwAddrRead) / 4; i++) {
			std::uint32_t v;

			BR_TRY(Read32(kRoutineReadBase + static_cast<std::uint32_t>(i * 4), v));
			if (v != zd1fw::kZd1FwAddrRead[i])
				return -ENODEV;
		}
		for (std::size_t i = 0; i < sizeof(zd1fw::kZd1FwAddrWrite) / 4; i++) {
			std::uint32_t v;

			BR_TRY(Read32(kRoutineWriteBase + static_cast<std::uint32_t>(i * 4), v));
			if (v != zd1fw::kZd1FwAddrWrite[i])
				return -ENODEV;
		}
		Log("request handler already patched\n");
		return 0;
	}
	if (already != 0)
		return -ENODEV;

	BR_TRY(WriteRoutine(kRoutineReadBase, zd1fw::kZd1FwAddrRead, sizeof(zd1fw::kZd1FwAddrRead) / 4));
	BR_TRY(WriteRoutine(kRoutineWriteBase, zd1fw::kZd1FwAddrWrite, sizeof(zd1fw::kZd1FwAddrWrite) / 4));

	/* 1 ワードずつ差し替える。どの時点でもハンドラが指すのは完成したルーチン */
	for (std::size_t i = 0; i < 5; i++)
		BR_TRY(Write32(sites[i].addr, patched_words[i]));

	patched_ = true;
	return 0;
}

int Mb86h47Bridge::SetupClocks()
{
	/* 値の意味は一部未解明。spec 01 章 1.5 のとおりに書く (アドレスは別名) */
	BR_TRY(Write16(0x9010C, 0x8201));
	BR_TRY(Write16(0x90114, 0x8008));
	BR_TRY(Write16(0x90118, 0xFFFF));

	static const std::uint32_t blocks[] = {
		0x14D0000, 0x10D0000, 0x12D0000, 0x13D0000, 0xE70000, 0xC70000, 0xD00000, 0x1600000, 0x1700000,
	};

	for (std::uint32_t base : blocks)
		BR_TRY(Modify32(base + 0x404, 0, 4u << 16));

	BR_TRY(Write16(0x9011C, 0x0006));
	BR_TRY(Write16(0x90128, 0x6000));
	BR_TRY(Write16(0x9012C, 0x4007));
	BR_TRY(Write16(0xF12144, 0x0003));
	BR_TRY(Write16(0xF12178, 0x0880));
	BR_TRY(Write16(0xF1217C, 0x0880));
	BR_TRY(Write16(0xF12198, 0x0880));
	BR_TRY(Write16(0xF1219C, 0x0880));
	return 0;
}

int Mb86h47Bridge::MaskInterrupts()
{
	/*
	 * ICR00〜03 は電源投入時に設定済みの USB 用割り込みなので触らない。
	 * 残りは、ホストが設定していない割り込みで不定のベクターへ飛ばないよう、レベル 31 (マスク) にする。
	 */
	std::uint8_t b[60];

	BR_TRY(ReadBlock(0x444, b, sizeof(b)));

	for (std::size_t i = 0; i < sizeof(b); i += 4) {
		std::uint32_t v = LoadBe(b + i);
		std::uint32_t m = v | 0x1F1F1F1F;

		if (m != v)
			BR_TRY(Write32(0x444 + static_cast<std::uint32_t>(i), m));
	}

	return 0;
}

int Mb86h47Bridge::SetupI2cMaster()
{
	BR_TRY(Modify32(0xFD8010, 0, 0x10));
	BR_TRY(Modify32(0xFD8014, 0x10, 0));
	BR_TRY(Write32(0xFD80C0, 1));
	BR_TRY(Write32(0xFD80C0, 0));
	BR_TRY(Write32(0xFD80C4, 0));
	BR_TRY(Write16(0x90114, 0x8008));
	BR_TRY(Write32(0xFD8FF8, 0x06));
	BR_TRY(Write32(0xFD8FF4, 0x08));
	BR_TRY(Write32(0xFD8FE8, 0x3C));
	BR_TRY(Write32(0xFD8FE4, 0x40));
	return 0;
}

int Mb86h47Bridge::LoadCore(std::uint32_t core_base, const std::uint32_t *code, std::size_t words, bool init_core1_vars)
{
	/* リセットベクターを先に置く。空のままでは、保持を解除しても動かない */
	BR_TRY(Write32(core_base + kCoreResetVector, kCoreCode));
	BR_TRY(Write32(core_base, kCoreHold));

	/* 再起動後のコアには前回のコードが残っていることが多いため、差分だけ書く */
	for (std::size_t i = 0; i < words; i += 16) {
		std::size_t n = std::min<std::size_t>(16, words - i);
		std::uint8_t cur[64], want[64];
		std::uint32_t addr = core_base + kCoreCode + static_cast<std::uint32_t>(i * 4);

		for (std::size_t j = 0; j < n; j++)
			StoreBe(want + j * 4, code[i + j]);

		BR_TRY(ReadBlock(addr, cur, n * 4));
		if (std::memcmp(cur, want, n * 4))
			BR_TRY(WriteBlock(addr, want, n * 4));
	}

	if (init_core1_vars) {
		BR_TRY(Write32(core_base + kMboxReq, 0));
		BR_TRY(Write32(core_base + kMboxAnswer, 0));
		BR_TRY(Write32(core_base + kMboxLive, 0));
		BR_TRY(Write32(core_base + kVarFirstErr, 0));
		BR_TRY(Write32(core_base + kVarAnswers, 0));
		BR_TRY(Write32(core_base + kVarSlave, 0x12C00000));
		BR_TRY(Write32(core_base + kVarSlot, kCmdWindow));
		/* 前回の実行が残したパケットに一致しないよう、起動のたびに新しいシーケンスから始める */
		LARGE_INTEGER qpc;

		QueryPerformanceCounter(&qpc);
		cmd_seq_ = (static_cast<std::uint32_t>(qpc.QuadPart) & 0x7FFFFFFF) | 1;
		BR_TRY(Write32(core_base + kVarExpect, cmd_seq_));
	}

	return 0;
}

int Mb86h47Bridge::ReleaseCore(std::uint32_t core_base)
{
	return Write32(core_base, kCoreRun);
}

int Mb86h47Bridge::Start()
{
	if (started_)
		return -EALREADY;

	std::uint16_t state;

	BR_TRY(Read16(0x82008, state));
	if (state) {
		Log("chip is not in the power-on state (0xF02008 = 0x%04X). replug the device.\n", state);
		return -EBUSY;
	}

	BR_TRY(PatchHandler());
	Log("request handler patched\n");
	BR_TRY(SetupClocks());
	BR_TRY(MaskInterrupts());
	BR_TRY(SetupI2cMaster());
	BR_TRY(LoadCore(kCore1Base, zd1fw::kZd1FwCore1, sizeof(zd1fw::kZd1FwCore1) / 4, true));
	BR_TRY(ReleaseCore(kCore1Base));
	Sleep(2);

	int ret = Core1Alive();
	if (ret) {
		Log("codec core 1 did not start\n");
		return ret;
	}

	/* コマンドチャネルが使えなくても、mailbox 経由で動作は続けられる */
	if (CmdSetup())
		Log("command channel is not available. falling back to the mailbox.\n");

	started_ = true;
	return 0;
}

void Mb86h47Bridge::Stop()
{
	StreamStop();
	cmd_ok_ = false;
	started_ = false;
}

int Mb86h47Bridge::Core1Alive()
{
	std::uint32_t a, b;

	BR_TRY(Read32(kCore1Base + kMboxLive, a));
	Sleep(2);
	BR_TRY(Read32(kCore1Base + kMboxLive, b));
	return (a != b) ? 0 : -ENODEV;
}

int Mb86h47Bridge::GpioRead(std::uint16_t &dir, std::uint16_t &val)
{
	BR_TRY(Read16(0x90014, dir));
	return Read16(0x90018, val);
}

int Mb86h47Bridge::ResetDemodulator()
{
	std::uint16_t v, d;

	BR_TRY(GpioRead(d, v));
	BR_TRY(Write16(0x90018, static_cast<std::uint16_t>((v & ~1) | (1 << 6))));
	BR_TRY(Write16(0x90014, static_cast<std::uint16_t>(d | 1 | (1 << 6))));
	Sleep(50);
	BR_TRY(Write16(0x90018, static_cast<std::uint16_t>(v | 1 | (1 << 6))));
	Sleep(10);

	/* I2C アドレスの確定 (DiB8000 のレジスタ 1287 に 3) */
	static const std::uint8_t cfg[] = { 0x05, 0x07, 0x00, 0x03 };

	return I2cTransfer(0x12, cfg, sizeof(cfg), nullptr, 0);
}

int Mb86h47Bridge::I2cTransfer(std::uint8_t addr8, const std::uint8_t *w, std::size_t wlen, std::uint8_t *r, std::size_t rlen)
{
	if (wlen > 32 || rlen > 32)
		return -EINVAL;

	if (++mbox_seq_ == 0)
		mbox_seq_ = 1;

	const std::uint8_t seq = mbox_seq_;

	if (wlen) {
		std::uint8_t buf[32] = {};

		std::memcpy(buf, w, wlen);
		BR_TRY(WriteBlock(kCore1Base + kMboxWrite, buf, (wlen + 3) & ~static_cast<std::size_t>(3)));
	}

	/* 要求ワードは最後に書く。書くと core 1 が開始する */
	BR_TRY(Write32(kCore1Base + kMboxReq, static_cast<std::uint32_t>(seq) << 24 | static_cast<std::uint32_t>(addr8) << 16 |
		static_cast<std::uint32_t>(wlen) << 8 | static_cast<std::uint32_t>(rlen)));

	const std::size_t ans_len = 4 + ((rlen + 3) & ~static_cast<std::size_t>(3));
	const ULONGLONG deadline = GetTickCount64() + 500;
	std::uint8_t ans[36];

	while (true) {
		BR_TRY(ReadBlock(kCore1Base + kMboxAnswer, ans, ans_len));
		if (ans[0] == seq)
			break;
		if (GetTickCount64() > deadline) {
			Log("i2c mailbox timeout (addr 0x%02X)\n", addr8);
			return -ETIMEDOUT;
		}
	}

	if (ans[3]) {
		Log("i2c transfer failed: addr 0x%02X error %u\n", addr8, ans[3]);
		return -EIO;
	}

	if (rlen)
		std::memcpy(r, ans + 4, rlen);

	return 0;
}

/* ------------------------------------------------------------------ */
/* コマンドチャネル (バルク OUT 0x02 / バルク IN 0x83)                   */
/* ------------------------------------------------------------------ */

int Mb86h47Bridge::CmdReadReply(std::uint32_t seq, std::uint32_t *answers, std::size_t max_answers, std::size_t &n_answers, unsigned int timeout_ms)
{
	const ULONGLONG deadline = GetTickCount64() + timeout_ms;

	/* 古い答えは先頭ワードが違うので読み飛ばす。全体の期限は絶対時刻で持つ */
	while (true) {
		const ULONGLONG now = GetTickCount64();

		if (now >= deadline)
			return -ETIMEDOUT;

		ULONG t = static_cast<ULONG>(deadline - now);
		std::uint8_t b[64];
		ULONG got = 0;

		WinUsb_SetPipePolicy(winusb_, kEpCmdIn, PIPE_TRANSFER_TIMEOUT, sizeof(t), &t);
		if (!WinUsb_ReadPipe(winusb_, kEpCmdIn, b, sizeof(b), &got, nullptr)) {
			if (GetLastError() == ERROR_SEM_TIMEOUT)
				return -ETIMEDOUT;
			Log("command reply read failed (error %lu)\n", GetLastError());
			return -EIO;
		}

		if (got < 8 || LoadBe(b) != seq)
			continue;

		const std::uint32_t hdr = LoadBe(b + 4);
		const std::size_t n = std::min<std::size_t>(hdr & 0xFF, (got - 8) / 4);

		for (std::size_t i = 0; i < n && i < max_answers; i++)
			answers[i] = LoadBe(b + 8 + i * 4);
		n_answers = n;

		if (hdr >> 16) {
			Log("i2c failure reported by core 1: code %u\n", hdr >> 16);
			return -EIO;
		}
		return 0;
	}
}

int Mb86h47Bridge::CmdSend(bool reply, std::uint32_t *answers, std::size_t max_answers, std::size_t &n_answers)
{
	const std::size_t n = cmd_held_.size();
	std::uint8_t pkt[kCmdSlotSize] = {};

	/* 最後のスロットは必ず答えを要求し、答えを受けてから次のパケットを出す */
	if (cmd_slot_ == kCmdSlots - 1)
		reply = true;

	const std::uint32_t seq = cmd_seq_;

	StoreBe(pkt, seq);
	StoreBe(pkt + 4, (reply ? 0x80000000u : 0) | static_cast<std::uint32_t>(n));
	for (std::size_t i = 0; i < n; i++)
		StoreBe(pkt + 8 + i * 4, cmd_held_[i]);
	StoreBe(pkt + kCmdSlotSize - 4, seq);

	ULONG written = 0;
	ULONG timeout = 500;

	WinUsb_SetPipePolicy(winusb_, kEpCmdOut, PIPE_TRANSFER_TIMEOUT, sizeof(timeout), &timeout);
	if (!WinUsb_WritePipe(winusb_, kEpCmdOut, pkt, sizeof(pkt), &written, nullptr) || written != sizeof(pkt)) {
		Log("command packet write failed (error %lu)\n", GetLastError());
		cmd_held_.clear();
		return -EIO;
	}

	cmd_held_.clear();
	cmd_seq_++;
	cmd_slot_ = (cmd_slot_ + 1) % kCmdSlots;
	n_answers = 0;

	int ret = 0;

	if (reply)
		ret = CmdReadReply(seq, answers, max_answers, n_answers, 500);

	if (cmd_slot_ == 0) {
		/* 16 パケットを終えたので、受信 DMA を先頭に戻す */
		int r = Write32(0xFD8508, kCmdWindow);
		if (!r)
			r = Write32(0xFD8504, 1);
		if (!ret)
			ret = r;
	}

	return ret;
}

int Mb86h47Bridge::CmdSetup()
{
	cmd_ok_ = false;

	BR_TRY(Write32(0xFD8300, 1));
	BR_TRY(Write32(0xFD8304, 0x00008004));
	BR_TRY(Modify32(0xFDBC14, 0, 0x00040000));
	BR_TRY(Modify32(0xFD80B4, 0, 4));
	BR_TRY(Write32(0xFD8510, kCmdSlotSize));
	BR_TRY(Write32(0xFD8508, kCmdWindow));
	BR_TRY(Write32(0xFD8504, 1));

	/* 前回の実行が残した答えを捨てる */
	for (int i = 0; i < 8; i++) {
		std::uint8_t b[64];
		ULONG got = 0;
		ULONG t = 5;

		WinUsb_SetPipePolicy(winusb_, kEpCmdIn, PIPE_TRANSFER_TIMEOUT, sizeof(t), &t);
		if (!WinUsb_ReadPipe(winusb_, kEpCmdIn, b, sizeof(b), &got, nullptr))
			break;
	}

	/* 空のパケットで、答えが返ることを確かめる */
	cmd_slot_ = 0;
	cmd_held_.clear();

	std::size_t n;
	int ret = CmdSend(true, nullptr, 0, n);

	if (ret)
		return ret;

	cmd_ok_ = true;
	return 0;
}

int Mb86h47Bridge::CmdPush(std::uint32_t word, bool is_read, std::uint16_t *value)
{
	cmd_held_.push_back(word);

	if (!is_read && cmd_held_.size() < kCmdMaxWords)
		return 0;

	std::uint32_t ans[16];
	std::size_t n = 0;
	int ret = CmdSend(is_read, ans, 16, n);

	if (ret)
		return ret;
	if (is_read) {
		if (n < 1)
			return -EIO;
		*value = static_cast<std::uint16_t>(ans[0]);
	}
	return 0;
}

int Mb86h47Bridge::I2cAccess(std::uint8_t addr8, const std::uint8_t *w, std::size_t wlen, std::uint8_t *r, std::size_t rlen)
{
	if (cmd_ok_) {
		/* レジスタ・アクセスの形 (A = DiB8000 の 2 バイト番号、B = DiB0090 の 1 バイト番号) だけをパケットに詰める */
		const bool a = (addr8 == 0x12), b = (addr8 == 0xC0);
		const bool shortreg = b;

		if ((a && ((wlen == 4 && rlen == 0) || (wlen == 2 && rlen == 2))) ||
			(b && ((wlen == 3 && rlen == 0) || (wlen == 1 && rlen == 2)))) {
			std::uint32_t reg = shortreg ? w[0] : static_cast<std::uint32_t>(w[0]) << 8 | w[1];

			if (reg <= 0x7FF) {
				std::uint32_t word = (shortreg ? 0x20000000u : 0) | reg << 16;

				if (rlen == 0) {
					const std::size_t off = shortreg ? 1 : 2;

					word |= static_cast<std::uint32_t>(w[off]) << 8 | w[off + 1];
					return CmdPush(word, false, nullptr);
				}

				std::uint16_t v = 0;
				int ret = CmdPush(word | 0x40000000u, true, &v);

				if (!ret) {
					r[0] = static_cast<std::uint8_t>(v >> 8);
					r[1] = static_cast<std::uint8_t>(v);
				}
				return ret;
			}
		}

		/* それ以外は mailbox。順序を保つため、溜めた分を送り終えてから行う */
		BR_TRY(I2cFlush());
	}

	return I2cTransfer(addr8, w, wlen, r, rlen);
}

int Mb86h47Bridge::I2cFlush()
{
	if (!cmd_ok_ || cmd_held_.empty())
		return 0;

	/* 溜めた書き込みの結果を確かめるため、答えを要求して送る */
	std::uint32_t ans[16];
	std::size_t n = 0;

	return CmdSend(true, ans, 16, n);
}

/* ------------------------------------------------------------------ */
/* カードスロット                                                      */
/* ------------------------------------------------------------------ */

namespace {

constexpr std::uint32_t kUartEnable = 0xFEA000;
constexpr std::uint32_t kUartLines = 0xFEA004;
constexpr std::uint32_t kUartBaud = 0xFEA008;
constexpr std::uint32_t kUartSend = 0xFEA00C;
constexpr std::uint32_t kUartControl = 0xFEA010;
constexpr std::uint32_t kUartFifo = 0xFEA014;
constexpr std::uint32_t kUartIrqMask = 0xFEA018;
constexpr std::uint32_t kUartIrq = 0xFEA01C;
constexpr std::uint32_t kUartCwt = 0xFEA020;
constexpr std::uint32_t kUartBwt = 0xFEA024;
constexpr std::uint32_t kUartTx = 0xFEA028;
constexpr std::uint32_t kUartRx = 0xFEA02C;
constexpr std::uint32_t kUartPower = 0xFEA034;
constexpr std::uint32_t kUartSentLrc = 0xFEA038;
constexpr std::uint32_t kUartRecvLrc = 0xFEA03C;

} // namespace

int Mb86h47Bridge::CardDetect(bool &detected)
{
	std::uint32_t v;

	BR_TRY(Read32(kUartIrq, v));
	detected = !(v & (1u << 10));
	return 0;
}

int Mb86h47Bridge::CardPowerDown()
{
	BR_TRY(Modify32(kUartLines, 1u << 2, 0));
	BR_TRY(Write32(kUartFifo, 0));
	BR_TRY(Modify32(kUartEnable, 1u << 2, 0));
	BR_TRY(Modify32(kUartLines, 0x0A, 0));
	BR_TRY(Write32(kUartIrqMask, 1));
	return Write32(kUartPower, 0);
}

int Mb86h47Bridge::CardReset()
{
	BR_TRY(CardPowerDown());
	Sleep(10);
	BR_TRY(Write32(kUartIrq, 0x3FF));
	BR_TRY(Write32(kUartPower, 1));
	BR_TRY(Modify32(kUartLines, 1u << 2, 0));
	BR_TRY(Write32(kUartFifo, 0));
	BR_TRY(Write32(kUartEnable, 0));
	BR_TRY(Write32(kUartEnable, 7));
	BR_TRY(Modify32(kUartLines, 0, 1u << 3));		/* VCC */
	BR_TRY(Write32(kUartBaud, 0xBA7F));			/* 9600bps */
	Sleep(2);
	BR_TRY(Modify32(kUartLines, 0, 1u << 1));		/* クロック */
	BR_TRY(Write32(kUartRecvLrc, 0x3B));
	BR_TRY(Write32(kUartCwt, 9600));
	BR_TRY(Write32(kUartBwt, 0));
	BR_TRY(Write32(kUartIrqMask, 0x1FF));
	Sleep(1);
	return Modify32(kUartLines, 0, 1u << 2);		/* リセット解除。カードが ATR を返す */
}

int Mb86h47Bridge::CardSetBaudrate(std::uint32_t baudrate)
{
	struct Rate {
		std::uint32_t bps, baud, control, cwt, bwt;
	};
	static const Rate rates[] = {
		{ 9600, 0xBA7F, 0, 9600, 0 },
		{ 19200, 0x5D5D, 0x00010000, 107, 36559 },
		{ 38400, 0x2EAE, 0x00010000, 107, 0xFFFF },
	};

	for (const auto &r : rates) {
		if (r.bps != baudrate)
			continue;

		BR_TRY(Write32(kUartBaud, r.baud));
		BR_TRY(Write32(kUartControl, r.control));
		BR_TRY(Write32(kUartCwt, r.cwt));
		return Write32(kUartBwt, r.bwt);
	}

	return -EINVAL;
}

int Mb86h47Bridge::CardDataReady(bool &ready)
{
	std::uint32_t v;

	BR_TRY(Read32(kUartFifo, v));
	ready = (v >> 16) != 0;
	return 0;
}

int Mb86h47Bridge::CardRead(std::uint8_t *buf, std::uint8_t &len)
{
	std::uint32_t v;

	BR_TRY(Read32(kUartFifo, v));

	const std::uint32_t waiting = v >> 16;
	const std::uint32_t n = std::min<std::uint32_t>(waiting, len);

	for (std::uint32_t i = 0; i < n; i++) {
		BR_TRY(Read32(kUartRx, v));
		buf[i] = static_cast<std::uint8_t>(v);
	}

	len = static_cast<std::uint8_t>(n);
	return 0;
}

int Mb86h47Bridge::CardWrite(const std::uint8_t *buf, std::uint8_t len)
{
	BR_TRY(Write32(kUartRecvLrc, 0));
	BR_TRY(Write32(kUartSentLrc, 0));
	for (std::uint8_t i = 0; i < len; i++)
		BR_TRY(Write32(kUartTx, buf[i]));
	BR_TRY(Write32(kUartIrq, 0x3FF));
	BR_TRY(Write32(kUartControl, (len >= 255) ? 0x280 : 0x200));
	return Write32(kUartSend, 1);
}

int Mb86h47Bridge::StreamPrepare()
{
	StreamPrepareBegin();
	BR_TRY(StreamPrepareEngine());
	return StreamPrepareUsb();
}

namespace {

struct PrepWrite {
	std::uint32_t addr;
	std::uint32_t value;
};

/* spec 04 章 4.2 の手順 1〜25 (TS エンジン)。TS の入力とリング書き込みは止めたまま設定する */
const PrepWrite kPrepWrites[] = {
	{ 0xD000C4, 0 },
	{ 0xD00000, 0xFFFFFFFF },
	{ 0xD00004, 0x001F33FF },
	{ 0xD00000, 0x30030FFF },
	{ 0xD00000, 0x00030000 },
	{ 0xD00004, 0x00003305 },
	{ 0xD01040, 0 },
	{ 0xD01064, 0 },
	{ 0xD010A0, 0 },
	{ 0xD010A4, 0 },
	{ 0xD010A8, 0 },
	{ 0xD01020, 0x02000000 },
	{ 0xD01004, 0 },
	{ 0xD01044, 0x00BC0000 },
	{ 0xD01048, 0x47000000 },
	{ 0xD0104C, 0x00FFFFFF },
	{ 0xD01050, 0 },
	{ 0xD01060, 0 },
	{ 0xD01000, 0x06070000 },
	{ 0xD01080, 0x01000000 },
	{ 0xD01088, 0 },
	{ 0xD0108C, 0xFF },
	{ 0xD01084, 0 },
	{ 0xD01090, 0x01000000 },
	{ 0xD02028, 0 },
	{ 0xD0202C, 0 },
	{ 0xD02030, 0 },
	{ 0xD02038, 0 },
	{ 0xD02000, 0x03020300 },
	{ 0xD02020, 0x00AE4000 },
	{ 0xD02024, 0x00600000 },
	{ 0xD00004, 0x00043305 },
};

} // namespace

void Mb86h47Bridge::StreamPrepareBegin()
{
	static_assert(sizeof(kPrepWrites) / sizeof(kPrepWrites[0]) == Mb86h47Bridge::kPrepEngineWrites, "prepare steps");
	prep_idx_ = 0;
}

int Mb86h47Bridge::StreamPrepareStep()
{
	if (prep_idx_ < kPrepEngineWrites) {
		const PrepWrite &w = kPrepWrites[prep_idx_++];

		return Write32(w.addr, w.value);
	}

	if (prep_idx_ == kPrepEngineWrites) {
		/* core 0 (パケタイザ) の読み込みと起動。差分だけ書くため、2 回目以降は短い */
		prep_idx_++;
		BR_TRY(LoadCore(kCore0Base, zd1fw::kZd1FwCore0, sizeof(zd1fw::kZd1FwCore0) / 4, false));
		core0_loaded_ = true;
		return ReleaseCore(kCore0Base);
	}

	return 0;
}

int Mb86h47Bridge::StreamPrepareEngine()
{
	while (StreamPrepareRemaining())
		BR_TRY(StreamPrepareStep());
	return 0;
}

int Mb86h47Bridge::StreamPrepareUsb()
{
	/* USB とエンドポイント 0x81 */
	BR_TRY(Modify32(0xFD8010, 0, 0x000F0000));
	BR_TRY(Write32(0xFD8200, 1));
	BR_TRY(Write32(0xFD8220, 1));
	BR_TRY(Modify32(0xFD8014, 0x000F0000, 0));
	BR_TRY(Write32(0xFD8240, 0));
	BR_TRY(Write32(0xFD82A0, 0));
	BR_TRY(Write32(0xFD8200, 0));
	BR_TRY(Write32(0xFD8204, 0xB09F0000));
	BR_TRY(Write32(0xFD8200, 2));
	BR_TRY(Write32(0xFD8220, 0));
	BR_TRY(Write32(0xFD8224, 0xB09F2000));
	BR_TRY(Write32(0xFD8220, 2));
	/* DMA 8 (EP 0x02 の受け) はコマンドチャネルが使っているため、設定済みなら触らない */
	if (!cmd_ok_) {
		BR_TRY(Write32(0xFD8300, 1));
		BR_TRY(Write32(0xFD8304, 0x00008004));
	}

	BR_TRY(Write32(0xFDBD08, 0x00FF6001));
	for (int i = 0; i < 100; i++) {
		std::uint32_t v;

		BR_TRY(Read32(0xFDBD08, v));
		if (!(v & 1))
			break;
	}
	BR_TRY(Write32(0xFDBD08, 0x00FF6000));
	BR_TRY(Write32(0xFDBC14, 0x00060030));
	return 0;
}

int Mb86h47Bridge::StreamStart(StreamHandler handler, const StreamConfig &config)
{
	if (stream_run_)
		return -EALREADY;

	stream_handler_ = std::move(handler);
	stream_config_ = config;

	ULONG raw_io = TRUE;
	ULONG timeout = 0;

	WinUsb_SetPipePolicy(winusb_, kEpStreamIn, RAW_IO, sizeof(raw_io), &raw_io);
	WinUsb_SetPipePolicy(winusb_, kEpStreamIn, PIPE_TRANSFER_TIMEOUT, sizeof(timeout), &timeout);
	WinUsb_FlushPipe(winusb_, kEpStreamIn);

	/* 先に読み出しを発行してから入力を許可する。送り始めてから発行すると先頭が欠ける */
	stream_run_ = true;
	stream_thread_ = std::thread(&Mb86h47Bridge::StreamThread, this);
	Sleep(20);

	int ret;

	ret = Write32(0xD01020, 0);
	if (!ret) ret = Write32(0xD01040, 0x03000000);
	if (!ret) ret = Write32(0xD010A4, 0x13170400);
	if (!ret) ret = Write32(0xD00484, 0x01000000);
	if (!ret) ret = Write32(0xD00488, 0x206C0000);
	if (!ret) ret = Write32(0xD02010, 0x02000000);
	if (ret)
		StreamStop();

	return ret;
}

void Mb86h47Bridge::StreamStop()
{
	if (stream_run_.exchange(false)) {
		WinUsb_AbortPipe(winusb_, kEpStreamIn);
		if (stream_thread_.joinable())
			stream_thread_.join();
	}

	if (core0_loaded_) {
		Write32(kCore0Base, kCoreHold);
		Write32(0xD01040, 0);
		Write32(0xD01020, 0x02000000);
		Write32(0xD02010, 0x04000000);
		core0_loaded_ = false;
	}
}

int Mb86h47Bridge::ReadCore0Counter(std::uint32_t &sent_units)
{
	return Read32(kCore0Base + 0xA3F04, sent_units);
}

void Mb86h47Bridge::StreamThread()
{
	const unsigned int n = stream_config_.urb_num;
	const unsigned int size = stream_config_.urb_bytes;
	std::vector<std::vector<std::uint8_t>> bufs(n, std::vector<std::uint8_t>(size));
	std::vector<OVERLAPPED> ovs(n);
	std::vector<bool> pending(n, false);

	for (unsigned int i = 0; i < n; i++) {
		std::memset(&ovs[i], 0, sizeof(OVERLAPPED));
		ovs[i].hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
	}

	auto submit = [&](unsigned int i) {
		ResetEvent(ovs[i].hEvent);
		if (!WinUsb_ReadPipe(winusb_, kEpStreamIn, bufs[i].data(), size, nullptr, &ovs[i]) && GetLastError() != ERROR_IO_PENDING)
			return false;
		pending[i] = true;
		return true;
	};

	bool ok = true;

	for (unsigned int i = 0; i < n && ok; i++)
		ok = submit(i);

	unsigned int cur = 0;

	while (ok && stream_run_) {
		DWORD wr = WaitForSingleObject(ovs[cur].hEvent, 300);

		if (wr == WAIT_TIMEOUT)
			continue;		/* 受信が止まっているだけ。データはチップ側のリングに残る */
		if (wr != WAIT_OBJECT_0)
			break;

		ULONG got = 0;

		pending[cur] = false;
		if (!WinUsb_GetOverlappedResult(winusb_, &ovs[cur], &got, FALSE))
			break;

		if (got && stream_handler_ && stream_handler_(bufs[cur].data(), got))
			break;

		if (!submit(cur))
			break;
		cur = (cur + 1) % n;
	}

	WinUsb_AbortPipe(winusb_, kEpStreamIn);
	for (unsigned int i = 0; i < n; i++) {
		if (pending[i]) {
			ULONG got;

			WinUsb_GetOverlappedResult(winusb_, &ovs[i], &got, TRUE);
		}
		CloseHandle(ovs[i].hEvent);
	}
}

} // namespace px4
