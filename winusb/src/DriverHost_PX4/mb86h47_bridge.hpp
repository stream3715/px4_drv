// mb86h47_bridge.hpp
//
// CECH-ZD1 J の USB ブリッジ (MB86H47A) の制御。
// ベンダー要求 0xBC によるチップ内メモリの読み書き、内蔵 CPU (codec core 0/1) へのプログラム投入、
// core 1 を介した I2C 転送、TS エンジンと SDRAM リングからのストリーム読み出しを扱う。

#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <atomic>
#include <thread>
#include <functional>

#include <windows.h>
#include <winusb.h>

namespace px4 {

class Mb86h47Bridge final {
public:
	using LogFn = void (*)(const char *fmt, ...);

	/* ストリームの 1 回分の読み出し結果 (192 バイト単位の連続) を受け取る。0 以外を返すと読み出しを止める */
	using StreamHandler = std::function<int(const std::uint8_t *buf, std::size_t len)>;

	struct StreamConfig {
		unsigned int urb_bytes = 64 * 1024;
		unsigned int urb_num = 16;
	};

	explicit Mb86h47Bridge(WINUSB_INTERFACE_HANDLE winusb, LogFn log = nullptr);
	~Mb86h47Bridge();

	Mb86h47Bridge(const Mb86h47Bridge &) = delete;
	Mb86h47Bridge& operator=(const Mb86h47Bridge &) = delete;

	/* 起動手順 (spec 01 章 1.4)。チップが新しい状態であること。 */
	int Start();
	void Stop();

	/* DiB8096 のリセット (01 章 1.6) と I2C アドレスの確定 (06 章 6.1 手順 4) */
	int ResetDemodulator();

	/* ---- チップ内メモリ ---- */
	int Read32(std::uint32_t addr, std::uint32_t &value);
	int Write32(std::uint32_t addr, std::uint32_t value);
	int Read16(std::uint32_t addr, std::uint16_t &value);
	int Write16(std::uint32_t addr, std::uint16_t value);
	int Modify32(std::uint32_t addr, std::uint32_t clear, std::uint32_t set);
	int ReadBlock(std::uint32_t addr, void *buf, std::size_t len);		/* 4 の倍数、最大 64 */
	int WriteBlock(std::uint32_t addr, const void *buf, std::size_t len);	/* 同上。パッチ後のみ 1 ワード超 */

	/*
	 * ---- I2C ----
	 * I2cAccess: DiB8000/DiB0090 のレジスタ・アクセスはコマンドチャネル (バルク転送) に溜めて送り、
	 * 読み出しのときに溜めた分と一緒に 1 往復で済ませる。書き込みの失敗は次の読み出しか I2cFlush で分かる。
	 * それ以外の転送、またはコマンドチャネルが使えないときは mailbox 経由。
	 */
	int I2cAccess(std::uint8_t addr8, const std::uint8_t *w, std::size_t wlen, std::uint8_t *r, std::size_t rlen);
	int I2cFlush();
	bool command_channel() const noexcept { return cmd_ok_; }
	/* mailbox 方式の転送 (最大 32 バイトずつ) */
	int I2cTransfer(std::uint8_t addr8, const std::uint8_t *w, std::size_t wlen, std::uint8_t *r, std::size_t rlen);
	/* core 1 の生存確認 */
	int Core1Alive();

	/* ---- GPIO (16 ビット) ---- */
	int GpioRead(std::uint16_t &dir, std::uint16_t &val);

	/* ---- カードスロット (スマートカード UART、spec 05 章) ---- */
	int CardDetect(bool &detected);
	int CardPowerDown();
	int CardReset();		/* リセットして ATR を待つ状態にする */
	int CardSetBaudrate(std::uint32_t baudrate);
	int CardDataReady(bool &ready);
	int CardRead(std::uint8_t *buf, std::uint8_t &len);
	int CardWrite(const std::uint8_t *buf, std::uint8_t len);

	/* ---- ストリーム ---- */
	int StreamPrepare();	/* 04 章 4.2 */
	/*
	 * StreamPrepare を、選局の待ち時間に 1 ステップずつ進められるようにしたもの (spec 04 章 4.4)。
	 * Begin で手順を用意し、Step で次の 1 つ (メモリ要求 1 回、または core 0 の読み込み) を実行する。
	 * Engine は残りの TS エンジンと core 0 を実行し、Usb は EP1/EP2 の DMA モードを書き換える。
	 * Usb は I2C のコマンドパケットの通信を乱すため、選局が終わってから実行する。
	 * 別のスレッドで並行して実行すると、I2C の読み出しが失敗することが実機で確認されたため、必ず同じスレッドで行う。
	 */
	void StreamPrepareBegin();
	bool StreamPrepareRemaining() const noexcept { return prep_idx_ <= kPrepEngineWrites; }
	int StreamPrepareStep();
	int StreamPrepareEngine();
	int StreamPrepareUsb();
	int StreamStart(StreamHandler handler, const StreamConfig &config);	/* 4.3 と読み出しの開始 */
	void StreamStop();	/* 4.5 */
	int ReadCore0Counter(std::uint32_t &sent_units);

	bool started() const noexcept { return started_; }

private:
	void Log(const char *fmt, ...) const;

	int Control(bool read, std::uint32_t addr, void *buf, std::size_t len);
	int PatchHandler();
	int WriteRoutine(std::uint32_t base, const std::uint32_t *words, std::size_t n);
	int SetupClocks();
	int MaskInterrupts();
	int SetupI2cMaster();
	int CmdSetup();
	int CmdPush(std::uint32_t word, bool is_read, std::uint16_t *value);
	int CmdSend(bool reply, std::uint32_t *answers, std::size_t max_answers, std::size_t &n_answers);
	int CmdReadReply(std::uint32_t seq, std::uint32_t *answers, std::size_t max_answers, std::size_t &n_answers, unsigned int timeout_ms);
	int LoadCore(std::uint32_t core_base, const std::uint32_t *code, std::size_t words, bool init_core1_vars);
	int ReleaseCore(std::uint32_t core_base);
	void StreamThread();

	WINUSB_INTERFACE_HANDLE winusb_;
	LogFn log_;
	static constexpr std::size_t kPrepEngineWrites = 32;
	std::size_t prep_idx_ = kPrepEngineWrites + 1;
	bool cmd_ok_;
	std::uint32_t cmd_seq_;
	std::size_t cmd_slot_;
	std::vector<std::uint32_t> cmd_held_;
	bool started_;
	bool patched_;
	bool core0_loaded_;
	std::uint8_t mbox_seq_;

	StreamHandler stream_handler_;
	StreamConfig stream_config_;
	std::atomic_bool stream_run_;
	std::thread stream_thread_;
};

} // namespace px4
