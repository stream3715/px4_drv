// dib8096.cpp
//
// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2005-9 DiBcom, SA et al.
// Linux の dib8000.c / dib0090.c / dibx000_common.c / dib0700_devices.c の移植。

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "dib8096.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <windows.h>

#include "dib8096_tables.h"

namespace px4 {

namespace {

constexpr std::uint8_t kDemodAddr = 0x12;
constexpr std::uint8_t kTunerAddr = 0xC0;
constexpr std::uint32_t kNever = 0xFFFFFFFFu;

/* 周波数帯 */
enum Band : int {
	BAND_LBAND = 0x01, BAND_UHF = 0x02, BAND_VHF = 0x04, BAND_SBAND = 0x08, BAND_FM = 0x10, BAND_CBAND = 0x20,
};

/* DVB API の値 */
enum : int { MOD_QPSK = 0, MOD_QAM16 = 1, MOD_QAM32 = 2, MOD_QAM64 = 3, MOD_AUTO = 6, MOD_DQPSK = 12 };
enum : int { FEC_NONE = 0, FEC_1_2 = 1, FEC_2_3 = 2, FEC_3_4 = 3, FEC_5_6 = 5, FEC_7_8 = 7, FEC_AUTO = 9 };
enum : int { TM_2K = 0, TM_8K = 1, TM_AUTO = 2, TM_4K = 3 };
enum : int { GI_1_32 = 0, GI_1_16 = 1, GI_1_8 = 2, GI_1_4 = 3, GI_AUTO = 4 };

/* 状態番号 */
enum : int {
	TUNER_START = 10, TUNER_STEP0, TUNER_STEP1, TUNER_STEP2, TUNER_STEP3, TUNER_STEP4, TUNER_STEP5, TUNER_STEP6, TUNER_STEP7, TUNER_STOP,
	AGC_START = 20, AGC_STEP0, AGC_STEP1, AGC_STEP2, AGC_STEP3, AGC_STOP,
	DEMOD_START = 30, DEMOD_STEP1, DEMOD_STEP2, DEMOD_STEP3, DEMOD_STEP4, DEMOD_STEP5, DEMOD_STEP6, DEMOD_STEP7,
	DEMOD_STEP8, DEMOD_STEP9, DEMOD_STEP10,
	DEMOD_SEARCH_NEXT = 51, DEMOD_STEP_LOCKED, DEMOD_STOP,
};

enum : int { SEARCH_NONE = 0, SEARCH_FFT, SEARCH_GUARD, SEARCH_DONE };
enum : int { ST_FAILED = 0, ST_PENDING = -3, ST_FFT_SUCCESS = -5, ST_LOCKED = -7, ST_DATA_LOCKED = -8 };

enum : unsigned int { CAL_DC = 1, CAL_WBD = 2, CAL_TEMP = 4, CAL_CAPTRIM = 8 };

/* ボード設定 (dib809x_dib0090_config) */
constexpr int kClockKHz = 12000;
constexpr int kAdcClockRatio = 8;
constexpr int kFRefClockRatio = 6;
constexpr int kFreqOffsetUHF = -63;
constexpr int kFreqOffsetVHF = -143;

const WbdSlope kWbdTable[] = {
	{ 120, 0, 500, 0, 500, 4 },
	{ 170, 0, 450, 0, 450, 4 },
	{ 380, 48, 373, 28, 259, 6 },
	{ 860, 34, 700, 36, 616, 6 },
	{ 0xFFFF, 34, 700, 36, 616, 6 },
};

struct AgcConfig {
	std::uint16_t agc1_max;
};

int BandOfFrequency(std::uint32_t khz)
{
	if (khz <= 170000)
		return BAND_CBAND;
	if (khz <= 250000)
		return BAND_VHF;
	if (khz <= 863000)
		return BAND_UHF;
	if (khz <= 2000000)
		return BAND_LBAND;
	return BAND_SBAND;
}

#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

/* スレッドごとの高分解能タイマー。作れない環境 (Windows 10 1803 より前) では Sleep() にフォールバックする */
struct PreciseTimer final {
	PreciseTimer() : handle(CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS)) {}
	~PreciseTimer() { if (handle) CloseHandle(handle); }
	HANDLE handle;
};

int Fls(int x)
{
	int r = 0;

	while (x) {
		r++;
		x >>= 1;
	}
	return r;
}

} // namespace

Dib8096::Dib8096(Bus &bus, LogFn log, WaitFn wait)
	: bus_(bus), log_(log), wait_(wait), err_(0),
	revision_(0), timf_(0), timf_default_(20199727), pll_prediv_cached_(0), gate_open_(false),
	current_band_(0), current_agc_(0), wbd_ref_(0), diversity_on_off_(0), differential_constellation_(false),
	mode_(3), seg_diff_mask_(0), seg_mask_(0x1FFF), isdbt_cfg_loaded_(false), channel_(),
	output_mode_(OutputMode::PARALLEL_GATED), asleep_(true), channel_parameters_set_(false),
	tune_state_(0), autosearch_state_(0), found_nfft_(TM_AUTO), found_guard_(GI_AUTO), status_(ST_PENDING),
	symbol_duration_(1), longest_intlv_layer_(0), timeout_us_(0), tuner_(),
	locks_(), locks_dirty_(false), manual_mode_(false), last_tune_manual_(false)
{
}

Dib8096::~Dib8096() {}

std::uint64_t Dib8096::NowUs() const
{
	return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

/* ------------------------------------------------------------------ */
/* I2C                                                                */
/* ------------------------------------------------------------------ */

void Dib8096::Wr(std::uint16_t reg, std::uint16_t val)
{
	if (err_)
		return;

	const std::uint8_t b[4] = { static_cast<std::uint8_t>(reg >> 8), static_cast<std::uint8_t>(reg), static_cast<std::uint8_t>(val >> 8), static_cast<std::uint8_t>(val) };

	err_ = bus_.Transfer(kDemodAddr, b, 4, nullptr, 0);
	if (err_ && log_)
		log_("dib8000: write(%u, 0x%04X) failed: %d\n", reg, val, err_);
}

std::uint16_t Dib8096::Rd(std::uint16_t reg)
{
	if (err_)
		return 0;

	const std::uint8_t w[2] = { static_cast<std::uint8_t>(reg >> 8), static_cast<std::uint8_t>(reg) };
	std::uint8_t r[2] = {};

	err_ = bus_.Transfer(kDemodAddr, w, 2, r, 2);
	if (err_ && log_)
		log_("dib8000: read(%u) failed: %d\n", reg, err_);
	return err_ ? 0 : static_cast<std::uint16_t>(r[0] << 8 | r[1]);
}

std::uint32_t Dib8096::Rd32(std::uint16_t reg)
{
	std::uint32_t hi = Rd(reg);

	return hi << 16 | Rd(reg + 1);
}

void Dib8096::Modify(std::uint16_t reg, std::uint16_t keep, std::uint16_t set)
{
	std::uint16_t v = Rd(reg);

	Wr(reg, (v & keep) | set);
}

void Dib8096::PreciseSleepMs(unsigned int ms)
{
	thread_local PreciseTimer timer;

	if (timer.handle) {
		LARGE_INTEGER due;

		due.QuadPart = -static_cast<LONGLONG>(ms) * 10000;
		if (SetWaitableTimer(timer.handle, &due, 0, nullptr, nullptr, FALSE) && WaitForSingleObject(timer.handle, INFINITE) == WAIT_OBJECT_0)
			return;
	}

	::Sleep(ms);
}

void Dib8096::FlushBus()
{
	/* 溜めた書き込みの失敗は、送り終えたここで初めて分かる */
	int r = bus_.Flush();

	if (r && !err_) {
		err_ = r;
		if (log_)
			log_("dib8096: deferred i2c error: %d\n", r);
	}
}

void Dib8096::Sleepms(unsigned int ms)
{
	if (err_)
		return;

	/* 溜めた書き込みを先に送る。待ち (wait_) の中でほかの準備を進めてよいが、I2C は使わない */
	FlushBus();
	if (err_)
		return;

	if (wait_)
		wait_(ms);
	else
		PreciseSleepMs(ms);
}

void Dib8096::WriteTable(const std::uint16_t *t)
{
	/* [個数 n, 先頭レジスタ r, 値 × n] の繰り返し。n == 0 で終了 */
	while (*t) {
		unsigned int n = *t++;
		unsigned int r = *t++;

		for (unsigned int i = 0; i < n; i++)
			Wr(static_cast<std::uint16_t>(r + i), *t++);
	}
}

void Dib8096::OpenGate()
{
	if (!gate_open_) {
		Wr(1025, 0xC000);
		gate_open_ = (err_ == 0);
	}
}

void Dib8096::TWr(std::uint8_t reg, std::uint16_t val)
{
	OpenGate();
	if (err_)
		return;

	const std::uint8_t b[3] = { reg, static_cast<std::uint8_t>(val >> 8), static_cast<std::uint8_t>(val) };

	err_ = bus_.Transfer(kTunerAddr, b, 3, nullptr, 0);
	if (err_ && log_)
		log_("dib0090: write(0x%02X, 0x%04X) failed: %d\n", reg, val, err_);
}

std::uint16_t Dib8096::TRd(std::uint8_t reg)
{
	OpenGate();
	if (err_)
		return 0;

	std::uint8_t r[2] = {};

	err_ = bus_.Transfer(kTunerAddr, &reg, 1, r, 2);
	if (err_ && log_)
		log_("dib0090: read(0x%02X) failed: %d\n", reg, err_);
	return err_ ? 0 : static_cast<std::uint16_t>(r[0] << 8 | r[1]);
}

void Dib8096::TWrs(std::uint8_t reg, const std::uint16_t *vals, int n)
{
	for (int i = 0; i < n; i++)
		TWr(static_cast<std::uint8_t>(reg + i), vals[i]);
}

void Dib8096::TableTWr(const std::uint16_t *t)
{
	while (*t) {
		unsigned int n = *t++;
		unsigned int r = *t++;

		for (unsigned int i = 0; i < n; i++)
			TWr(static_cast<std::uint8_t>(r + i), *t++);
	}
}

/* ------------------------------------------------------------------ */
/* DiB8000                                                            */
/* ------------------------------------------------------------------ */

int Dib8096::Identify()
{
	Rd(896);		/* 電源投入後の最初の読み出しは化けることがある */
	std::uint16_t v = Rd(896);
	if (err_)
		return err_;
	if (v != 0x01B3) {
		if (log_)
			log_("dib8000: unexpected vendor id 0x%04X\n", v);
		return -ENODEV;
	}

	std::uint16_t dev = Rd(897);
	if (err_)
		return err_;
	if (dev != 0x8000 && dev != 0x8001 && dev != 0x8002) {
		if (log_)
			log_("dib8000: unsupported device id 0x%04X\n", dev);
		return -ENODEV;
	}

	revision_ = dev;
	return 0;
}

void Dib8096::SetPowerMode(bool all)
{
	std::uint16_t r774 = 0x3FFF, r775 = 0xFFFF, r776 = 0xFFFF;
	std::uint16_t r900 = (Rd(900) & 0xFFFC) | 0x0003;
	std::uint16_t r1280 = (Rd(1280) & 0x00FF) | 0xFF00;

	if (all) {
		r774 = r775 = r776 = 0;
		r900 &= 0xFFFC;
		r1280 &= 0x00FF;
	} else {
		r1280 &= 0x00FF;
	}

	Wr(774, r774);
	Wr(775, r775);
	Wr(776, r776);
	Wr(900, r900);
	Wr(1280, r1280);
}

void Dib8096::SetAdcState(AdcState s)
{
	std::uint16_t r907 = Rd(907), r908 = Rd(908);

	switch (s) {
	case ADC_SLOW_ON:
		r908 |= 3;
		Wr(908, r908);
		r908 &= ~2;
		break;
	case ADC_SLOW_OFF:
		r908 |= 3;
		break;
	case ADC_ON:
		r907 &= 0x0FFF;
		r908 &= 0x0003;
		break;
	case ADC_OFF:
		r907 = 0x3000;
		r908 = 0x007A;
		break;
	}

	Wr(907, r907);
	Wr(908, r908);
}

void Dib8096::ResetPll()
{
	/* dib8090_pll_config_12mhz: prediv 1, ratio 18, range 3, reset 1, bypass 0, ioClkEnCore 1, adcClkSrc 1, modulo 2 */
	Wr(901, 1 << 8 | 18);
	std::uint16_t clk = 1 << 10 | 0 << 9 | 1 << 8 | 0 << 5 | 0 << 4 | 1 << 3 | 3 << 1 | 1;
	Wr(902, clk);
	clk = (clk & 0xFFF7) | 0 << 3;
	Wr(902, clk);
	Wr(904, 0 << 15 | 1 << 12 | (3 & 3) << 10 | 2 << 8 | 1 << 7);

	/* resetPLLCommon: internal 54000kHz */
	Wr(23, ((54000u * 1000) >> 16) & 0xFFFF);
	Wr(24, (54000u * 1000) & 0xFFFF);
	Wr(27, 0);
	Wr(28, 0);
	Wr(26, 0);
	Wr(922, 0xD257);
}

void Dib8096::UpdatePll(int ratio)
{
	std::uint16_t r1856 = Rd(1856);
	unsigned int prediv = r1856 & 0x3F, loopdiv = (r1856 >> 6) & 0x3F;

	if (prediv == 1 && loopdiv == 18)
		return;
	if (ratio != 0)
		Wr(901, static_cast<std::uint16_t>(1 << 8 | ratio));
}

void Dib8096::SetBandwidth()
{
	std::uint32_t timf = timf_ ? timf_ : timf_default_;

	Wr(29, static_cast<std::uint16_t>(timf >> 16));
	Wr(30, static_cast<std::uint16_t>(timf & 0xFFFF));
}

void Dib8096::CtrlTimf(int op, std::uint32_t timf)
{
	if (op == 0) {
		timf_ = timf;
	} else if (op == 2) {
		timf_ = Rd32(435);
		Wr(29, static_cast<std::uint16_t>(timf_ >> 16));
		Wr(30, static_cast<std::uint16_t>(timf_ & 0xFFFF));
	}
	SetBandwidth();
}

void Dib8096::SadCalibrate()
{
	Wr(923, 0);
	Wr(924, 776);
	Wr(923, 1);
	Wr(923, 0);
	Sleepms(1);
}

void Dib8096::SetWbdRef(int value)
{
	if (value < 0)
		value = 0;
	if (value > 4095)
		value = 4095;
	wbd_ref_ = static_cast<std::uint16_t>(value);
	Wr(106, wbd_ref_);
}

void Dib8096::SetOutputModeRaw(OutputMode mode)
{
	std::uint16_t fifo_threshold = 1792, sram = 0x0205, outreg = 0;
	std::uint16_t smo = (Rd(299) & 0x0050) | 0x0002;

	switch (mode) {
	case OutputMode::PARALLEL_GATED: outreg = 0x0400; break;
	case OutputMode::PARALLEL_CONT: outreg = 0x0440; break;
	case OutputMode::SERIAL: outreg = 0x0480; break;
	case OutputMode::DIVERSITY: outreg = 0x0500; sram &= 0xFDFF; break;
	case OutputMode::FIFO: smo |= 3 << 1; fifo_threshold = 512; outreg = 0x0540; break;
	case OutputMode::HIGH_Z: outreg = 0; break;
	case OutputMode::ANALOG_ADC: outreg = 0x04C0; Modify(298, 0xFFFF, 0x09); break;
	}

	smo |= 1 << 5;		/* 188 バイトの MPEG2 TS */
	Wr(299, smo);
	Wr(300, fifo_threshold);
	Wr(1286, outreg);
	Wr(1291, sram);
}

void Dib8096::ResetGpio()
{
	Wr(1029, 0xFFFF);
	Wr(1030, 0x0000);
	Wr(1032, 0xFFFF);
	Wr(1037, 0);
}

void Dib8096::SetGpioRaw(unsigned int pin, int dir, int val)
{
	std::uint16_t d = Rd(1029);

	d = static_cast<std::uint16_t>((d & ~(1 << pin)) | (dir & 1) << pin);
	Wr(1029, d);

	std::uint16_t v = Rd(1030);

	v = static_cast<std::uint16_t>((v & ~(1 << pin)) | (val & 1) << pin);
	Wr(1030, v);
}

void Dib8096::Reset()
{
	Identify();
	Wr(1287, 0x0003);
	Wr(1025, 0x0100);
	gate_open_ = false;
	Wr(1028, 0);
	SetPowerMode(true);
	SetAdcState(ADC_OFF);

	Wr(770, 0xFFFF); Wr(771, 0xFFFF); Wr(772, 0xFFFC);
	Wr(898, 0x000C);
	Wr(1280, 0x004D); Wr(1281, 0x000C);
	Wr(770, 0x0000); Wr(771, 0x0000); Wr(772, 0x0000);
	Wr(898, 0x0004);
	Wr(1280, 0x0000); Wr(1281, 0x0000);

	Wr(906, 0x2D98);
	ResetPll();
	Wr(898, 0x0004);
	ResetGpio();
	SetOutputModeRaw(OutputMode::HIGH_Z);
	current_agc_ = 0;
	Wr(40, 0x0755);

	WriteTable(dib8000Defaults);
	isdbt_cfg_loaded_ = false;
	Wr(903, 0x31);
	Modify(1285, 0xFFFD, 0);
	SetBandwidth();
	SetAdcState(ADC_SLOW_ON);
	SadCalibrate();
	SetAdcState(ADC_SLOW_OFF);
	Modify(285, 0xFF9F, 3 << 5);
	SetPowerMode(false);
	Rd(565);
}

void Dib8096::Wakeup()
{
	SetPowerMode(true);
	SetAdcState(ADC_ON);
	SetAdcState(ADC_SLOW_ON);
}

void Dib8096::SleepChip()
{
	SetOutputModeRaw(OutputMode::HIGH_Z);
	SetPowerMode(false);
	SetAdcState(ADC_SLOW_OFF);
	SetAdcState(ADC_OFF);
}

int Dib8096::SetAgcConfig(int band)
{
	if (current_band_ == band && current_agc_)
		return 0;

	current_band_ = band;
	if (!(band & (BAND_UHF | BAND_VHF | BAND_LBAND | BAND_SBAND | BAND_CBAND)))
		return -EINVAL;

	/* dib8090_agc_config。CBAND だけ agc1Max が 0 */
	const std::uint16_t agc1_max = (band == BAND_CBAND) ? 0 : 65535;

	current_agc_ = 1;
	Wr(76, 5 << 11 | 3 << 5 | 5 << 1);
	Wr(77, 787);
	Wr(78, 10);
	Wr(101, 0 << 12 | 118);
	Wr(102, 28 << 5 | 26);
	Wr(103, 31 << 6 | 51);
	Wr(106, wbd_ref_ ? wbd_ref_ : 3530);
	Wr(107, 5 << 9 | 0 << 8);
	Wr(108, agc1_max);
	Wr(109, 0);
	Wr(110, 65535);
	Wr(111, 0);
	Wr(112, 0 << 8 | 32);
	Wr(113, 143 << 8 | 144);
	Wr(114, 114 << 8 | 227);
	Wr(115, 116 << 8 | 117);
	Wr(75, 114);
	Modify(923, 0xFFE3, 0 << 4 | 1 << 2);
	return err_;
}

void Dib8096::RestartAgc()
{
	Wr(770, 0x0A00);
	Wr(770, 0);
}

unsigned int Dib8096::AgcStartup()
{
	unsigned int ret = 0;
	const int band = BandOfFrequency(channel_.frequency / 1000);

	switch (tune_state_) {
	case AGC_START:
		SetAdcState(ADC_ON);
		if (SetAgcConfig(band)) {
			tune_state_ = AGC_STOP;
			if (!err_)
				err_ = -EINVAL;
			break;
		}
		ret = 70;
		tune_state_ = AGC_STEP0;
		break;
	case AGC_STEP0:
		TWr(0x04, 0);		/* dccFreq(restart = 1) */
		RestartAgc();
		ret = 50;
		tune_state_ = AGC_STEP1;
		break;
	case AGC_STEP1:
		ret = 70;
		tune_state_ = AGC_STEP2;
		break;
	case AGC_STEP2:
		TWr(0x04, 1);		/* dccFreq(0) と setDCServo(1) */
		TWr(0x04, 1);
		ret = 0;
		tune_state_ = AGC_STOP;
		break;
	default:
		tune_state_ = AGC_STOP;
		break;
	}

	return ret;
}

void Dib8096::SetDiversityIn(int onoff)
{
	std::uint16_t sync_wait = Rd(273) & 0xFFF0;

	if (!differential_constellation_) {
		Wr(272, 1 << 9);
		Wr(273, sync_wait | 1 << 2 | 2);
	} else {
		Wr(272, 0);
		Wr(273, sync_wait);
	}

	diversity_on_off_ = onoff;
	switch (onoff) {
	case 0: Wr(270, 1); Wr(271, 0); break;
	case 1: Wr(270, 6); Wr(271, 6); break;
	case 2: Wr(270, 0); Wr(271, 1); break;
	}

	/*
	 * 同じ設定の繰り返しでも、この追加の書き込みと 30ms の待ちを省くとロックしない選局が出る
	 * (実機で、省いた場合に 18 回中 1 回ロックしなかった)。
	 */
	if (revision_ == 0x8002) {
		std::uint16_t t = Rd(903);

		Wr(903, t & ~(1 << 3));
		Sleepms(30);
		Wr(903, t | 1 << 3);
	}
}

void Dib8096::LoadAnaFeCoefs(const std::int16_t *table)
{
	if (!isdbt_cfg_loaded_) {
		for (int m = 0; m < 24; m++)
			Wr(static_cast<std::uint16_t>(117 + m), static_cast<std::uint16_t>(table[m]));
	}
}

int Dib8096::SetLayer(unsigned int i, int max_constellation)
{
	const Layer &l = channel_.layer[i];
	int constellation, cr;

	switch (l.modulation) {
	case MOD_DQPSK: constellation = 0; break;
	case MOD_QPSK: constellation = 1; break;
	case MOD_QAM16: constellation = 2; break;
	default: constellation = 3; break;
	}

	switch (l.fec) {
	case FEC_1_2: cr = 1; break;
	case FEC_2_3: cr = 2; break;
	case FEC_3_4: cr = 3; break;
	case FEC_5_6: cr = 5; break;
	default: cr = 7; break;
	}

	int time_intlv = Fls(l.interleaving);

	if (time_intlv > 3)
		time_intlv = 0;

	Wr(static_cast<std::uint16_t>(2 + i), static_cast<std::uint16_t>(constellation << 10 | (l.segment_count & 0xF) << 6 | cr << 3 | time_intlv));

	if (l.segment_count > 0) {
		if ((max_constellation == MOD_DQPSK || max_constellation == MOD_QPSK) && (l.modulation == MOD_QAM16 || l.modulation == MOD_QAM64))
			max_constellation = l.modulation;
		else if (max_constellation == MOD_QAM16 && l.modulation == MOD_QAM64)
			max_constellation = l.modulation;
	}

	return max_constellation;
}

void Dib8096::SetIsdbtCommonChannel(unsigned int seq, bool autosearching)
{
	Channel &c = channel_;

	if (autosearching)
		c.partial_reception = 1;

	Wr(10, static_cast<std::uint16_t>(seq << 4));
	mode_ = (c.transmission_mode == TM_2K) ? 1 : (c.transmission_mode == TM_4K) ? 2 : 3;
	Wr(1, static_cast<std::uint16_t>((Rd(1) & 0xFFFC) | (c.guard_interval & 3)));
	Modify(274, 0xFFCF, static_cast<std::uint16_t>((c.partial_reception & 1) << 5 | 0 << 4));

	if (c.partial_reception) {
		seg_diff_mask_ = static_cast<std::uint16_t>((c.layer[0].modulation == MOD_DQPSK) << permuSeg[0]);
		int nbseg_diff = 0;

		for (int i = 1; i < 3; i++)
			nbseg_diff += (c.layer[i].modulation == MOD_DQPSK) * c.layer[i].segment_count;
		for (int i = 0; i < nbseg_diff && i + 1 < 13; i++)
			seg_diff_mask_ |= 1 << permuSeg[i + 1];
	} else {
		int nbseg_diff = 0;

		for (int i = 0; i < 3; i++)
			nbseg_diff += (c.layer[i].modulation == MOD_DQPSK) * c.layer[i].segment_count;
		for (int i = 0; i < nbseg_diff && i < 13; i++)
			seg_diff_mask_ |= 1 << permuSeg[i];
	}

	if (seg_diff_mask_ != 0)
		Modify(268, 0xF9FF, 0x0200);
	else
		Wr(268, 2 << 9 | 39);

	int max_constellation = MOD_DQPSK;

	for (unsigned int i = 0; i < 3; i++)
		max_constellation = SetLayer(i, max_constellation);

	Wr(0, static_cast<std::uint16_t>(mode_ << 13 | seg_diff_mask_));
	differential_constellation_ = (seg_diff_mask_ != 0);

	/* adpFineTune と updateAnaGain */
	const std::uint16_t *adp;
	int ana_gain;

	if (max_constellation == MOD_QAM64) { ana_gain = 7; adp = adpQ64; }
	else if (max_constellation == MOD_QAM16) { ana_gain = 7; adp = adpQ16; }
	else { ana_gain = 0; adp = adpQDefault; }

	for (int i = 0; i < 4; i++)
		Wr(static_cast<std::uint16_t>(215 + i), adp[i]);

	Wr(116, static_cast<std::uint16_t>(ana_gain));
	for (int i = 0; i < 10; i++)
		Wr(static_cast<std::uint16_t>(80 + i), static_cast<std::uint16_t>(ana_gain ? adcTarget16dB[i] : adcTarget16dB[i] - 355));

	LoadAnaFeCoefs(c.partial_reception ? anaFECoeff3Seg : anaFECoeff1Seg);

	/* set13SegChannel */
	seg_mask_ = 0x1FFF;
	if (!isdbt_cfg_loaded_) {
		Wr(180, 16 << 6 | 9);
		Wr(187, 4 << 12 | 8 << 5 | 0x2);
		for (int i = 0; i < 6; i++)
			Wr(static_cast<std::uint16_t>(181 + i), 0x2800);
		Wr(338, 1 << 12 | 1 << 10 | 0 << 9 | 3 << 5 | 1);
		Wr(340, 8 << 6 | 6);
		Wr(341, 4 << 3 | 1 << 2 | 1 << 1 | 1);
		Wr(228, 0);
		Wr(265, 31);
		Wr(205, 0x200F);
	}
	Wr(266, static_cast<std::uint16_t>(~seg_mask_ | seg_diff_mask_ | 0x40));
	LoadAnaFeCoefs(anaFECoeff13Seg);

	/* smallFineTune */
	Wr(352, seg_diff_mask_);
	Wr(353, seg_mask_);
	Wr(351, 0 << 9 | 0 << 8 | 13 << 4 | 5);
	Wr(1, static_cast<std::uint16_t>(0x0FFF << 2 | (Rd(1) & 3)));

	/* 端のセグメント */
	std::uint16_t left = 0, right = 0;
	const std::uint16_t enabled = seg_mask_ & ~seg_diff_mask_;

	for (int i = 0; i < 13; i++) {
		if (!(~seg_diff_mask_ & (1 << i)))
			continue;
		if (i == 0 || !(enabled & (1 << (i - 1))))
			left += 1 << i;
		if (i == 12 || !(enabled & (1 << (i + 1))))
			right += 1 << i;
	}

	Wr(222, left);
	Wr(223, right);
	Wr(189, static_cast<std::uint16_t>(~seg_mask_ | seg_diff_mask_));
	Wr(192, static_cast<std::uint16_t>(~seg_mask_ | seg_diff_mask_));
	Wr(225, static_cast<std::uint16_t>(~seg_mask_ | seg_diff_mask_));
	Wr(288, autosearching ? 0x1FFF : static_cast<std::uint16_t>((~seg_mask_ | seg_diff_mask_) & 0x1FFF));
	Wr(211, seg_mask_ & ~seg_diff_mask_);
	Wr(287, static_cast<std::uint16_t>(~seg_mask_ | 0x1000));
	Wr(178, 32);

	unsigned int tmcc_pow = 0;

	for (int i = 0; i < 3; i++)
		tmcc_pow += ((c.layer[i].modulation == MOD_DQPSK) * 4 + 1) * c.layer[i].segment_count;
	tmcc_pow *= 1 << (9 - 2);
	Wr(290, static_cast<std::uint16_t>(tmcc_pow));
	Wr(291, static_cast<std::uint16_t>(tmcc_pow));
	Wr(292, static_cast<std::uint16_t>(tmcc_pow));

	if (!isdbt_cfg_loaded_)
		Wr(250, 3285);
	isdbt_cfg_loaded_ = false;
}

void Dib8096::WaitLock(unsigned int w0, unsigned int w1, unsigned int w2, std::uint32_t &last)
{
	const unsigned int w[3] = { w0, w1, w2 };
	const unsigned int regs[3] = { 11, 13, 15 };

	for (int i = 0; i < 3; i++) {
		std::uint32_t value = 54000u * w[i];

		Wr(static_cast<std::uint16_t>(regs[i]), static_cast<std::uint16_t>(value >> 16));
		Wr(static_cast<std::uint16_t>(regs[i] + 1), static_cast<std::uint16_t>(value & 0xFFFF));
		last = value;
	}
}

void Dib8096::AutosearchStart()
{
	/* revision 0x8002: 2 段階の探索 */
	if (autosearch_state_ == SEARCH_FFT) {
		std::uint32_t value;

		Wr(37, 0x0065);
		Wr(116, 0x0000);
		Modify(0, 0x1FFF, 0 << 13 | 1 << 15);
		Modify(1, 0xFFFC, 0);
		Wr(6, 0);
		Wr(7, 0);
		Wr(8, 0);
		Modify(10, 0x0200, 16 << 4 | 0);
		WaitLock(20, 20, 20, value);
		Wr(17, 0);
		Wr(18, 200);
		Wr(19, 0);
		Wr(20, 400);
		Wr(21, static_cast<std::uint16_t>(value >> 16));
		Wr(22, static_cast<std::uint16_t>(value & 0xFFFF));
		Modify(32, 0xF0FF, 9 << 8);
		Wr(355, 2);
		Wr(356, 0);
		Wr(357, 0x111);
		Modify(770, 0xDFFF, 1 << 13);
		Modify(770, 0xDFFF, 0 << 13);
		Modify(0, 0x07FF, 0 << 15 | 1 << 13);
	} else if (autosearch_state_ == SEARCH_GUARD) {
		std::uint32_t value;

		channel_.transmission_mode = found_nfft_;
		channel_.guard_interval = GI_1_8;
		channel_.inversion = 0;
		channel_.layer[0] = { FEC_2_3, MOD_QAM64, 0, 13 };
		SetIsdbtCommonChannel(16, true);
		Wr(6, 0x0004);
		Wr(7, 0x0008);
		Wr(8, 0x1000);
		WaitLock(50, 200, 1000, value);
		Wr(355, 3);
		Wr(356, 0);
		Wr(357, 0x000F);
		std::uint16_t v = Rd(0);
		Wr(0, 1 << 15 | v);
		Rd(1284);
		Wr(0, v);
	}
}

unsigned int Dib8096::AutosearchIrq()
{
	std::uint16_t r = Rd(1284);

	if (autosearch_state_ == SEARCH_FFT) {
		if (r & 1)
			return 3;
	} else if (r & 1) {
		return 1;
	}

	if (r & 2)
		return 2;
	return 0;
}

void Dib8096::ViterbiState(bool on)
{
	std::uint16_t r = Rd(771);

	Wr(771, on ? (r & 0xFFFD) : (r | 0x0002));
}

void Dib8096::SetIsdbtLoopParams(bool tune1)
{
	std::uint16_t reg32, reg37;

	if (tune1) {
		reg32 = static_cast<std::uint16_t>((9 - mode_) << 12 | 6 << 8 | 0x80);
		reg37 = static_cast<std::uint16_t>(3 << 5 | 0 << 4 | (8 - mode_));
	} else {
		reg32 = static_cast<std::uint16_t>((11 - mode_) << 12 | 6 << 8 | 0x80);
		reg37 = static_cast<std::uint16_t>((5 + mode_) << 5 | (10 - mode_));
	}

	Wr(32, reg32);
	Wr(37, reg37);
}

void Dib8096::SetSyncWait()
{
	unsigned int sync_wait = (channel_.transmission_mode == TM_8K) ? 256 : (channel_.transmission_mode == TM_4K) ? 128 : 64;

	sync_wait = sync_wait * (1u << channel_.guard_interval) * 3 / 2 + 48;
	Modify(273, 0x000F, static_cast<std::uint16_t>(sync_wait << 4));
}

void Dib8096::SetDds(std::int32_t offset_khz)
{
	const std::uint32_t abs_khz = static_cast<std::uint32_t>(offset_khz < 0 ? -offset_khz : offset_khz);
	std::uint32_t dds = 0;		/* ifreq == 0 */
	const unsigned int invert = 0;
	std::int32_t unit = 67108864 / 54000;

	if (offset_khz < 0)
		unit = -unit;

	dds = invert ? dds - abs_khz * unit : dds + abs_khz * unit;
	if (abs_khz <= 54000 / 2) {
		Wr(26, invert);
		Wr(27, static_cast<std::uint16_t>((dds >> 16) & 0x1FF));
		Wr(28, static_cast<std::uint16_t>(dds & 0xFFFF));
	}
}

void Dib8096::SetFrequencyOffset()
{
	std::int32_t current_rf = static_cast<std::int32_t>(TunerGetFrequency() / 1000);

	SetDds(current_rf - static_cast<std::int32_t>(channel_.frequency / 1000));
}

void Dib8096::GetFrontend()
{
	Channel &c = channel_;

	c.bandwidth_hz = 6000000;
	if (!(ReadStatusFlags() & STATUS_SYNC))
		return;

	Rd(508);
	std::uint16_t v = Rd(570);

	c.inversion = (v & 0x40) >> 6;
	switch ((v & 0x30) >> 4) {
	case 1: c.transmission_mode = TM_2K; break;
	case 2: c.transmission_mode = TM_4K; break;
	default: c.transmission_mode = TM_8K; break;
	}
	c.guard_interval = v & 3;
	c.partial_reception = Rd(505) & 1;

	for (int i = 0; i < 3; i++) {
		c.layer[i].segment_count = Rd(static_cast<std::uint16_t>(493 + i)) & 0x0F;
		c.layer[i].interleaving = Rd(static_cast<std::uint16_t>(499 + i)) & 3;
		if (c.layer[i].interleaving == 3)
			c.layer[i].interleaving = 4;
		switch (Rd(static_cast<std::uint16_t>(481 + i)) & 7) {
		case 1: c.layer[i].fec = FEC_1_2; break;
		case 2: c.layer[i].fec = FEC_2_3; break;
		case 3: c.layer[i].fec = FEC_3_4; break;
		case 5: c.layer[i].fec = FEC_5_6; break;
		default: c.layer[i].fec = FEC_7_8; break;
		}
		switch (Rd(static_cast<std::uint16_t>(487 + i)) & 3) {
		case 0: c.layer[i].modulation = MOD_DQPSK; break;
		case 1: c.layer[i].modulation = MOD_QPSK; break;
		case 2: c.layer[i].modulation = MOD_QAM16; break;
		default: c.layer[i].modulation = MOD_QAM64; break;
		}
	}
}

unsigned int Dib8096::TuneStateMachine()
{
	unsigned int ret = 1;
	std::uint16_t locks;

	switch (tune_state_) {
	case DEMOD_START:
		Rd(565);
		status_ = ST_PENDING;
		channel_parameters_set_ = manual_mode_;		/* 記録済みのパラメーターがあれば手動モード */
		ViterbiState(false);
		Modify(285, 0x0060, 0);
		SetFrequencyOffset();
		SetBandwidth();
		found_nfft_ = TM_AUTO;
		found_guard_ = GI_AUTO;
		if (manual_mode_) {
			autosearch_state_ = SEARCH_DONE;
			tune_state_ = DEMOD_STEP3;
		} else {
			autosearch_state_ = SEARCH_FFT;
			tune_state_ = DEMOD_SEARCH_NEXT;
		}
		symbol_duration_ = 1;
		break;

	case DEMOD_SEARCH_NEXT:
		AutosearchStart();
		ret = 15;
		tune_state_ = DEMOD_STEP1;
		break;

	case DEMOD_STEP1:
		switch (AutosearchIrq()) {
		case 1:
			status_ = ST_FAILED;
			autosearch_state_ = SEARCH_DONE;
			tune_state_ = DEMOD_STOP;
			break;
		case 2:
			status_ = ST_FFT_SUCCESS;
			tune_state_ = DEMOD_STEP3;
			if (autosearch_state_ == SEARCH_GUARD)
				tune_state_ = DEMOD_STEP2;
			else
				autosearch_state_ = SEARCH_DONE;
			break;
		case 3:
			tune_state_ = DEMOD_STEP2;
			break;
		default:
			break;
		}
		break;

	case DEMOD_STEP2:
		if (autosearch_state_ == SEARCH_FFT) {
			std::uint32_t corm[3];

			corm[2] = static_cast<std::uint32_t>(Rd(594)) << 16 | Rd(595);
			corm[1] = static_cast<std::uint32_t>(Rd(596)) << 16 | Rd(597);
			corm[0] = static_cast<std::uint32_t>(Rd(598)) << 16 | Rd(599);

			int max = 0;

			for (int i = 1; i < 3; i++)
				if (corm[i] > corm[max])
					max = i;
			found_nfft_ = (max == 0) ? TM_2K : (max == 1) ? TM_4K : TM_8K;
			tune_state_ = DEMOD_SEARCH_NEXT;
			autosearch_state_ = SEARCH_GUARD;
			ret = 10;
		} else if (autosearch_state_ == SEARCH_GUARD) {
			found_guard_ = Rd(570) & 3;
			tune_state_ = DEMOD_STEP3;
		} else {
			status_ = ST_FAILED;
			autosearch_state_ = SEARCH_DONE;
			tune_state_ = DEMOD_STOP;
		}
		break;

	case DEMOD_STEP3:
		SetIsdbtLoopParams(true);
		SetIsdbtCommonChannel(0, false);
		tune_state_ = DEMOD_STEP4;
		break;

	case DEMOD_STEP4:
		Wr(770, 0x4000);
		Wr(770, 0);
		SetSyncWait();
		SetDiversityIn(diversity_on_off_);
		locks = (Rd(180) >> 6) & 0x3F;
		timeout_us_ = NowUs() + 2ull * locks * symbol_duration_ * 100;
		tune_state_ = DEMOD_STEP5;
		break;

	case DEMOD_STEP5:
		locks = Rd(568);
		if (locks & (3 << 11)) {
			CtrlTimf(2, 0);
			if (!differential_constellation_) {
				timeout_us_ = NowUs() + 20ull * ((Rd(188) >> 5) & 0x1F) * symbol_duration_ * 100;
				tune_state_ = DEMOD_STEP7;
			} else {
				tune_state_ = DEMOD_STEP8;
			}
		} else if (NowUs() > timeout_us_) {
			tune_state_ = DEMOD_STEP6;
		}
		break;

	case DEMOD_STEP6:
		ViterbiState(true);
		SetIsdbtLoopParams(false);
		tune_state_ = DEMOD_STOP;
		status_ = ST_FAILED;
		break;

	case DEMOD_STEP7:
		locks = Rd(568);
		if (locks & (1 << 10)) {
			ret = 14;
			tune_state_ = DEMOD_STEP8;
		} else if (NowUs() > timeout_us_) {
			tune_state_ = DEMOD_STEP6;
		}
		break;

	case DEMOD_STEP8:
		ViterbiState(true);
		SetIsdbtLoopParams(false);
		tune_state_ = DEMOD_STEP9;
		status_ = ST_LOCKED;
		break;

	case DEMOD_STEP9:
		if ((Rd(1291) >> 9) & 1) {
			int deeper = 0;

			for (int i = 0; i < 3; i++) {
				if (channel_.layer[i].interleaving >= deeper && channel_.layer[i].segment_count > 0) {
					deeper = channel_.layer[0].interleaving;	/* Linux どおり layer[0] の値を使う */
					longest_intlv_layer_ = i;
				}
			}

			int n = (deeper == 0) ? 2 : (deeper == 3) ? 8 : 2 * deeper;

			if (diversity_on_off_)
				n *= 2;
			timeout_us_ = NowUs() + 200000ull * n;
			tune_state_ = DEMOD_STEP10;
		} else {
			tune_state_ = DEMOD_STOP;
		}
		break;

	case DEMOD_STEP10:
		locks = Rd(568);
		/*
		 * いずれかの階層が MPEG ロックした時点で抜ける。最も深いインターリーブの階層を待つと、
		 * 受信できる階層があっても、そのデインターリーバが満ちるまで約 0.4 秒余計にかかる。
		 * 待ち時間切れの判定 (下) が既に「いずれかの階層」であることと揃えている。
		 */
		if (locks & (7 << 5)) {
			status_ = ST_DATA_LOCKED;
			tune_state_ = DEMOD_STOP;
		} else if (NowUs() > timeout_us_) {
			status_ = (locks & (7 << 5)) ? ST_DATA_LOCKED : ST_FAILED;
			tune_state_ = DEMOD_STOP;
		}
		break;

	default:
		tune_state_ = DEMOD_STOP;
		break;
	}

	if (tune_state_ == DEMOD_STOP)
		ret = 0;
	if (ret > 0 && tune_state_ > DEMOD_STEP3)
		return ret * symbol_duration_;
	if (ret > 0 && ret < symbol_duration_)
		return symbol_duration_;
	return ret;
}

int Dib8096::SetFrontend()
{
	last_tune_manual_ = manual_mode_;

	int r = SetFrontendOnce();
	const bool locked = (status_ == ST_LOCKED || status_ == ST_DATA_LOCKED);

	if (!r && manual_mode_ && !locked) {
		/* 記録したパラメーターでロックしなかった。記録を捨てて自動探索でやり直す */
		if (log_)
			log_("dib8000: stored lock parameters failed for %u Hz. searching again.\n", channel_.frequency);
		if (locks_.erase(channel_.frequency))
			locks_dirty_ = true;
		manual_mode_ = false;
		last_tune_manual_ = false;

		const std::uint32_t freq = channel_.frequency;

		std::memset(&channel_, 0, sizeof(channel_));
		channel_.frequency = freq;
		channel_.transmission_mode = TM_AUTO;
		channel_.guard_interval = GI_AUTO;
		channel_.inversion = 2;
		channel_.layer_enabled = 7;
		for (auto &l : channel_.layer) {
			l.fec = FEC_AUTO;
			l.modulation = MOD_AUTO;
		}
		r = SetFrontendOnce();
	}

	if (!r && !manual_mode_ && (status_ == ST_LOCKED || status_ == ST_DATA_LOCKED)) {
		/* 自動探索でロックした値を覚える。自動探索の結果は GetFrontend() が channel_ へ取り込んでいる */
		LockRecord rec = {};

		rec.frequency = channel_.frequency;
		rec.transmission_mode = channel_.transmission_mode;
		rec.guard_interval = channel_.guard_interval;
		rec.inversion = channel_.inversion;
		rec.partial_reception = channel_.partial_reception;
		for (int i = 0; i < 3; i++) {
			rec.fec[i] = channel_.layer[i].fec;
			rec.modulation[i] = channel_.layer[i].modulation;
			rec.interleaving[i] = channel_.layer[i].interleaving;
			rec.segment_count[i] = channel_.layer[i].segment_count;
		}

		auto it = locks_.find(rec.frequency);

		if (it == locks_.end() || std::memcmp(&it->second, &rec, sizeof(rec))) {
			locks_[rec.frequency] = rec;
			locks_dirty_ = true;
		}
	}

	return r;
}

std::vector<Dib8096::LockRecord> Dib8096::ExportLocks() const
{
	std::vector<LockRecord> v;

	for (const auto &kv : locks_)
		v.push_back(kv.second);
	return v;
}

void Dib8096::ImportLocks(const std::vector<LockRecord> &records)
{
	for (const auto &r : records) {
		/* 不正な値が入り込んだファイルを使わないよう、範囲を確かめる */
		bool ok = (r.transmission_mode == TM_2K || r.transmission_mode == TM_8K || r.transmission_mode == TM_4K) &&
			r.guard_interval >= GI_1_32 && r.guard_interval <= GI_1_4 && (r.inversion == 0 || r.inversion == 1);

		for (int i = 0; i < 3 && ok; i++)
			ok = r.segment_count[i] >= 0 && r.segment_count[i] <= 13 && r.interleaving[i] >= 0 && r.interleaving[i] <= 4 &&
				(r.modulation[i] == MOD_DQPSK || (r.modulation[i] >= MOD_QPSK && r.modulation[i] <= MOD_QAM64)) &&
				r.fec[i] >= FEC_NONE && r.fec[i] <= FEC_7_8;
		if (ok)
			locks_[r.frequency] = r;
	}
}

int Dib8096::SetFrontendOnce()
{
	channel_.bandwidth_hz = 6000000;
	SetDiversityIn(1);
	SetOutputModeRaw(OutputMode::HIGH_Z);

	int r = TunerSetParams();
	if (r)
		return r;

	tune_state_ = AGC_START;
	SetDiversityIn(0);

	while (!err_) {
		unsigned int wait = AgcStartup();

		if (wait == 0 || err_)
			break;
		wait = 10 * (wait + 99) / 100;
		Sleepms(wait);
		if (tune_state_ == AGC_STOP)
			break;
	}

	tune_state_ = DEMOD_START;
	const std::uint64_t give_up = NowUs() + 5000000;
	bool active = true;

	while (active && !err_) {
		const int executed = tune_state_;
		unsigned int delay = TuneStateMachine();
		const bool have_cb = (delay != 0);
		const std::uint64_t cb = NowUs() + static_cast<std::uint64_t>(delay) * 100;

		if (!channel_parameters_set_ && status_ == ST_FFT_SUCCESS) {
			GetFrontend();
			channel_parameters_set_ = true;
		}

		if (status_ == ST_FAILED || status_ == ST_LOCKED || status_ == ST_DATA_LOCKED)
			active = (tune_state_ != DEMOD_STOP);
		if (active && !have_cb)
			active = false;
		if (active) {
			const std::uint64_t now = NowUs();
			std::uint64_t wait_us = (cb > now) ? cb - now : 0;

			/*
			 * 待ちの下限。ステージ 4〜8 は、書き込み直後の再起動や、シンボル数で数えるタイムアウトを
			 * 10ms 周期の確認を前提に決めているため 10ms を保つ。それ以外は 3ms 周期で確認する。
			 */
			const std::uint64_t floor_us = (executed >= DEMOD_STEP4 && executed <= DEMOD_STEP8) ? 10000 : 3000;

			if (wait_us < floor_us)
				wait_us = floor_us;
			Sleepms(static_cast<unsigned int>((wait_us + 999) / 1000));
		}
		if (err_)
			return err_;
		if (active && NowUs() > give_up) {
			if (log_)
				log_("dib8000: tune did not finish in 5s\n");
			return -ETIMEDOUT;
		}
	}

	SetOutputModeRaw(output_mode_);
	FlushBus();
	return err_;
}

unsigned int Dib8096::ReadStatusFlags()
{
	std::uint16_t lock = Rd(568);
	unsigned int stat = 0;

	if ((lock >> 13) & 1) stat |= STATUS_SIGNAL;
	if ((lock >> 8) & 1) stat |= STATUS_CARRIER;
	if (((lock >> 1) & 0xF) == 0xF) stat |= STATUS_SYNC;
	if (((lock >> 12) & 1) && ((lock >> 5) & 7)) stat |= STATUS_LOCK;
	if ((lock >> 12) & 1) {
		for (int r = 554; r <= 556; r++)
			if (Rd(static_cast<std::uint16_t>(r)) & 1)
				stat |= STATUS_VITERBI;
	}
	return stat;
}

std::uint32_t Dib8096::GetSnr()
{
	std::uint16_t v = Rd(542);
	std::uint32_t n = (v >> 6) & 0xFF;
	int e = v & 0x3F;

	if (e & 0x20)
		e -= 0x40;
	n <<= ((e + 16) & 31);

	v = Rd(543);
	std::uint32_t s = (v >> 6) & 0xFF;

	e = v & 0x3F;
	if (e & 0x20)
		e -= 0x40;
	s <<= ((e + 16) & 31);

	if (n > 0) {
		std::uint32_t t = (s / n) << 16;

		return t + ((s << 16) - n * t) / n;
	}
	return 0xFFFFFFFFu;
}

std::int32_t Dib8096::GetAdcPower(bool db)
{
	std::int32_t val = static_cast<std::int32_t>(Rd32(384));

	if (!db)
		return val;
	if (val <= 0)
		return 0;

	int exp = Fls(val) - 1;
	std::int32_t mant = static_cast<std::int32_t>(static_cast<std::int64_t>(val) * 1000 / (1 << exp));
	std::uint8_t ix = static_cast<std::uint8_t>((mant - 1000) / 100);

	if (ix > 10)
		ix = 10;
	val = lut1000lnMant[ix] + 693 * (exp - 20) - 6908;
	return val * 256 / 1000;
}

int Dib8096::GetStatus(Status &st)
{
	err_ = 0;
	st.flags = ReadStatusFlags();

	std::uint32_t strength = 65535 - Rd(390);
	std::int64_t y;
	const std::size_t n = sizeof(strengthToDBTable) / sizeof(strengthToDBTable[0]);

	if (strength >= strengthToDBTable[0].x) {
		y = strengthToDBTable[0].y;
	} else if (strength < strengthToDBTable[n - 1].x) {
		y = strengthToDBTable[n - 1].y;
	} else {
		y = strengthToDBTable[n - 1].y;
		for (std::size_t i = 1; i < n; i++) {
			if (strengthToDBTable[i].x == strength) {
				y = strengthToDBTable[i].y;
				break;
			}
			if (strength > strengthToDBTable[i].x) {
				y = strengthToDBTable[i].y + (static_cast<std::int64_t>(strength) - strengthToDBTable[i].x) *
					(strengthToDBTable[i - 1].y - strengthToDBTable[i].y) / (static_cast<std::int64_t>(strengthToDBTable[i - 1].x) - strengthToDBTable[i].x);
				break;
			}
		}
	}
	st.strength_mdbm = static_cast<int>(y - 131000);

	st.cnr_valid = false;
	st.cnr_mdb = 0;
	if (st.flags & STATUS_LOCK) {
		std::uint32_t snr = GetSnr() >> 16;

		st.cnr_valid = true;
		st.cnr_mdb = snr ? static_cast<int>(10000.0 * std::log10(static_cast<double>(snr)) + 0.5) : 0;
	}

	FlushBus();
	return err_;
}

int Dib8096::GetLockFlags(unsigned int &flags)
{
	err_ = 0;

	std::uint16_t lock = Rd(568);

	flags = 0;
	if ((lock >> 13) & 1) flags |= STATUS_SIGNAL;
	if ((lock >> 8) & 1) flags |= STATUS_CARRIER;
	if (((lock >> 1) & 0xF) == 0xF) flags |= STATUS_SYNC;
	if (((lock >> 12) & 1) && ((lock >> 5) & 7)) flags |= STATUS_LOCK;

	FlushBus();
	return err_;
}

int Dib8096::SetOutputMode(OutputMode mode)
{
	err_ = 0;
	output_mode_ = mode;
	SetOutputModeRaw(mode);
	FlushBus();
	return err_;
}

int Dib8096::HoldOutput()
{
	err_ = 0;
	SetOutputModeRaw(OutputMode::HIGH_Z);
	FlushBus();
	return err_;
}

int Dib8096::SetGpio(unsigned int pin, bool high)
{
	err_ = 0;
	SetGpioRaw(pin, 0, high ? 1 : 0);
	FlushBus();
	return err_;
}

int Dib8096::Sleep()
{
	err_ = 0;
	TunerSleepLine(1);
	SleepChip();
	asleep_ = true;
	FlushBus();
	return err_;
}

int Dib8096::Initialize()
{
	err_ = 0;
	timf_default_ = 20199727;
	gate_open_ = false;

	int r = Identify();
	if (r)
		return r;

	Wr(1025, 0x0100);
	gate_open_ = false;
	Reset();
	Modify(285, 0xFF9F, 3 << 5);
	if (err_)
		return err_;

	tuner_ = Tuner();
	tuner_.bb1_def = 0;
	tuner_.rf_lt_def = 0x7C00;
	TunerReset();
	if (err_)
		return err_;

	Wakeup();
	TunerSleepLine(0);
	TWr(0x23, TRd(0x23) | 1 << 14);
	asleep_ = false;

	/* DC 校正は周波数に依存しない。初回の選局に含めず、開くときに済ませる */
	TunerCalibrateDc();

	output_mode_ = OutputMode::PARALLEL_GATED;
	SetOutputModeRaw(output_mode_);
	FlushBus();
	return err_;
}

/* ------------------------------------------------------------------ */
/* DiB0090                                                            */
/* ------------------------------------------------------------------ */

void Dib8096::TunerResetLine(int on)
{
	SetGpioRaw(5, 0, on == 0);
}

void Dib8096::TunerSleepLine(int on)
{
	SetGpioRaw(0, 0, on);
}

int Dib8096::TunerIdentify()
{
	std::uint16_t v = TRd(0x1A) & ~0x0800;

	tuner_.id = v;
	tuner_.version = v & 0xFF;

	const unsigned int product = (v >> 8) & 0xF;

	if (err_)
		return err_;
	if (product != 2 || (tuner_.version & 3) == 2) {
		if (log_)
			log_("dib0090: unsupported tuner 0x%04X\n", v);
		return -ENODEV;
	}

	switch ((tuner_.version >> 5) & 7) {
	case 1: case 4: case 6: case 7:
		break;
	default:
		return -ENODEV;
	}

	switch (tuner_.version & 0x1F) {
	case 0xF: case 0x7: tuner_.p1g = true; break;
	case 0x3: case 0x1: tuner_.p1g = false; break;
	default: return -ENODEV;
	}

	return 0;
}

void Dib8096::TunerResetDigital()
{
	TunerSleepLine(0);
	Sleepms(10);
	TunerResetLine(1);
	Sleepms(10);
	TunerResetLine(0);
	Sleepms(10);

	TWr(0x24, enPLL | enCrystal);
	TWr(0x1B, enDigClk | enPLL | enCrystal);
	TWr(0x20, (kAdcClockRatio - 1) << 11 | 0 << 10 | 1 << 9 | 1 << 8);
	TWr(0x23, 0 << 15 | 0 << 14 | 2 << 10 | 1 << 9 | 0 << 8 | 1 << 5 | 1 << 4);
	std::uint16_t pll_cfg = TRd(0x21);
	pll_cfg |= 1 << 15;
	TWr(0x21, pll_cfg);
}

void Dib8096::TunerSetEfuse()
{
	std::uint16_t e2 = TRd(0x26), e4 = TRd(0x28);

	if (tuner_.version == 0x3 || tuner_.version == 0x7 || e2 == 0xFFFF) {
		TWr(0x22, 0x10);
		int cal = (TRd(0x22) >> 6) & 0x3FF;

		if (cal < 670 || cal == 1023)
			cal = 850;
		int n = 165 - ((cal * 10) >> 6);

		e2 = static_cast<std::uint16_t>(3 << 12 | 34 << 6 | n);
		e4 = e2;
	}

	if (e2 != e4)
		e2 &= e4;

	if (e2 != 0xFFFF) {
		int c = e2 & 0x3F, n = (e2 >> 12) & 0xF, h = (e2 >> 6) & 0x3F;

		c = (c >= 40 || c <= 9) ? 32 : c + 14;
		h = (h >= 40 || h <= 25) ? 34 : h;
		n = (n >= 8 || n <= 0) ? 3 : n;
		TWr(0x13, static_cast<std::uint16_t>(h << 10));
		TWr(0x02, static_cast<std::uint16_t>(n << 11 | (h >> 2) << 6 | c));
	}
}

void Dib8096::TunerReset()
{
	TunerResetDigital();
	if (err_)
		return;
	if (TunerIdentify())
		return;

	if ((TRd(0x1A) >> 5) & 0x2)
		TWr(0x1B, enIQADC | enBB | enBias | enDigClk | enPLL | enCrystal);
	else
		TWr(0x1B, enDigClk | enPLL | enCrystal);

	TableTWr(dib0090Defaults);
	if (tuner_.p1g)
		TableTWr(dib0090P1GAdditionalDefaults);
	if ((tuner_.version & 0x1F) >= 0x3)
		TunerSetEfuse();

	TWr(0x14, 2);		/* 12MHz の水晶 */
	tuner_.calibrate = CAL_DC | CAL_WBD | CAL_TEMP;
}

void Dib8096::TunerSetBoost(bool on)
{
	tuner_.bb1_def = static_cast<std::uint16_t>((tuner_.bb1_def & 0xDFFF) | (on ? 1 << 13 : 0));
}

void Dib8096::TunerSetBandwidth()
{
	/* 6MHz */
	tuner_.bb1_def = static_cast<std::uint16_t>((tuner_.bb1_def & 0x3FFF) | 2 << 14);
	TWr(0x01, tuner_.bb1_def);
	TWr(0x03, 0x6008);
	TWr(0x04, 0x0001);
	TWr(0x02, 5 << 11 | 8 << 6 | 22);
	TWr(0x05, 0xABCD);
}

int Dib8096::WbdIndex(std::uint32_t mhz) const
{
	int i = 0;

	while (mhz > kWbdTable[i].maxFreq && kWbdTable[i].maxFreq != 0xFFFF)
		i++;
	return i;
}

unsigned int Dib8096::DcOffsetCalibration()
{
	Tuner &t = tuner_;
	const DcCalibration *dc = static_cast<const DcCalibration *>(t.dc);
	unsigned int ret = 10;

	auto set_trim = [&]() {
		std::uint16_t *val = (dc[t.dc_next].addr == 0x07) ? &t.bb7 : &t.bb6;

		*val &= ~(0x1F << dc[t.dc_next].offset);
		*val |= t.step << dc[t.dc_next].offset;
		TWr(dc[t.dc_next].addr, *val);
	};
	auto step0 = [&]() {
		TWr(0x01, dc[t.dc_next].bb1);
		TWr(0x07, t.bb7 | dc[t.dc_next].i << 7);
		t.step = 0;
		t.min_adc_diff = 1023;
		t.tune_state = TUNER_STEP1;
	};

	switch (t.tune_state) {
	case TUNER_START:
		t.bb6 = 0;
		t.bb7 = 0x040D;
		TWr(0x24, TRd(0x24) & 0x0FFB);
		t.wbdmux = TRd(0x10);
		TWr(0x10, (t.wbdmux & ~(0xFF << 3)) | 0x7 << 3 | 0x3);
		TWr(0x23, TRd(0x23) & ~(1 << 14));
		t.dc = t.p1g ? dcP1GTable : dcTable;
		dc = static_cast<const DcCalibration *>(t.dc);
		t.dc_next = 0;
		step0();
		ret = 50;
		break;
	case TUNER_STEP0:
		step0();
		ret = 50;
		break;
	case TUNER_STEP1:
		set_trim();
		t.tune_state = TUNER_STEP2;
		break;
	case TUNER_STEP2:
		TWr(0x1F, 0x7);
		t.tune_state = TUNER_STEP3;
		ret = 100;
		break;
	case TUNER_STEP3:
		t.adc_diff = TRd(0x1D);
		TWr(0x1F, 0x4);
		t.tune_state = TUNER_STEP4;
		ret = 100;
		break;
	case TUNER_STEP4:
		t.adc_diff = static_cast<std::int16_t>(t.adc_diff - TRd(0x1D));
		t.tune_state = TUNER_STEP5;
		ret = 0;
		break;
	case TUNER_STEP5: {
		auto steps = [](int u) { return u > 15 ? u - 16 : u; };
		const DcCalibration &row = dc[t.dc_next];

		if (t.step == 0 && t.adc_diff < 0)
			t.min_adc_diff = -1023;
		if (t.step == 0) {
			if (row.pga != 0 && t.adc_diff < 0)
				t.step = 0x10;
			if (row.pga == 0 && t.adc_diff > 0)
				t.step = 0x10;
		}
		if (((t.adc_diff < 0) == (t.min_adc_diff < 0)) && steps(t.step) < 15) {
			t.step++;
			t.min_adc_diff = t.adc_diff;
			t.tune_state = TUNER_STEP1;
		} else {
			if (std::abs(t.adc_diff) > std::abs(t.min_adc_diff))
				t.step--;
			set_trim();
			t.dc_next++;
			t.tune_state = (dc[t.dc_next].addr == 0) ? TUNER_STEP6 : TUNER_STEP0;
		}
		break;
	}
	case TUNER_STEP6:
		TWr(0x07, t.bb7 & ~0x0008);
		TWr(0x1F, 0x7);
		t.tune_state = TUNER_START;
		t.calibrate &= ~CAL_DC;
		break;
	}

	return ret;
}

void Dib8096::TunerCalibrateDc()
{
	tuner_.tune_state = TUNER_START;

	while (!err_ && (tuner_.calibrate & CAL_DC)) {
		unsigned int ret = TunerTune();

		if (!(tuner_.calibrate & CAL_DC))
			break;
		Sleepms(10 * (ret + 99) / 100);
	}

	tuner_.tune_state = TUNER_START;
}

unsigned int Dib8096::WbdCalibration()
{
	Tuner &t = tuner_;

	switch (t.tune_state) {
	case TUNER_START: {
		const WbdSlope &w = kWbdTable[WbdIndex(t.current_rf / 1000)];
		std::uint8_t gain = w.wbdGain ? w.wbdGain : 4;

		if (gain == t.wbd_cal_gain) {
			t.calibrate &= ~CAL_WBD;
			return 0;
		}
		TWr(0x10, 0x1B81 | 1 << 10 | gain << 13 | 1 << 3);
		TWr(0x24, (enUHF & 0x0FFF) | 1 << 1);
		t.tune_state = TUNER_STEP0;
		t.wbd_cal_gain = gain;
		return 90;
	}
	case TUNER_STEP0:
		t.wbd_offset = TRd(0x1D);
		t.tune_state = TUNER_START;
		t.calibrate &= ~CAL_WBD;
		return 10;
	}

	return 10;
}

unsigned int Dib8096::GetTemperature()
{
	Tuner &t = tuner_;

	switch (t.tune_state) {
	case TUNER_START:
		t.wbdmux = TRd(0x10);
		TWr(0x10, (t.wbdmux & ~(0xFF << 3)) | 0x8 << 3);
		t.bias = TRd(0x13);
		TWr(0x13, t.bias | 0x3 << 8);
		t.tune_state = TUNER_STEP0;
		return 15;
	case TUNER_STEP0:
		t.adc_diff = TRd(0x1D);
		TWr(0x13, (t.bias & ~(0x3 << 8)) | 0x2 << 8);
		t.tune_state = TUNER_STEP1;
		return 15;
	case TUNER_STEP1: {
		int val = TRd(0x1D);

		t.temperature = (((val - t.adc_diff) * 180) >> 8) + 55;
		t.tune_state = TUNER_STEP2;
		return 15;
	}
	case TUNER_STEP2:
		TWr(0x13, t.bias);
		TWr(0x10, t.wbdmux);
		t.tune_state = TUNER_START;
		t.calibrate &= ~CAL_TEMP;
		t.temp_time_us = NowUs();
		return 15;
	}

	return 15;
}

unsigned int Dib8096::CaptrimSearch()
{
	Tuner &t = tuner_;
	unsigned int ret = 10;

	switch (t.tune_state) {
	case TUNER_START:
		TWr(0x10, 0x02B1);
		TWr(0x1E, 0x0032);
		if (!t.tuned) {
			if (!t.p1g)
				t.step = t.captrim = t.fcaptrim = 64;
			t.current_rf = t.rf_request;
		} else if (!t.p1g) {
			t.step = 4;
			t.captrim = TRd(0x18) & 0x7F;
			t.fcaptrim = t.captrim;
		}
		t.adc_diff = 3000;
		t.tune_state = TUNER_STEP0;
		if (!t.p1g && !t.tuned) {
			/* 前回この周波数で決めた値を試し、ADC がほぼ同じなら二分探索を省く */
			auto it = t.captrim_cache.find(t.rf_request);

			if (it != t.captrim_cache.end()) {
				t.reuse = it->second;
				TWr(0x18, 0xE900 | t.reuse.captrim);
				t.tune_state = TUNER_STEP4;
			}
		}
		break;
	case TUNER_STEP4: {
		const int adc = TRd(0x1D);
		const int target = (t.rest == 0) ? 200 : 400;

		if (std::abs(adc - target) <= t.reuse.adc_diff + 50) {
			t.fcaptrim = t.reuse.captrim;
			t.tune_state = TUNER_STEP3;
		} else {
			t.captrim_cache.erase(t.rf_request);
			t.step = t.captrim = t.fcaptrim = 64;
			t.tune_state = TUNER_STEP0;
		}
		break;
	}
	case TUNER_STEP0:
		if (t.p1g) {
			TWr(0x40, 3 << 7 | 31 << 2 | 1 << 1 | 1);
			TRd(0x40);
			ret = 50;
		} else {
			t.step /= 2;
			TWr(0x18, 0xE900 | t.captrim);
		}
		t.tune_state = TUNER_STEP1;
		break;
	case TUNER_STEP1:
		if (t.p1g) {
			TWr(0x40, 0x018C);
			TRd(0x40);
			t.fcaptrim = TRd(0x18) & 0x7F;
			t.tune_state = TUNER_STEP3;
		} else {
			int adc = TRd(0x1D);
			int target = (t.rest == 0) ? 200 : 400;
			int d = std::abs(adc - target);
			int sign = (adc >= target) ? -1 : 1;

			if (d < t.adc_diff) {
				t.adc_diff = d;
				t.fcaptrim = t.captrim;
			}
			t.captrim += sign * t.step;
			t.tune_state = (t.step >= 1) ? TUNER_STEP0 : TUNER_STEP2;
			ret = 25;
		}
		break;
	case TUNER_STEP2:
		TWr(0x18, 0xE900 | t.fcaptrim);
		t.captrim_cache[t.rf_request] = { t.fcaptrim, t.adc_diff };
		t.tune_state = TUNER_STEP3;
		break;
	case TUNER_STEP3:
		t.calibrate &= ~CAL_CAPTRIM;
		t.tune_state = TUNER_STEP0;
		break;
	}

	return ret;
}

std::int32_t Dib8096::GetWbdTarget()
{
	Tuner &t = tuner_;
	const std::uint32_t f_mhz = channel_.frequency / 1000000;
	int temp = std::min(std::max(t.temperature, 0), 128);
	const WbdSlope &w = kWbdTable[WbdIndex(f_mhz)];

	t.wbdmux = static_cast<std::uint16_t>((t.wbdmux & ~(7 << 13)) | (w.wbdGain ? w.wbdGain : 4) << 13);
	TWr(0x10, t.wbdmux);

	std::int32_t thot = static_cast<std::int32_t>(static_cast<std::uint32_t>(w.offsetHot) - ((w.slopeHot * f_mhz) >> 6));
	std::int32_t tcold = static_cast<std::int32_t>(static_cast<std::uint32_t>(w.offsetCold) - ((w.slopeCold * f_mhz) >> 6));

	tcold += ((thot - tcold) * temp) >> 7;
	return static_cast<std::int32_t>(t.wbd_offset) + tcold;
}

unsigned int Dib8096::TunerTune()
{
	Tuner &t = tuner_;
	unsigned int ret = 10;

	if (t.tune_state == TUNER_START) {
		if (t.calibrate & (CAL_DC | CAL_TEMP | CAL_WBD))
			TWr(0x23, TRd(0x23) & ~(1 << 14));
	}

	if (t.calibrate & CAL_DC)
		return DcOffsetCalibration();
	if (t.calibrate & CAL_WBD) {
		if (t.current_rf == 0)
			t.current_rf = channel_.frequency / 1000;
		return WbdCalibration();
	}
	if (t.calibrate & CAL_TEMP)
		return GetTemperature();
	if (t.calibrate & CAL_CAPTRIM)
		return CaptrimSearch();

	switch (t.tune_state) {
	case TUNER_START: {
		int band = BandOfFrequency(channel_.frequency / 1000);
		const int offset = (band == BAND_UHF) ? kFreqOffsetUHF : kFreqOffsetVHF;
		const std::uint32_t rf_request = static_cast<std::uint32_t>(static_cast<std::int32_t>(channel_.frequency / 1000) + offset);

		t.rf_request = rf_request;
		t.band = band;

		if (t.current_rf != rf_request || t.current_standard != 8) {
			t.tuned = false;
			t.current_rf = 0;
			t.current_standard = 0;

			const TuningEntry *tune_table;
			const PllEntry *pll_table;
			const int pkg = (t.version >> 5) & 7;

			tune_table = t.p1g ? dib0090P1GTuningTable : dib0090TuningTable;
			if ((pkg == 4 || pkg == 7) && (band == BAND_FM || band == BAND_CBAND || band == BAND_VHF)) {
				band = BAND_CBAND;
				tune_table = t.p1g ? dib0090P1GTuningTableFMVHFOnCBand : dib0090TuningTableFMVHFOnCBand;
			}
			pll_table = t.p1g ? dib0090P1GPLLTable : dib0090PLLTable;

			const TuningEntry *tune = tune_table;
			while (rf_request > tune->maxFreq && tune->maxFreq != 900000 && tune->maxFreq != 0)
				tune++;
			const PllEntry *pll = pll_table;
			while (rf_request > pll->maxFreq && pll->maxFreq != 860000)
				pll++;
			t.tune_entry = tune;
			t.pll_entry = pll;

			TWr(0x0B, 0xB800 | tune->switchTrim);

			const std::uint32_t vcof = (pll->hfdiv * rf_request) * 2;
			const std::uint32_t f_ref = kClockKHz / kFRefClockRatio;
			std::uint32_t fb_div = vcof / pll->topresc / f_ref;
			std::uint32_t rest = vcof / pll->topresc - fb_div * f_ref;
			const std::uint32_t lpf = 100;

			if (rest < lpf) rest = 0;
			else if (rest < 2 * lpf) rest = 2 * lpf;
			else if (rest > f_ref - lpf) { rest = 0; fb_div++; }
			else if (rest > f_ref - 2 * lpf) rest = f_ref - 2 * lpf;
			rest = (rest * 6528) / (f_ref / 10);
			t.rest = rest;

			std::uint16_t lo5, lo6;

			if (rest == 0)
				lo5 = pll->vcoBand ? 0x049F : 0x041F;
			else
				lo5 = pll->vcoBand ? 0x049E : 0x041D;	/* analogOutput */
			if (t.p1g)
				lo5 = 0x042C;
			lo5 |= pll->hfdivCode << 11 | pll->vcoBand << 7;

			lo6 = (t.p1g || rest == 0) ? 0xFFF8 : 0xFF28;
			std::uint32_t den = 1;

			if (rest > 0) {
				lo6 |= 1 << 2 | 2;
				den = 255;
			}

			TWr(0x15, static_cast<std::uint16_t>(fb_div));
			TWr(0x16, static_cast<std::uint16_t>(den << 8 | kFRefClockRatio));
			TWr(0x17, static_cast<std::uint16_t>(rest));
			TWr(0x19, lo5);
			TWr(0x1C, lo6);
			lo6 = tune->tunerEnable;
			lo6 = (lo6 & 0xFF9F) | 0x2;
			TWr(0x24, lo6 | enLO | enCrystal);
		}
		t.current_rf = rf_request;
		t.current_standard = 8;
		ret = 20;
		t.calibrate = CAL_CAPTRIM;
		break;
	}
	case TUNER_STEP0: {
		const TuningEntry *tune = static_cast<const TuningEntry *>(t.tune_entry);
		const WbdSlope &w = kWbdTable[WbdIndex(t.current_rf / 1000)];

		TWr(0x1E, 0x07FF);
		std::uint8_t gain = w.wbdGain ? w.wbdGain : 4;

		t.wbdmux = static_cast<std::uint16_t>(gain << 13 | 3 << 11 | (0x0781 | 1 << 1));
		TWr(0x10, t.wbdmux);
		if (tune->tunerEnable == enCAB && t.p1g) {
			TWr(0x09, tune->lnaBias);
			TWr(0x0B, 0xB800 | tune->lnaTune << 6 | tune->switchTrim);
		} else {
			TWr(0x09, static_cast<std::uint16_t>(tune->lnaTune << 5 | tune->lnaBias));
		}
		TWr(0x0C, tune->v2i);
		TWr(0x0D, tune->mix);
		TWr(0x0E, tune->load);
		t.tune_state = TUNER_STEP1;
		break;
	}
	case TUNER_STEP1:
		t.rf_lt_def = 0x7C00;
		TunerSetBandwidth();
		t.tuned = true;
		/* 温度は短時間では変わらないため、60 秒以内の測定値を使い回す */
		t.calibrate |= CAL_WBD;
		if (!t.temp_time_us || NowUs() - t.temp_time_us > 60000000ull)
			t.calibrate |= CAL_TEMP;
		t.tune_state = TUNER_STOP;
		return kNever;
	default:
		return kNever;
	}

	return ret;
}

int Dib8096::TunerSetParams(std::uint32_t frequency_hz)
{
	(void)frequency_hz;
	tuner_.tune_state = TUNER_START;

	while (!err_) {
		unsigned int ret = TunerTune();

		if (ret == kNever || err_)
			break;
		Sleepms(10 * (ret + 99) / 100);
		if (tuner_.tune_state == TUNER_STOP)
			break;
	}

	return err_;
}

void Dib8096::TunerSetRfRamp(const std::uint16_t *cfg)
{
	tuner_.rf_ramp = cfg;
	TWr(0x2A, 0xFFFF);
	TWrs(0x2C, cfg + 3, 6);
	TWrs(0x3E, cfg + 9, 2);
}

void Dib8096::TunerSetBbRamp(const std::uint16_t *cfg)
{
	tuner_.bb_ramp = cfg;
	TunerSetBoost(cfg[0] > 500);
	TWr(0x33, 0xFFFF);
	TWrs(0x35, cfg + 3, 4);
}

void Dib8096::TunerPwmGainReset()
{
	Tuner &t = tuner_;
	const std::uint16_t *rf_ramp = nullptr;

	switch (t.band) {
	case BAND_CBAND: rf_ramp = rfRampPWMCBand; break;
	case BAND_VHF: rf_ramp = rfRampPWMVHF; break;
	case BAND_UHF: rf_ramp = rfRampPWMUHF; break;
	}

	if (rf_ramp)
		TunerSetRfRamp(rf_ramp);
	TunerSetBbRamp(bbRampPWMNormal);

	unsigned int mux = 1;

	if (rf_ramp && ((t.rf_ramp && t.rf_ramp[0] == 0) || (t.band == BAND_CBAND && (t.version & 0x1F) <= 0x3)))
		mux = 0;
	TWr(0x32, static_cast<std::uint16_t>(mux << 12 | mux << 11));
	TWr(0x04, 1);
	TWr(0x39, 1 << 10);
}

int Dib8096::TunerSetParams()
{
	const std::uint32_t freq_hz = channel_.frequency;
	const int band = BandOfFrequency(freq_hz / 1000);

	SetGpioRaw(3, 0, band == BAND_UHF ? 0 : 1);

	int r = TunerSetParams(freq_hz);
	if (r)
		return r;

	if (channel_.bandwidth_hz != 6000000)
		return -EINVAL;

	UpdatePll(0);

	/* 復調器の PLL 比を、ADC クロックの高調波がチャンネルから遠くなるように選ぶ */
	int optimal = 20;
	std::uint32_t max = 0;

	for (int rr = 17; rr <= 20; rr++) {
		std::uint32_t freq_adc = 12 * rr * 256 / 16;
		std::uint32_t x = (freq_hz / 1000) * 256 / 1000;
		std::uint32_t q = x / freq_adc;
		std::uint32_t rest = x - q * freq_adc;

		if (rest > freq_adc / 2)
			rest = freq_adc - rest;
		if (rest > max && rest > 717) {
			optimal = rr;
			max = rest;
		}
	}

	std::uint32_t timf;

	switch (optimal) {
	case 17: timf = 21387946; break;
	case 18: timf = 20199727; break;
	case 19: timf = 19136583; break;
	default: timf = 18179756; break;
	}

	UpdatePll(optimal);
	CtrlTimf(0, timf);

	if (band != BAND_CBAND) {
		std::int32_t target = (GetWbdTarget() * 8 * 18 / 33 + 1) / 2;

		SetWbdRef(target);
		TunerPwmGainReset();
	} else {
		return -ENOTSUP;
	}

	return err_;
}

int Dib8096::Tune(std::uint32_t frequency_hz)
{
	err_ = 0;

	if (frequency_hz < 45000000 || frequency_hz > 860000000)
		return -EINVAL;
	if (BandOfFrequency(frequency_hz / 1000) == BAND_CBAND)
		return -ENOTSUP;

	if (asleep_) {
		Wakeup();
		TunerSleepLine(0);
		TWr(0x23, TRd(0x23) | 1 << 14);
		asleep_ = false;
	}

	std::memset(&channel_, 0, sizeof(channel_));
	channel_.frequency = frequency_hz;
	channel_.transmission_mode = TM_AUTO;
	channel_.guard_interval = GI_AUTO;
	channel_.inversion = 2;
	channel_.layer_enabled = 7;
	for (auto &l : channel_.layer) {
		l.fec = FEC_AUTO;
		l.modulation = MOD_AUTO;
		l.interleaving = 0;
		l.segment_count = 0;
	}

	auto lock = locks_.find(frequency_hz);

	manual_mode_ = (lock != locks_.end());
	if (manual_mode_) {
		const LockRecord &rec = lock->second;

		channel_.transmission_mode = rec.transmission_mode;
		channel_.guard_interval = rec.guard_interval;
		channel_.inversion = rec.inversion;
		channel_.partial_reception = rec.partial_reception;
		for (int i = 0; i < 3; i++) {
			channel_.layer[i].fec = rec.fec[i];
			channel_.layer[i].modulation = rec.modulation[i];
			channel_.layer[i].interleaving = rec.interleaving[i];
			channel_.layer[i].segment_count = rec.segment_count[i];
		}
	}

	int r = SetFrontend();

	FlushBus();
	return r ? r : err_;
}

} // namespace px4
