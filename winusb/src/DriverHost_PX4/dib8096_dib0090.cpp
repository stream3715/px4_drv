// dib8096_dib0090.cpp
//
// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2005-9 DiBcom, SA et al.
// DiB0090 チューナーの制御と、選局 (Tune)。

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

#include "dib8096_internal.h"

namespace px4 {

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
