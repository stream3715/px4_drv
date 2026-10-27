// dib8096.cpp
//
// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2005-9 DiBcom, SA et al.
// DiB8000 復調器の制御と、DiB8096 全体の初期化・選局。

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

} // namespace px4
