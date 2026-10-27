// dib8096_internal.h
//
// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2005-9 DiBcom, SA et al.
// dib8096*.cpp が共有する定数と補助関数。無名名前空間のため、翻訳単位ごとに内部リンケージで持つ。

#pragma once

#include <cstdint>

#include "dib8096.hpp"
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

inline int BandOfFrequency(std::uint32_t khz)
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

inline int Fls(int x)
{
	int r = 0;

	while (x) {
		r++;
		x >>= 1;
	}
	return r;
}

} // namespace

} // namespace px4
