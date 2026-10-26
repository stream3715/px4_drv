// SPDX-License-Identifier: GPL-2.0-only
//
// DiB8096 (DiB8000 demodulator + DiB0090 tuner) tables.
//
// Copyright (C) 2009 DiBcom (http://www.dibcom.fr/)
// Copyright (C) 2005-9 DiBcom (http://www.dibcom.fr/)
//
// The values are those of the Linux media subsystem's dib8000.c and dib0090.c
// (GPL-2.0-only), as carried by coax's internal/coaxd/demod/dib8096/tables.go.
// This file was converted from that file mechanically; see 09-tables.md.
//
// dib8000.c's ISDB-Tsb tables are left out with the paths that use them, and
// dib0090.c's LBAND and SBAND rows with the symbols that file never defines.
#pragma once
#include <cstdint>

// Enable bits of DiB0090 registers 0x1b and 0x24 (see 08-dib0090.md).
enum : uint16_t {
    enLNA0 = 0x8000, enLNA1 = 0x4000, enLNA2 = 0x2000, enLNA3 = 0x1000,
    enMix0 = 0x0800, enMix1 = 0x0400, enMix2 = 0x0200, enMix3 = 0x0100,
    enIQADC = 0x0040, enPLL = 0x0020, enTX = 0x0010, enBB = 0x0008,
    enLO = 0x0004, enBias = 0x0001, enIQANA = 0x0002, enDigClk = 0x0080,
    enCrystal = 0x0002,
    enUHF = 0x22E9, enVHF = 0x44E9, enLBD = 0x11E9, enSBD = 0x44E9, enCAB = 0x88E9,
};

struct LinearSegment { uint32_t x; int32_t y; };
// for every frequency <= maxFreq (kHz)
struct TuningEntry { uint32_t maxFreq; uint8_t switchTrim; uint8_t lnaTune; uint16_t lnaBias;
                     uint16_t v2i; uint16_t mix; uint16_t load; uint16_t tunerEnable; };
struct PllEntry { uint32_t maxFreq; uint8_t vcoBand; uint8_t hfdivCode; uint8_t hfdiv; uint8_t topresc; };
// maxFreq in MHz
struct WbdSlope { uint16_t maxFreq; uint16_t slopeCold; uint16_t offsetCold;
                  uint16_t slopeHot; uint16_t offsetHot; uint8_t wbdGain; };
struct DcCalibration { uint8_t addr; uint8_t offset; uint8_t pga; uint16_t bb1; uint8_t i; };

// anaFECoeff3Seg is ana_fe_coeff_3seg.
static const int16_t anaFECoeff3Seg[24] = {
	81, 80, 78, 74, 68, 61, 54, 45, 37, 28, 19, 11, 4, 1022, 1017, 1013, 1010, 1008, 1008, 1008, 1008, 1010, 1014, 1017,
};

// anaFECoeff1Seg is ana_fe_coeff_1seg.
static const int16_t anaFECoeff1Seg[24] = {
	249, 226, 164, 82, 5, 981, 970, 988, 1018, 20, 31, 26, 8, 1012, 1000, 1018, 1012, 8, 15, 14, 9, 3, 1017, 1003,
};

// anaFECoeff13Seg is ana_fe_coeff_13seg.
static const int16_t anaFECoeff13Seg[24] = {
	396, 305, 105, -51, -77, -12, 41, 31, -11, -30, -11, 14, 15, -2, -13, -7, 5, 8, 1, -6, -7, -3, 0, 1,
};

// dib8000Defaults is dib8000_defaults: runs of a count, the first register,
// then the values, ended by a zero count.
static const uint16_t dib8000Defaults[] = {
	// auto search configuration - lock0 by default waiting
	// for cpil_lock; lock1 cpil_lock; lock2 tmcc_sync_lock
	3, 7,
	0x0004,
	0x0400,
	0x0814,

	12, 11,
	0x001b,
	0x7740,
	0x005b,
	0x8d80,
	0x01c9,
	0xc380,
	0x0000,
	0x0080,
	0x0000,
	0x0090,
	0x0001,
	0xd4c0,

	// 1, 32,
	//	0x6680 // P_corm_thres Lock algorithms configuration

	11, 80, // set ADC level to -16
	(1 << 13) - 825 - 117,
	(1 << 13) - 837 - 117,
	(1 << 13) - 811 - 117,
	(1 << 13) - 766 - 117,
	(1 << 13) - 737 - 117,
	(1 << 13) - 693 - 117,
	(1 << 13) - 648 - 117,
	(1 << 13) - 619 - 117,
	(1 << 13) - 575 - 117,
	(1 << 13) - 531 - 117,
	(1 << 13) - 501 - 117,

	4, 108,
	0,
	0,
	0,
	0,

	1, 175,
	0x0410,
	1, 179,
	8192, // P_fft_nb_to_cut

	6, 181,
	0x2800, // P_coff_corthres_ ( 2k 4k 8k ) 0x2800
	0x2800,
	0x2800,
	0x2800, // P_coff_cpilthres_ ( 2k 4k 8k ) 0x2800
	0x2800,
	0x2800,

	2, 193,
	0x0666, // P_pha3_thres
	0x0000, // P_cti_use_cpe, P_cti_use_prog

	2, 205,
	0x200f, // P_cspu_regul, P_cspu_win_cut
	0x000f, // P_des_shift_work

	5, 215,
	0x023d, // P_adp_regul_cnt
	0x00a4, // P_adp_noise_cnt
	0x00a4, // P_adp_regul_ext
	0x7ff0, // P_adp_noise_ext
	0x3ccc, // P_adp_fil

	1, 230,
	0x0000, // P_2d_byp_ti_num

	1, 263,
	0x800, // P_equal_thres_wgn

	1, 268,
	(2 << 9) | 39, // P_equal_ctrl_synchro, P_equal_speedmode

	1, 270,
	0x0001, // P_div_lock0_wait
	1, 285,
	0x0020, // p_fec_
	1, 299,
	0x0062, // P_smo_mode, P_smo_rs_discard, P_smo_fifo_flush, P_smo_pid_parse, P_smo_error_discard

	1, 338,
	(1 << 12) | // P_ctrl_corm_thres4pre_freq_inh=1
		(1 << 10) |
		(0 << 9) | // P_ctrl_pre_freq_inh=0
		(3 << 5) | // P_ctrl_pre_freq_step=3
		(1 << 0), // P_pre_freq_win_len=1

	0,
};

// adcTarget16dB is adc_target_16dB.
static const uint16_t adcTarget16dB[11] = {
	7250, 7238, 7264, 7309, 7338, 7382, 7427, 7456, 7500, 7544, 7574,
};

// permuSeg is permu_seg, the order segments are counted from the centre.
static const uint8_t permuSeg[13] = {6, 5, 7, 4, 8, 3, 9, 2, 10, 1, 11, 0, 12};

// Channel estimation settings by densest constellation, adp_Q64, adp_Q16 and
// adp_Qdefault: P_adp_regul_cnt, P_adp_noise_cnt, P_adp_regul_ext,
// P_adp_noise_ext.
// (adp tables)
static const uint16_t adpQ64[4] = {0x0148, 0xfff0, 0x00a4, 0xfff8}; // 0.04, -0.002, 0.02, -0.001
static const uint16_t adpQ16[4] = {0x023d, 0xffdf, 0x00a4, 0xfff0}; // 0.07, -0.004, 0.02, -0.002
static const uint16_t adpQDefault[4] = {0x099a, 0xffae, 0x0333, 0xfff8}; // 0.3, -0.01, 0.1, -0.002


// strengthToDBTable is strength_to_db_table: signal strength in dBm (plus
// 131000, in thousandths) against the inverted AGC value. Linux measured it
// with a DTA-2111 generator on a dib8076 stick and puts its precision at about
// 0.5dB; on another board it is an estimate.
static const LinearSegment strengthToDBTable[] = {
	{55953, 108500}, // -22.5 dBm
	{55394, 108000},
	{53834, 107000},
	{52863, 106000},
	{52239, 105000},
	{52012, 104000},
	{51803, 103000},
	{51566, 102000},
	{51356, 101000},
	{51112, 100000},
	{50869, 99000},
	{50600, 98000},
	{50363, 97000},
	{50117, 96000}, // -35 dBm
	{49889, 95000},
	{49680, 94000},
	{49493, 93000},
	{49302, 92000},
	{48929, 91000},
	{48416, 90000},
	{48035, 89000},
	{47593, 88000},
	{47282, 87000},
	{46953, 86000},
	{46698, 85000},
	{45617, 84000},
	{44773, 83000},
	{43845, 82000},
	{43020, 81000},
	{42010, 80000}, // -51 dBm
	{0, 0},
};

// bbRampPWMNormal is bb_ramp_pwm_normal, the PWM ramp for all other Krosus.
static const uint16_t bbRampPWMNormal[] = {
	500, // max BB gain in 10th of dB
	8,   // ramp_slope = 1dB of gain -> clock_ticks_per_db = clk_khz / ramp_slope -> BB_RAMP2
	400,
	(2 << 9) | 0,   // BB_RAMP3 = 21dB
	(0 << 9) | 168, // BB_RAMP4
	(2 << 9) | 168, // BB_RAMP5 = 29dB
	(0 << 9) | 400, // BB_RAMP6
};

// bbRampPWMNormalSoCs is bb_ramp_pwm_normal_socs, the baseband ramp the
// software AGC takes for a P1G tuner in CBAND or VHF.
static const uint16_t bbRampPWMNormalSoCs[] = {
	550,          // max BB gain in 10th of dB
	(1 << 9) | 8, // ramp_slope = 1dB of gain -> clock_ticks_per_db = clk_khz / ramp_slope -> BB_RAMP2
	440,
	(4 << 9) | 0,   // BB_RAMP3 = 26dB
	(0 << 9) | 208, // BB_RAMP4
	(4 << 9) | 208, // BB_RAMP5 = 29dB
	(0 << 9) | 440, // BB_RAMP6
};

// rfRampPWMCBand7090P is rf_ramp_pwm_cband_7090p, the RF ramp the software
// AGC takes for a P1G tuner in CBAND or VHF.
static const uint16_t rfRampPWMCBand7090P[] = {
	280,              // max RF gain in 10th of dB
	18,               // ramp_slope = 1dB of gain -> clock_ticks_per_db = clk_khz / ramp_slope -> RF_RAMP2
	504,              // ramp_max = maximum X used on the ramp
	(29 << 10) | 364, // RF_RAMP5, LNA 1 = 8dB
	(0 << 10) | 504,  // RF_RAMP6, LNA 1
	(60 << 10) | 228, // RF_RAMP7, LNA 2 = 7.7dB
	(0 << 10) | 364,  // RF_RAMP8, LNA 2
	(34 << 10) | 109, // GAIN_4_1, LNA 3 = 6.8dB
	(0 << 10) | 228,  // GAIN_4_2, LNA 3
	(37 << 10) | 0,   // RF_RAMP3, LNA 4 = 6.2dB
	(0 << 10) | 109,  // RF_RAMP4, LNA 4
};

// lut1000lnMant is lut_1000ln_mant: 1000*ln of the mantissa 1.0 to 2.0 in
// tenths, for the ADC power in dB.
static const int32_t lut1000lnMant[] = {
	908, 7003, 7090, 7170, 7244, 7313, 7377, 7438, 7495, 7549, 7600,
};

// rfRampPWMCBand is rf_ramp_pwm_cband.
static const uint16_t rfRampPWMCBand[] = {
	314,              // max RF gain in 10th of dB
	33,               // ramp_slope = 1dB of gain -> clock_ticks_per_db = clk_khz / ramp_slope -> RF_RAMP2
	1023,             // ramp_max = maximum X used on the ramp
	(8 << 10) | 743,  // RF_RAMP3, LNA 1 = 0dB
	(0 << 10) | 1023, // RF_RAMP4, LNA 1
	(15 << 10) | 469, // RF_RAMP5, LNA 2 = 0dB
	(0 << 10) | 742,  // RF_RAMP6, LNA 2
	(9 << 10) | 234,  // RF_RAMP7, LNA 3 = 0dB
	(0 << 10) | 468,  // RF_RAMP8, LNA 3
	(9 << 10) | 0,    // GAIN_4_1, LNA 4 = 0dB
	(0 << 10) | 233,  // GAIN_4_2, LNA 4
};

// rfRampPWMVHF is rf_ramp_pwm_vhf.
static const uint16_t rfRampPWMVHF[] = {
	398,              // max RF gain in 10th of dB
	24,               // ramp_slope = 1dB of gain -> clock_ticks_per_db = clk_khz / ramp_slope -> RF_RAMP2
	954,              // ramp_max = maximum X used on the ramp
	(7 << 10) | 0,    // RF_RAMP3, LNA 1 = 13.2dB
	(0 << 10) | 290,  // RF_RAMP4, LNA 1
	(16 << 10) | 699, // RF_RAMP5, LNA 2 = 10.5dB
	(0 << 10) | 954,  // RF_RAMP6, LNA 2
	(17 << 10) | 580, // RF_RAMP7, LNA 3 = 5dB
	(0 << 10) | 699,  // RF_RAMP8, LNA 3
	(7 << 10) | 290,  // GAIN_4_1, LNA 4 = 12.5dB
	(0 << 10) | 580,  // GAIN_4_2, LNA 4
};

// rfRampPWMUHF is rf_ramp_pwm_uhf.
static const uint16_t rfRampPWMUHF[] = {
	398,              // max RF gain in 10th of dB
	24,               // ramp_slope = 1dB of gain -> clock_ticks_per_db = clk_khz / ramp_slope -> RF_RAMP2
	954,              // ramp_max = maximum X used on the ramp
	(7 << 10) | 0,    // RF_RAMP3, LNA 1 = 13.2dB
	(0 << 10) | 290,  // RF_RAMP4, LNA 1
	(16 << 10) | 699, // RF_RAMP5, LNA 2 = 10.5dB
	(0 << 10) | 954,  // RF_RAMP6, LNA 2
	(17 << 10) | 580, // RF_RAMP7, LNA 3 = 5dB
	(0 << 10) | 699,  // RF_RAMP8, LNA 3
	(7 << 10) | 290,  // GAIN_4_1, LNA 4 = 12.5dB
	(0 << 10) | 580,  // GAIN_4_2, LNA 4
};

// dib0090Defaults is dib0090_defaults, in dib8000Defaults' run format.
static const uint16_t dib0090Defaults[] = {
	25, 0x01,
	0x0000,
	0x99a0,
	0x6008,
	0x0000,
	0x8bcb,
	0x0000,
	0x0405,
	0x0000,
	0x0000,
	0x0000,
	0xb802,
	0x0300,
	0x2d12,
	0xbac0,
	0x7c00,
	0xdbb9,
	0x0954,
	0x0743,
	0x8000,
	0x0001,
	0x0040,
	0x0100,
	0x0000,
	0xe910,
	0x149e,

	1, 0x1c,
	0xff2d,

	1, 0x39,
	0x0000,

	2, 0x1e,
	0x07FF,
	0x0007,

	1, 0x24,
	enUHF | enCrystal,

	2, 0x3c,
	0x3ff,
	0x111,
	0,
};

// dib0090P1GAdditionalDefaults is dib0090_p1g_additionnal_defaults.
static const uint16_t dib0090P1GAdditionalDefaults[] = {
	1, 0x05,
	0xabcd,

	1, 0x11,
	0x00b4,

	1, 0x1c,
	0xfffd,

	1, 0x40,
	0x108,
	0,
};

// dcTable is dc_table: addr, trim register offset, pga, CTRL_BB1 value, I or Q.
static const DcCalibration dcTable[] = {
	// Step1 BB gain1= 26 with boost 1, gain 2 = 0
	{0x06, 5, 1, (1 << 13) | (0 << 8) | (26 << 3), 1},
	{0x07, 11, 1, (1 << 13) | (0 << 8) | (26 << 3), 0},
	// Step 2 BB gain 1 = 26 with boost = 1 & gain 2 = 29
	{0x06, 0, 0, (1 << 13) | (29 << 8) | (26 << 3), 1},
	{0x06, 10, 0, (1 << 13) | (29 << 8) | (26 << 3), 0},
	{},
};

// dcP1GTable is dc_p1g_table.
static const DcCalibration dcP1GTable[] = {
	// Step1 BB gain1= 26 with boost 1, gain 2 = 0
	{0x06, 5, 1, (1 << 13) | (0 << 8) | (15 << 3), 1},
	{0x07, 11, 1, (1 << 13) | (0 << 8) | (15 << 3), 0},
	// Step 2 BB gain 1 = 26 with boost = 1 & gain 2 = 29
	{0x06, 0, 0, (1 << 13) | (29 << 8) | (15 << 3), 1},
	{0x06, 10, 0, (1 << 13) | (29 << 8) | (15 << 3), 0},
	{},
};

// dib0090PLLTable is dib0090_pll_table.
static const PllEntry dib0090PLLTable[] = {
	// CBAND
	{56000, 0, 9, 48, 6},
	{70000, 1, 9, 48, 6},
	{87000, 0, 8, 32, 4},
	{105000, 1, 8, 32, 4},
	{115000, 0, 7, 24, 6},
	{140000, 1, 7, 24, 6},
	{170000, 0, 6, 16, 4},
	// VHF
	{200000, 1, 6, 16, 4},
	{230000, 0, 5, 12, 6},
	{280000, 1, 5, 12, 6},
	{340000, 0, 4, 8, 4},
	{380000, 1, 4, 8, 4},
	{450000, 0, 3, 6, 6},
	// UHF
	{580000, 1, 3, 6, 6},
	{700000, 0, 2, 4, 4},
	{860000, 1, 2, 4, 4},
};

// dib0090TuningTableFMVHFOnCBand is dib0090_tuning_table_fm_vhf_on_cband.
static const TuningEntry dib0090TuningTableFMVHFOnCBand[] = {
	// CBAND
	{184000, 4, 1, 15, 0x280, 0x2912, 0xb94e, enCAB},
	{227000, 4, 3, 15, 0x280, 0x2912, 0xb94e, enCAB},
	{380000, 4, 7, 15, 0x280, 0x2912, 0xb94e, enCAB},
	// UHF
	{520000, 2, 0, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{550000, 2, 2, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{650000, 2, 3, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{750000, 2, 5, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{850000, 2, 6, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{900000, 2, 7, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
};

// dib0090TuningTable is dib0090_tuning_table.
static const TuningEntry dib0090TuningTable[] = {
	// CBAND
	{170000, 4, 1, 15, 0x280, 0x2912, 0xb94e, enCAB},
	// VHF
	{184000, 1, 1, 15, 0x300, 0x4d12, 0xb94e, enVHF},
	{227000, 1, 3, 15, 0x300, 0x4d12, 0xb94e, enVHF},
	{380000, 1, 7, 15, 0x300, 0x4d12, 0xb94e, enVHF},
	// UHF
	{520000, 2, 0, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{550000, 2, 2, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{650000, 2, 3, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{750000, 2, 5, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{850000, 2, 6, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{900000, 2, 7, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
};

// dib0090P1GTuningTable is dib0090_p1g_tuning_table.
static const TuningEntry dib0090P1GTuningTable[] = {
	// CBAND
	{170000, 4, 1, 0x820f, 0x300, 0x2d22, 0x82cb, enCAB},
	// VHF
	{184000, 1, 1, 15, 0x300, 0x4d12, 0xb94e, enVHF},
	{227000, 1, 3, 15, 0x300, 0x4d12, 0xb94e, enVHF},
	{380000, 1, 7, 15, 0x300, 0x4d12, 0xb94e, enVHF},
	// UHF
	{510000, 2, 0, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{540000, 2, 1, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{600000, 2, 3, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{630000, 2, 4, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{680000, 2, 5, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{720000, 2, 6, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{900000, 2, 7, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
};

// dib0090P1GPLLTable is dib0090_p1g_pll_table.
static const PllEntry dib0090P1GPLLTable[] = {
	// CBAND
	{57000, 0, 11, 48, 6},
	{70000, 1, 11, 48, 6},
	{86000, 0, 10, 32, 4},
	{105000, 1, 10, 32, 4},
	{115000, 0, 9, 24, 6},
	{140000, 1, 9, 24, 6},
	{170000, 0, 8, 16, 4},
	// VHF
	{200000, 1, 8, 16, 4},
	{230000, 0, 7, 12, 6},
	{280000, 1, 7, 12, 6},
	{340000, 0, 6, 8, 4},
	{380000, 1, 6, 8, 4},
	{455000, 0, 5, 6, 6},
	// UHF
	{580000, 1, 5, 6, 6},
	{680000, 0, 4, 4, 4},
	{860000, 1, 4, 4, 4},
};

// dib0090P1GTuningTableFMVHFOnCBand is dib0090_p1g_tuning_table_fm_vhf_on_cband.
static const TuningEntry dib0090P1GTuningTableFMVHFOnCBand[] = {
	// CBAND
	{184000, 4, 3, 0x4187, 0x2c0, 0x2d22, 0x81cb, enCAB},
	{227000, 4, 3, 0x4187, 0x2c0, 0x2d22, 0x81cb, enCAB},
	{380000, 4, 3, 0x4187, 0x2c0, 0x2d22, 0x81cb, enCAB},
	// UHF
	{520000, 2, 0, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{550000, 2, 2, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{650000, 2, 3, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{750000, 2, 5, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{850000, 2, 6, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
	{900000, 2, 7, 15, 0x300, 0x1d12, 0xb9ce, enUHF},
};

// dib0090WBDTableDefault is dib0090_wbd_table_default, for a board that
// gives none. The DiB8096 board gives its own.
static const WbdSlope dib0090WBDTableDefault[] = {
	{470, 0, 250, 0, 100, 4},
	{860, 51, 866, 21, 375, 4},
	{1700, 0, 800, 0, 850, 4},
	{2900, 0, 250, 0, 100, 6},
	{0xFFFF, 0, 0, 0, 0, 0},
};
