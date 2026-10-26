// dib8096.hpp
//
// DiB8096 (DiB8000 ISDB-T 復調器 + DiB0090 チューナー) のドライバー。
// Linux メディアサブシステムの dib8000.c / dib0090.c / dibx000_common.c / dib0700_devices.c
// (DiBcom, GPL) の ISDB-T 13 セグメント経路の移植。
//
// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2005-9 DiBcom, SA et al.
//
// 地上デジタル放送 (UHF/VHF) の選局に必要な経路だけを持つ。170MHz 以下 (CBAND) のソフト AGC、
// ISDB-Tsb、ダイバーシティは対象外で、CBAND の選局は拒否する。

#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <map>
#include <vector>

namespace px4 {

class Dib8096 final {
public:
	/* ブリッジの I2C。スレーブは 8 ビット書式。0 を返せば成功、負の errno で失敗 */
	class Bus {
	public:
		virtual ~Bus() {}
		virtual int Transfer(std::uint8_t addr8, const std::uint8_t *w, std::size_t wlen, std::uint8_t *r, std::size_t rlen) = 0;
		/* 溜めた書き込みがあれば送る */
		virtual int Flush() { return 0; }
	};

	using LogFn = void (*)(const char *fmt, ...);
	/* 待ち。溜めた書き込みを送ってから ms ミリ秒待つ。待ち時間に他の準備を進めてよい */
	using WaitFn = std::function<void(unsigned int ms)>;

	enum class OutputMode : int {
		HIGH_Z = 0,
		PARALLEL_GATED = 1,
		PARALLEL_CONT = 2,
		DIVERSITY = 4,
		FIFO = 5,
		ANALOG_ADC = 6,
		SERIAL = 7,
	};

	enum : unsigned int {
		STATUS_SIGNAL = 0x01,
		STATUS_CARRIER = 0x02,
		STATUS_VITERBI = 0x04,
		STATUS_SYNC = 0x08,
		STATUS_LOCK = 0x10,
	};

	struct Status {
		unsigned int flags;	/* STATUS_* */
		int strength_mdbm;	/* 1/1000 dBm */
		int cnr_cdb;		/* 1/100 dB。ロック時のみ有効 */
		bool cnr_valid;
	};

	/*
	 * 既定の待ち。Sleep() は既定のタイマー精度 (約 15.6 ms) に丸められ、選局で 90 回前後ある数 ms の待ちが
	 * 合計で 0.5 秒ほど延びるため、高分解能タイマーで待つ。
	 */
	static void PreciseSleepMs(unsigned int ms);

	/*
	 * 一度ロックしたチャンネルの伝送パラメーター (モード、ガード間隔、各階層)。放送局ごとに固定のため、
	 * 次回の選局では自動探索 (FFT 探索とガード探索) を飛ばし、パラメーター設定から始められる。
	 * この値でロックしなかった場合は、記録を捨てて自動探索でやり直す。
	 */
	struct LockRecord {
		std::uint32_t frequency;	/* Hz */
		int transmission_mode;
		int guard_interval;
		int inversion;
		int partial_reception;
		int fec[3];
		int modulation[3];
		int interleaving[3];
		int segment_count[3];
	};

	Dib8096(Bus &bus, LogFn log, WaitFn wait);
	~Dib8096();

	Dib8096(const Dib8096 &) = delete;
	Dib8096& operator=(const Dib8096 &) = delete;

	/* ブリッジ側の DiB8096 リセットと、レジスタ 1287 への 3 の書き込みが済んでいること */
	int Initialize();
	int SetOutputMode(OutputMode mode);
	int SetGpio(unsigned int pin, bool high);
	int Tune(std::uint32_t frequency_hz);
	int GetStatus(Status &status);
	/* ロック状態だけの軽い読み出し (STATUS_SIGNAL/CARRIER/SYNC/LOCK) */
	int GetLockFlags(unsigned int &flags);
	int Sleep();

	/* ロック済みチャンネルの記録の出し入れ。記録が増減したときは locks_dirty() が true になる */
	std::vector<LockRecord> ExportLocks() const;
	void ImportLocks(const std::vector<LockRecord> &records);
	bool locks_dirty() const noexcept { return locks_dirty_; }
	void clear_locks_dirty() noexcept { locks_dirty_ = false; }

	/* 診断用 */
	std::uint16_t revision() const noexcept { return revision_; }
	std::uint16_t tuner_id() const noexcept { return tuner_.id; }

private:
	struct Layer {
		int fec;
		int modulation;
		int interleaving;
		int segment_count;
	};

	struct Channel {
		std::uint32_t frequency;
		std::uint32_t bandwidth_hz;
		int transmission_mode;
		int guard_interval;
		int inversion;
		int partial_reception;
		unsigned int layer_enabled;
		Layer layer[3];
	};

	struct Tuner {
		std::uint16_t id;
		std::uint8_t version;
		bool p1g;
		unsigned int calibrate;
		int tune_state;
		std::uint32_t current_rf;	/* kHz */
		int current_standard;
		bool tuned;
		std::uint32_t rest;
		const void *tune_entry;
		const void *pll_entry;
		std::uint16_t wbd_offset;
		std::uint8_t wbd_cal_gain;
		int temperature;
		std::uint16_t bb1_def;
		std::uint16_t rf_lt_def;
		const std::uint16_t *rf_ramp;
		const std::uint16_t *bb_ramp;
		/* 校正の作業変数 */
		int adc_diff;
		int captrim, fcaptrim, step;
		std::uint16_t wbdmux, bias;
		std::uint16_t bb6, bb7;
		const void *dc;
		int dc_next;
		int min_adc_diff;
		int band;
		std::uint32_t rf_request;
		/* 周波数ごとの captrim の再利用 */
		struct CaptrimEntry {
			int captrim;
			int adc_diff;
		};
		std::map<std::uint32_t, CaptrimEntry> captrim_cache;
		CaptrimEntry reuse;
		std::uint64_t temp_time_us;	/* 最後に温度を測った時刻。0 = 未測定 */
	};

	/* ---- I2C (復調器) ---- */
	void Wr(std::uint16_t reg, std::uint16_t val);
	std::uint16_t Rd(std::uint16_t reg);
	std::uint32_t Rd32(std::uint16_t reg);
	void Modify(std::uint16_t reg, std::uint16_t keep, std::uint16_t set);
	void Sleepms(unsigned int ms);
	void FlushBus();
	void WriteTable(const std::uint16_t *table);

	/* ---- チューナーの I2C (ゲート越し) ---- */
	void OpenGate();
	void TWr(std::uint8_t reg, std::uint16_t val);
	std::uint16_t TRd(std::uint8_t reg);
	void TWrs(std::uint8_t reg, const std::uint16_t *vals, int n);
	void TableTWr(const std::uint16_t *table);

	/* ---- 復調器 ---- */
	int Identify();
	void Reset();
	void SetPowerMode(bool all);
	enum AdcState { ADC_ON, ADC_OFF, ADC_SLOW_ON, ADC_SLOW_OFF };
	void SetAdcState(AdcState s);
	void ResetPll();
	void UpdatePll(int ratio);
	void SetBandwidth();
	void CtrlTimf(int op, std::uint32_t timf);
	void SadCalibrate();
	void SetWbdRef(int value);
	void SetOutputModeRaw(OutputMode mode);
	void ResetGpio();
	void SetGpioRaw(unsigned int pin, int dir, int val);
	void Wakeup();
	void SleepChip();
	int SetAgcConfig(int band);
	void RestartAgc();
	unsigned int AgcStartup();
	void SetDiversityIn(int onoff);
	int SetFrontend();
	int SetFrontendOnce();
	void SetIsdbtCommonChannel(unsigned int seq, bool autosearching);
	int SetLayer(unsigned int i, int max_constellation);
	void LoadAnaFeCoefs(const std::int16_t *table);
	void AutosearchStart();
	unsigned int AutosearchIrq();
	unsigned int TuneStateMachine();
	void SetFrequencyOffset();
	void SetDds(std::int32_t offset_khz);
	void GetFrontend();
	void ViterbiState(bool on);
	void SetIsdbtLoopParams(bool tune1);
	void SetSyncWait();
	unsigned int ReadStatusFlags();
	std::uint32_t GetSnr();
	std::int32_t GetAdcPower(bool db);
	void WaitLock(unsigned int w0, unsigned int w1, unsigned int w2, std::uint32_t &last);
	int TunerSetParams();
	std::uint64_t NowUs() const;

	/* ---- チューナー ---- */
	void TunerReset();
	void TunerResetDigital();
	int TunerIdentify();
	void TunerSetEfuse();
	void TunerSetBandwidth();
	void TunerSetBoost(bool on);
	void TunerPwmGainReset();
	void TunerSetRfRamp(const std::uint16_t *cfg);
	void TunerSetBbRamp(const std::uint16_t *cfg);
	int TunerSetParams(std::uint32_t frequency_hz);
	unsigned int TunerTune();
	unsigned int DcOffsetCalibration();
	unsigned int WbdCalibration();
	unsigned int GetTemperature();
	unsigned int CaptrimSearch();
	void TunerCalibrateDc();
	std::int32_t GetWbdTarget();
	std::uint32_t TunerGetFrequency() const { return tuner_.current_rf * 1000; }
	void TunerSleepLine(int on);
	void TunerResetLine(int on);
	int WbdIndex(std::uint32_t mhz) const;

	Bus &bus_;
	LogFn log_;
	WaitFn wait_;
	int err_;

	/* 復調器の状態 */
	std::uint16_t revision_;
	std::uint32_t timf_;
	std::uint32_t timf_default_;
	std::uint32_t pll_prediv_cached_;
	bool gate_open_;
	int current_band_;
	int current_agc_;
	std::uint16_t wbd_ref_;
	int diversity_on_off_;
	bool differential_constellation_;
	int mode_;
	std::uint16_t seg_diff_mask_;
	std::uint16_t seg_mask_;
	bool isdbt_cfg_loaded_;
	Channel channel_;
	OutputMode output_mode_;
	bool asleep_;
	bool channel_parameters_set_;
	int tune_state_;
	int autosearch_state_;
	int found_nfft_;
	int found_guard_;
	int status_;
	unsigned int symbol_duration_;
	int longest_intlv_layer_;
	std::uint64_t timeout_us_;

	Tuner tuner_;

	std::map<std::uint32_t, LockRecord> locks_;
	bool locks_dirty_;
	bool manual_mode_;
	bool last_tune_manual_;
};

} // namespace px4
