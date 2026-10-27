// dib8096_i2c.cpp
//
// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2005-9 DiBcom, SA et al.
// DiB8096 の I2C (レジスター・チューナーのバスアクセス) と待機。

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

namespace {

#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

/* スレッドごとの高分解能タイマー。作れない環境 (Windows 10 1803 より前) では Sleep() にフォールバックする */
struct PreciseTimer final {
	PreciseTimer() : handle(CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS)) {}
	~PreciseTimer() { if (handle) CloseHandle(handle); }
	HANDLE handle;
};

} // namespace

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

} // namespace px4
