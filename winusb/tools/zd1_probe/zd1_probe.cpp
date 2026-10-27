// zd1_probe.cpp
//
// CECH-ZD1 J の bring-up 用コマンドライン。DriverHost_PX4 と同じ Mb86h47Bridge を使い、
// 起動手順を段階ごとに実行して結果を表示する。
//   zd1_probe <device-interface-guid> info
//   zd1_probe <device-interface-guid> start
//   zd1_probe <device-interface-guid> tune <物理チャンネル 13-62> [秒数] [出力.ts]

#include <cstdio>
#include <cstdarg>
#include <cstdint>
#include <string>
#include <vector>
#include <functional>

#include <windows.h>
#include <setupapi.h>
#include <winusb.h>
#include <initguid.h>

#include "mb86h47_bridge.hpp"
#include "dib8096.hpp"
#include "zd1_ts_framer.hpp"

static void Log(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	fflush(stdout);
}

static bool ParseGuid(const char *s, GUID &g)
{
	unsigned long d1;
	unsigned int d2, d3, b[8];

	if (*s == '{')
		s++;
	if (sscanf(s, "%8lx-%4x-%4x-%2x%2x-%2x%2x%2x%2x%2x%2x", &d1, &d2, &d3,
		&b[0], &b[1], &b[2], &b[3], &b[4], &b[5], &b[6], &b[7]) != 11)
		return false;

	g.Data1 = d1;
	g.Data2 = static_cast<USHORT>(d2);
	g.Data3 = static_cast<USHORT>(d3);
	for (int i = 0; i < 8; i++)
		g.Data4[i] = static_cast<UCHAR>(b[i]);
	return true;
}

class BridgeBus final : public px4::Dib8096::Bus {
public:
	explicit BridgeBus(px4::Mb86h47Bridge &b) : b_(b), count_(0) {}
	int Transfer(std::uint8_t addr8, const std::uint8_t *w, std::size_t wlen, std::uint8_t *r, std::size_t rlen) override
	{
		count_++;
		return b_.I2cAccess(addr8, w, wlen, r, rlen);
	}
	int Flush() override { return b_.I2cFlush(); }
	unsigned int count() const { return count_; }
private:
	px4::Mb86h47Bridge &b_;
	unsigned int count_;
};

static HANDLE OpenDevice(const GUID &guid, WINUSB_INTERFACE_HANDLE &winusb)
{
	HDEVINFO info = SetupDiGetClassDevsW(&guid, nullptr, nullptr, DIGCF_DEVICEINTERFACE | DIGCF_PRESENT);

	if (info == INVALID_HANDLE_VALUE)
		return INVALID_HANDLE_VALUE;

	SP_DEVICE_INTERFACE_DATA data = {};

	data.cbSize = sizeof(data);
	if (!SetupDiEnumDeviceInterfaces(info, nullptr, &guid, 0, &data)) {
		SetupDiDestroyDeviceInfoList(info);
		return INVALID_HANDLE_VALUE;
	}

	DWORD size = 0;

	SetupDiGetDeviceInterfaceDetailW(info, &data, nullptr, 0, &size, nullptr);
	std::vector<std::uint8_t> buf(size);
	auto *detail = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W *>(buf.data());

	detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
	SetupDiGetDeviceInterfaceDetailW(info, &data, detail, size, nullptr, nullptr);
	SetupDiDestroyDeviceInfoList(info);

	HANDLE h = CreateFileW(detail->DevicePath, GENERIC_WRITE | GENERIC_READ, FILE_SHARE_WRITE | FILE_SHARE_READ,
		nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, nullptr);

	if (h == INVALID_HANDLE_VALUE)
		return h;

	if (!WinUsb_Initialize(h, &winusb)) {
		CloseHandle(h);
		return INVALID_HANDLE_VALUE;
	}

	return h;
}

int main(int argc, char **argv)
{
	if (argc < 3) {
		fprintf(stderr, "usage: zd1_probe <device-interface-guid> info|start\n");
		return 2;
	}

	GUID guid;

	if (!ParseGuid(argv[1], guid)) {
		fprintf(stderr, "invalid guid\n");
		return 2;
	}

	WINUSB_INTERFACE_HANDLE winusb = nullptr;
	HANDLE h = OpenDevice(guid, winusb);

	if (h == INVALID_HANDLE_VALUE) {
		fprintf(stderr, "cannot open device (error %lu)\n", GetLastError());
		return 1;
	}

	USB_DEVICE_DESCRIPTOR desc = {};
	ULONG got = 0;

	WinUsb_GetDescriptor(winusb, USB_DEVICE_DESCRIPTOR_TYPE, 0, 0, reinterpret_cast<PUCHAR>(&desc), sizeof(desc), &got);
	Log("VID:PID %04X:%04X\n", desc.idVendor, desc.idProduct);

	px4::Mb86h47Bridge br(winusb, Log);
	std::string cmd = argv[2];
	int ret = 0;

	if (cmd == "info") {
		std::uint16_t st = 0, dir = 0, val = 0;

		ret = br.Read16(0x82008, st);
		Log("0xF02008 (state) = 0x%04X ret=%d\n", st, ret);
		if (!ret) {
			ret = br.GpioRead(dir, val);
			Log("GPIO dir=0x%04X val=0x%04X ret=%d\n", dir, val, ret);
		}
	} else if (cmd == "start") {
		ret = br.Start();
		Log("Start: %d\n", ret);
		if (!ret) {
			std::uint32_t live = 0;

			br.Read32(0x1700000 + 0xA3F7C, live);
			Log("core1 live counter = %u\n", live);
			ret = br.ResetDemodulator();
			Log("ResetDemodulator: %d\n", ret);
		}
		if (!ret) {
			std::uint8_t reg[2] = { 0x03, 0x80 }, v[2] = {};	/* DiB8000 レジスタ 896 (ベンダー ID) */

			for (int i = 0; i < 2; i++)
				ret = br.I2cTransfer(0x12, reg, 2, v, 2);
			Log("DiB8000 reg896 = 0x%02X%02X (expect 0x01B3) ret=%d\n", v[0], v[1], ret);
			reg[0] = 0x03; reg[1] = 0x81;
			ret = br.I2cTransfer(0x12, reg, 2, v, 2);
			Log("DiB8000 reg897 = 0x%02X%02X (expect 0x8000..0x8002) ret=%d\n", v[0], v[1], ret);
		}
	} else if (cmd == "card") {
		ret = br.Start();
		Log("Start: %d\n", ret);
		bool present = false;

		if (!ret) ret = br.CardDetect(present);
		Log("card present: %d (ret %d)\n", present, ret);
		for (std::uint32_t a : { 0xFEA000u, 0xFEA004u, 0xFEA014u, 0xFEA018u, 0xFEA01Cu, 0xFEA020u, 0xFEA024u }) {
			std::uint32_t v = 0;

			br.Read32(a, v);
			Log("  uart[0x%06X] = 0x%08X\n", a, v);
		}
		/* レジスタの幅の確認: 書いて読み戻す */
		br.Write32(0xFEA024, 0x12345678);
		std::uint32_t v = 0;
		br.Read32(0xFEA024, v);
		Log("  BWT after Write32(0x12345678) = 0x%08X\n", v);
		br.Write32(0xFEA020, 0x12345678);
		br.Read32(0xFEA020, v);
		Log("  CWT after Write32(0x12345678) = 0x%08X\n", v);
		if (!ret && present) {
			ret = br.CardReset();
			Log("CardReset: %d\n", ret);
			for (int i = 0; i < 20 && !ret; i++) {
				Sleep(50);
				bool ready = false;

				br.CardDataReady(ready);
				if (ready && i > 2) {
					std::uint8_t atr[64];
					std::uint8_t n = sizeof(atr);

					br.CardRead(atr, n);
					Log("ATR (%u bytes):", n);
					for (int k = 0; k < n; k++)
						Log(" %02X", atr[k]);
					Log("\n");
					break;
				}
			}
		}
		br.CardPowerDown();
	} else if (cmd == "tune") {
		int ch = (argc > 3) ? atoi(argv[3]) : 27;
		int secs = (argc > 4) ? atoi(argv[4]) : 5;
		const char *outpath = (argc > 5) ? argv[5] : nullptr;
		std::uint32_t hz = 473142857u + 6000000u * (ch - 13);

		hz = hz / 1000 * 1000;
		ret = br.Start();
		Log("Start: %d\n", ret);
		if (!ret) ret = br.ResetDemodulator();
		Log("ResetDemodulator: %d\n", ret);

		BridgeBus bus(br);
		px4::Dib8096 dib(bus, Log, nullptr);

		if (!ret) {
			ULONGLONG t0 = GetTickCount64();

			ret = dib.Initialize();
			Log("Dib8096::Initialize: %d (rev 0x%04X tuner 0x%04X) %llums %u i2c\n", ret, dib.revision(), dib.tuner_id(), GetTickCount64() - t0, bus.count());
		}
		if (!ret) {
			dib.SetGpio(1, true);
			ULONGLONG t0 = GetTickCount64();
			unsigned int c0 = bus.count();

			ret = dib.Tune(hz);
			Log("Tune(%u Hz, ch%d): %d  %llums %u i2c\n", hz, ch, ret, GetTickCount64() - t0, bus.count() - c0);
		}
		px4::Dib8096::Status st = {};
		if (!ret) {
			ret = dib.GetStatus(st);
			Log("status flags=0x%02X strength=%d mdBm cnr=%d mdB valid=%d ret=%d\n", st.flags, st.strength_mdbm, st.cnr_mdb, st.cnr_valid, ret);
		}
		if (!ret && (st.flags & px4::Dib8096::STATUS_LOCK)) {
			ret = br.StreamPrepare();
			Log("StreamPrepare: %d\n", ret);
			if (!ret) {
				px4::Zd1TsFramer framer;
				std::vector<std::uint8_t> ts;
				FILE *fp = outpath ? fopen(outpath, "wb") : nullptr;
				std::uint64_t total = 0, pid0 = 0, tei = 0;
				std::uint64_t raw = 0;

				px4::Mb86h47Bridge::StreamConfig sc;

				ret = br.StreamStart([&](const std::uint8_t *buf, std::size_t len) {
					raw += len;
					ts.clear();
					framer.Feed(buf, len, ts);
					for (std::size_t i = 0; i + 188 <= ts.size(); i += 188) {
						total++;
						if (((ts[i + 1] & 0x1F) << 8 | ts[i + 2]) == 0)
							pid0++;
						if (ts[i + 1] & 0x80)
							tei++;
					}
					if (fp && !ts.empty())
						fwrite(ts.data(), 1, ts.size(), fp);
					return 0;
				}, sc);
				Log("StreamStart: %d\n", ret);
				for (int i = 0; i < secs && !ret; i++) {
					Sleep(1000);
					std::uint32_t units = 0;

					br.ReadCore0Counter(units);
					Log("  t=%ds raw=%llu bytes ts=%llu pkts pid0=%llu tei=%llu syncloss=%llu core0units=%u\n", i + 1, raw, total, pid0, tei, framer.sync_losses(), units);
				}
				br.StreamStop();
				if (fp) fclose(fp);
			}
			dib.SetOutputMode(px4::Dib8096::OutputMode::HIGH_Z);
		}
		dib.SetGpio(1, false);
		dib.Sleep();
	} else if (cmd == "tunebench") {
		/* tunebench <ch,ch,...> <繰り返し>: 選局だけを繰り返し、待ちと I2C の時間の内訳を出す */
		std::vector<int> chs;
		std::string list = (argc > 3) ? argv[3] : "27";
		int repeat = (argc > 4) ? atoi(argv[4]) : 3;

		for (size_t i = 0; i < list.size();) {
			chs.push_back(atoi(list.c_str() + i));
			size_t j = list.find(',', i);

			i = (j == std::string::npos) ? list.size() : j + 1;
		}

		ret = br.Start();
		if (!ret) ret = br.ResetDemodulator();

		struct Prof { std::vector<unsigned> hist; unsigned sleeps = 0; double sleep_req = 0, sleep_real = 0, i2c = 0; unsigned i2c_n = 0; } prof;
		LARGE_INTEGER f;

		QueryPerformanceFrequency(&f);
		auto now_ms = [&]() { LARGE_INTEGER t; QueryPerformanceCounter(&t); return 1000.0 * t.QuadPart / f.QuadPart; };

		class ProfBus final : public px4::Dib8096::Bus {
		public:
			ProfBus(px4::Mb86h47Bridge &b, Prof &p, std::function<double()> n) : b_(b), p_(p), n_(n) {}
			int Transfer(std::uint8_t a, const std::uint8_t *w, std::size_t wl, std::uint8_t *r, std::size_t rl) override
			{
				double t = n_();
				int ret = b_.I2cAccess(a, w, wl, r, rl);

				p_.i2c += n_() - t;
				p_.i2c_n++;
				return ret;
			}
			int Flush() override { double t = n_(); int ret = b_.I2cFlush(); p_.i2c += n_() - t; return ret; }
		private:
			px4::Mb86h47Bridge &b_;
			Prof &p_;
			std::function<double()> n_;
		} pbus(br, prof, now_ms);

		px4::Dib8096 dib(pbus, nullptr, [&](unsigned int ms) {
			pbus.Flush();
			double t = now_ms();

			px4::Dib8096::PreciseSleepMs(ms);
			prof.sleeps++;
			prof.hist.push_back(ms);
			prof.sleep_req += ms;
			prof.sleep_real += now_ms() - t;
		});

		if (!ret) ret = dib.Initialize();
		for (int r = 0; r < repeat && !ret; r++) {
			for (int ch : chs) {
				prof = Prof();
				double t0 = now_ms();

				ret = dib.Tune((473142857u + 6000000u * (ch - 13)) / 1000 * 1000);
				double total = now_ms() - t0;
				px4::Dib8096::Status st = {};

				dib.GetStatus(st);
				if (getenv("ZD1_HIST")) {
					std::string h;

					for (unsigned v : prof.hist)
						h += std::to_string(v) + " ";
					Log("  sleeps: %s\n", h.c_str());
				}
				Log("ch%d: ret=%d total=%.0fms sleeps=%u (req %.0f real %.0f) i2c=%u (%.0fms) flags=0x%02X cnr=%d\n",
					ch, ret, total, prof.sleeps, prof.sleep_req, prof.sleep_real, prof.i2c_n, prof.i2c, st.flags, st.cnr_mdb);
			}
		}
		dib.Sleep();
	} else if (cmd == "bench") {
		ret = br.Start();
		if (!ret) ret = br.ResetDemodulator();
		const int n = 200;
		std::uint8_t reg[2] = { 0x03, 0x80 }, v[2];
		LARGE_INTEGER f, a, b;

		QueryPerformanceFrequency(&f);
		QueryPerformanceCounter(&a);
		for (int i = 0; i < n && !ret; i++)
			ret = br.I2cTransfer(0x12, reg, 2, v, 2);
		QueryPerformanceCounter(&b);
		Log("i2c read x%d: %.3f ms each\n", n, 1000.0 * (b.QuadPart - a.QuadPart) / f.QuadPart / n);
		std::uint32_t x;
		QueryPerformanceCounter(&a);
		for (int i = 0; i < n && !ret; i++)
			ret = br.Read32(0x82008, x);
		QueryPerformanceCounter(&b);
		Log("mem read32 x%d: %.3f ms each\n", n, 1000.0 * (b.QuadPart - a.QuadPart) / f.QuadPart / n);
	}

	WinUsb_Free(winusb);
	CloseHandle(h);
	return ret ? 1 : 0;
}
