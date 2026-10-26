// bondriver_test.cpp
//
// BonDriver_PX4 (BonDriver_CECHZD1J) を LoadLibrary し、DriverHost_PX4 経由で選局と TS 受信を行う動作確認用クライアント。
//   bondriver_test <BonDriver の DLL> <チャンネル番号 (ChSet の通し番号) をカンマ区切りで> [各チャンネルの秒数] [出力.ts]

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <vector>

#include <windows.h>

#include "IBonDriver2.h"

int wmain(int argc, wchar_t **argv)
{
	if (argc < 3) {
		fwprintf(stderr, L"usage: bondriver_test <dll> <channel> [seconds] [out.ts]\n");
		return 2;
	}

	HMODULE mod = LoadLibraryW(argv[1]);

	if (!mod) {
		fwprintf(stderr, L"LoadLibrary failed (%lu)\n", GetLastError());
		return 1;
	}

	using CreateFn = IBonDriver *(*)();
	auto create = reinterpret_cast<CreateFn>(GetProcAddress(mod, "CreateBonDriver"));

	if (!create) {
		fwprintf(stderr, L"CreateBonDriver not found\n");
		return 1;
	}

	IBonDriver2 *drv = dynamic_cast<IBonDriver2 *>(create());
	if (!drv) {
		drv = reinterpret_cast<IBonDriver2 *>(create());
	}

	std::vector<DWORD> chs;
	for (wchar_t *ctx = nullptr, *tok = wcstok_s(argv[2], L",", &ctx); tok; tok = wcstok_s(nullptr, L",", &ctx))
		chs.push_back(static_cast<DWORD>(_wtoi(tok)));
	const int secs = (argc > 3) ? _wtoi(argv[3]) : 10;
	FILE *fp = nullptr;

	if (argc > 4)
		_wfopen_s(&fp, argv[4], L"wb");

	DWORD t0 = GetTickCount();

	if (!drv->OpenTuner()) {
		fwprintf(stderr, L"OpenTuner failed\n");
		return 1;
	}
	wprintf(L"OpenTuner ok (%lu ms) tuner name=%s\n", GetTickCount() - t0, drv->GetTunerName());

	for (DWORD i = 0; drv->EnumTuningSpace(i); i++) {
		wprintf(L" space %lu: %s\n", i, drv->EnumTuningSpace(i));
		for (DWORD j = 0; j < 3 && drv->EnumChannelName(i, j); j++)
			wprintf(L"   ch %lu: %s\n", j, drv->EnumChannelName(i, j));
	}

	for (DWORD ch : chs) {
		t0 = GetTickCount();
		BOOL ok = drv->SetChannel(0, ch);

		wprintf(L"SetChannel(0, %lu) = %d (%lu ms) level=%.2f\n", ch, ok, GetTickCount() - t0, drv->GetSignalLevel());

		DWORD begin = GetTickCount();
		std::uint64_t total = 0, bad = 0, pid0 = 0, cerr = 0;
		int cc[8192];
		for (int &c : cc)
			c = -1;
		DWORD last = begin;
		DWORD first_ts = 0;

		while (ok && GetTickCount() - begin < static_cast<DWORD>(secs) * 1000) {
			BYTE *p = nullptr;
			DWORD size = 0, remain = 0;

			if (drv->GetTsStream(&p, &size, &remain) && p && size) {
				if (!first_ts)
					first_ts = GetTickCount() - begin;
				for (DWORD i = 0; i + 188 <= size; i += 188) {
					total++;
					if (p[i] != 0x47)
						bad++;
					const unsigned pid = ((p[i + 1] & 0x1F) << 8) | p[i + 2];

					if (pid == 0)
						pid0++;
					if (pid != 0x1FFF && (p[i + 3] & 0x10)) {
						const int c = p[i + 3] & 0x0F;

						if (cc[pid] >= 0 && ((cc[pid] + 1) & 15) != c)
							cerr++;
						cc[pid] = c;
					}
				}
				if (fp)
					fwrite(p, 1, size, fp);
			} else {
				Sleep(5);
			}
		}

		wprintf(L"  ch %lu: first TS after %lu ms, pkts=%llu bad=%llu pid0=%llu continuity-errors=%llu level=%.2f\n",
			ch, first_ts, total, bad, pid0, cerr, drv->GetSignalLevel());
	}

	if (fp)
		fclose(fp);
	drv->CloseTuner();
	wprintf(L"done\n");
	drv->Release();
	return 0;
}
