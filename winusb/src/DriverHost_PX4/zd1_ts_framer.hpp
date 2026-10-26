// zd1_ts_framer.hpp
//
// CECH-ZD1 J の TS ストリームは 192 バイト単位 (4 バイトの到着タイムスタンプ + 188 バイトの TS パケット)。
// タイムスタンプを捨てて TS パケットを取り出し、単位の境界を見失ったら回復する。

#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <cstring>

#include "ts_sync.h"

namespace px4 {

class Zd1TsFramer final {
public:
	static constexpr std::size_t kUnit = 192;
	static constexpr std::size_t kHead = 4;
	static constexpr std::size_t kPacket = 188;

	void Reset() noexcept
	{
		carry_.clear();
		synced_ = false;
	}

	std::uint64_t sync_losses() const noexcept { return sync_losses_; }
	std::uint64_t packets() const noexcept { return packets_; }

	/* 読み出したバイト列を与え、TS パケット (188 バイトの連続) を out へ追記する */
	void Feed(const std::uint8_t *data, std::size_t len, std::vector<std::uint8_t> &out)
	{
		const std::uint8_t *p;
		std::size_t n;

		if (carry_.empty()) {
			p = data;
			n = len;
		} else {
			carry_.insert(carry_.end(), data, data + len);
			p = carry_.data();
			n = carry_.size();
		}

		std::size_t pos = 0;

		while (true) {
			if (!synced_) {
				bool found = false;

				/* 1 個の 0x47 はペイロードにも現れるため、192 バイト先の 0x47 でも確かめる */
				while (pos + kHead + 1 + kUnit <= n) {
					if (px4_ts_has_plain_sync(p[pos + kHead]) && px4_ts_has_plain_sync(p[pos + kHead + kUnit])) {
						found = true;
						break;
					}
					pos++;
				}

				if (!found)
					break;
				synced_ = true;
			}

			if (pos + kUnit > n)
				break;

			if (!px4_ts_has_plain_sync(p[pos + kHead])) {
				synced_ = false;
				sync_losses_++;
				pos++;
				continue;
			}

			out.insert(out.end(), p + pos + kHead, p + pos + kHead + kPacket);
			packets_++;
			pos += kUnit;
		}

		/* 端数は次の読み出しの先頭へつなぐ */
		std::vector<std::uint8_t> rest(p + pos, p + n);

		carry_.swap(rest);
	}

private:
	std::vector<std::uint8_t> carry_;
	bool synced_ = false;
	std::uint64_t sync_losses_ = 0;
	std::uint64_t packets_ = 0;
};

} // namespace px4
