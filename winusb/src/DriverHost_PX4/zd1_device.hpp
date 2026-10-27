// zd1_device.hpp
//
// Sony CECH-ZD1 J (USB 054c:04b2)。ブリッジ MB86H47A と DiB8096 (DiB8000 + DiB0090) による ISDB-T 1 チューナー。
// ファームウェア・ファイルは不要で、起動のたびに内蔵 CPU へ小さなプログラムを書き込む (mb86h47_bridge)。

#pragma once

#include <cstdint>
#include <memory>
#include <atomic>
#include <string>
#include <mutex>
#include <condition_variable>
#include <vector>

#include "device_definition_set.hpp"
#include "device_base.hpp"
#include "receiver_base.hpp"
#include "receiver_manager.hpp"

#include "mb86h47_bridge.hpp"
#include "dib8096.hpp"
#include "zd1_ts_framer.hpp"

namespace px4 {

struct Zd1DeviceConfig final {
	Zd1DeviceConfig()
		: usb{ 64 * 1024, 16 },
		device{ 2048 }
	{}
	struct {
		unsigned int urb_bytes;
		unsigned int urb_num;
	} usb;
	struct {
		unsigned int receiver_max_packets;
	} device;
};

class Zd1Device final : public px4::DeviceBase {
public:
	Zd1Device() = delete;
	Zd1Device(const std::wstring &path, const px4::DeviceDefinition &device_def, std::uintptr_t index, px4::ReceiverManager &receiver_manager);
	~Zd1Device();

	// cannot copy
	Zd1Device(const Zd1Device &) = delete;
	Zd1Device& operator=(const Zd1Device &) = delete;

	// cannot move
	Zd1Device(Zd1Device &&) = delete;
	Zd1Device& operator=(Zd1Device &&) = delete;

	int Init() override;
	void Term() override;
	void SetAvailability(bool available) override;
	px4::ReceiverBase * GetReceiver(int id) const override;
	bool HasCardReader() const noexcept override { return true; }
	int OpenCard() override;
	void CloseCard() override;
	int DetectCard(bool &detected) override;
	int ResetCard() override;
	int SetCardBaudrate(std::uint32_t baudrate) override;
	int IsCardDataReady(bool &ready) override;
	int ReadCardData(std::uint8_t *buf, std::uint8_t &len) override;
	int WriteCardData(const std::uint8_t *buf, std::uint8_t len) override;

private:
	/* DiB8096 が使う I2C。ブリッジの mailbox 方式の転送へ橋渡しする */
	class BridgeBus final : public px4::Dib8096::Bus {
	public:
		explicit BridgeBus(Zd1Device &parent) : parent_(parent) {}
		int Transfer(std::uint8_t addr8, const std::uint8_t *w, std::size_t wlen, std::uint8_t *r, std::size_t rlen) override;
		int Flush() override;

	private:
		Zd1Device &parent_;
	};

	class Zd1Receiver final : public px4::ReceiverBase {
	public:
		Zd1Receiver() = delete;
		explicit Zd1Receiver(Zd1Device &parent);
		~Zd1Receiver();

		// cannot copy
		Zd1Receiver(const Zd1Receiver &) = delete;
		Zd1Receiver& operator=(const Zd1Receiver &) = delete;

		// cannot move
		Zd1Receiver(Zd1Receiver &&) = delete;
		Zd1Receiver& operator=(Zd1Receiver &&) = delete;

		int Open() override;
		void Close() override;
		int CheckLock(bool &locked) override;
		int SetLnbVoltage(std::int32_t voltage) override;
		int SetCapture(bool capture) override;
		int ReadStat(px4::command::StatType type, std::int32_t &value) override;

	protected:
		int SetFrequency() override;
		int SetStreamId() override;

	private:
		Zd1Device &parent_;

		std::mutex lock_;
		std::atomic_bool open_;
		std::condition_variable close_cond_;
		std::atomic_bool streaming_;
	};

	void LoadConfig();

	int StartCapture();
	void StopCapture();
	/* ストリーム設定 (TS エンジン、core 0、USB の DMA) を、選局の待ち時間に並行して済ませる */
	void BeginStreamPrepare();
	void WaitDuringTune(unsigned int ms);
	int FinishStreamPrepare();
	void LoadLocks();
	void SaveLocks();
	int StreamHandler(const std::uint8_t *buf, std::size_t len);

	Zd1DeviceConfig config_;
	std::recursive_mutex lock_;
	std::atomic_bool available_;
	std::atomic_bool init_;
	bool capture_;
	bool card_open_;
	bool prepare_active_;
	int prepare_error_;
	std::unique_ptr<Zd1Receiver> receiver_;
	std::unique_ptr<px4::Mb86h47Bridge> bridge_;
	std::unique_ptr<BridgeBus> bus_;
	std::unique_ptr<px4::Dib8096> dib_;
	px4::Zd1TsFramer framer_;
	std::vector<std::uint8_t> ts_buf_;
	std::shared_ptr<px4::ReceiverBase::StreamBuffer> stream_buf_;
};

} // namespace px4
