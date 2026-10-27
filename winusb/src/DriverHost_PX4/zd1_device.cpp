// zd1_device.cpp

#include "zd1_device.hpp"

#include <cstdarg>
#include <cstdio>
#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "type.hpp"
#include "command.hpp"
#include "misc_win.h"

namespace px4 {

namespace {

void BridgeLog(const char *fmt, ...)
{
	char buf[512];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	msg_printf("[zd1] %s", buf);
}

/* 前面 LED は DiB8000 の GPIO 1。High で赤、Low で緑 */
constexpr unsigned int kLedPin = 1;

} // namespace

Zd1Device::Zd1Device(const std::wstring &path, const px4::DeviceDefinition &device_def, std::uintptr_t index, px4::ReceiverManager &receiver_manager)
	: DeviceBase(path, device_def, index, receiver_manager),
	config_(),
	lock_(),
	available_(true),
	init_(false),
	capture_(false),
	card_open_(false),
	prepare_active_(false),
	prepare_error_(0)
{
	if (usb_dev_.descriptor.idVendor != 0x054c || usb_dev_.descriptor.idProduct != 0x04b2)
		throw DeviceError("px4::Zd1Device::Zd1Device: unsupported device. (unknown vendor id or product id)");

	LoadConfig();
}

Zd1Device::~Zd1Device()
{
	Term();
}

void Zd1Device::LoadConfig()
{
	auto &configs = device_def_.configs;

	if (configs.Exists(L"UrbBytes"))
		config_.usb.urb_bytes = px4::util::wtoui(configs.Get(L"UrbBytes"));

	if (configs.Exists(L"MaxUrbs"))
		config_.usb.urb_num = px4::util::wtoui(configs.Get(L"MaxUrbs"));

	if (configs.Exists(L"ReceiverMaxPackets"))
		config_.device.receiver_max_packets = px4::util::wtoui(configs.Get(L"ReceiverMaxPackets"));

	/* USB の読み出し長は 1536 バイト (8 ユニット) の倍数、最大 64KiB */
	config_.usb.urb_bytes = (config_.usb.urb_bytes / 1536) * 1536;
	if (config_.usb.urb_bytes < 1536)
		config_.usb.urb_bytes = 1536;
	if (config_.usb.urb_bytes > 64 * 1024)
		config_.usb.urb_bytes = 64 * 1024 / 1536 * 1536;
	if (!config_.usb.urb_num)
		config_.usb.urb_num = 1;

	dev_dbg(&dev_, "px4::Zd1Device::LoadConfig: urb_bytes: %u\n", config_.usb.urb_bytes);
	dev_dbg(&dev_, "px4::Zd1Device::LoadConfig: urb_num: %u\n", config_.usb.urb_num);
	dev_dbg(&dev_, "px4::Zd1Device::LoadConfig: receiver_max_packets: %u\n", config_.device.receiver_max_packets);
}

int Zd1Device::BridgeBus::Transfer(std::uint8_t addr8, const std::uint8_t *w, std::size_t wlen, std::uint8_t *r, std::size_t rlen)
{
	/* 切断後の古い要求から USB 制御転送を発行しない */
	if (!parent_.available_)
		return -ENODEV;

	return parent_.bridge_->I2cAccess(addr8, w, wlen, r, rlen);
}

int Zd1Device::BridgeBus::Flush()
{
	if (!parent_.available_)
		return -ENODEV;

	return parent_.bridge_->I2cFlush();
}

int Zd1Device::Init()
{
	dev_dbg(&dev_, "px4::Zd1Device::Init: init_: %s\n", (init_) ? "true" : "false");

	std::lock_guard<std::recursive_mutex> lock(lock_);

	if (init_)
		return -EALREADY;

	int ret;

	bridge_.reset(new px4::Mb86h47Bridge(usb_dev_.winusb, BridgeLog));

	ret = bridge_->Start();
	if (ret) {
		dev_err(&dev_, "px4::Zd1Device::Init: bridge Start() failed. (ret: %d)\n", ret);
		goto fail;
	}

	ret = bridge_->ResetDemodulator();
	if (ret) {
		dev_err(&dev_, "px4::Zd1Device::Init: ResetDemodulator() failed. (ret: %d)\n", ret);
		goto fail;
	}

	bus_.reset(new BridgeBus(*this));
	dib_.reset(new px4::Dib8096(*bus_, BridgeLog, [this](unsigned int ms) { WaitDuringTune(ms); }));

	ret = dib_->Initialize();
	if (ret) {
		dev_err(&dev_, "px4::Zd1Device::Init: Dib8096::Initialize() failed. (ret: %d)\n", ret);
		goto fail;
	}

	/* 受信機を開くまでは省電力にしておく。次の Tune で自動的に復帰する */
	ret = dib_->Sleep();
	if (ret)
		goto fail;

	receiver_.reset(new Zd1Receiver(*this));
	stream_buf_ = receiver_->GetStreamBuffer();
	init_ = true;

	for (auto it = device_def_.receivers.cbegin(); it != device_def_.receivers.cend(); ++it) {
		px4::command::ReceiverInfo ri;

		wcscpy_s(ri.device_name, device_def_.name.c_str());
		ri.device_guid = device_def_.guid;
		wcscpy_s(ri.receiver_name, it->name.c_str());
		ri.receiver_guid = it->guid;
		ri.systems = it->systems;
		ri.index = it->index;
		ri.data_id = 0;

		receiver_manager_.Register(ri, receiver_.get());
	}

	return 0;

fail:
	dib_.reset();
	bus_.reset();
	bridge_.reset();
	return ret;
}

void Zd1Device::Term()
{
	dev_dbg(&dev_, "px4::Zd1Device::Term: init_: %s\n", (init_) ? "true" : "false");

	std::unique_lock<std::recursive_mutex> lock(lock_);

	if (!init_)
		return;

	receiver_manager_.Unregister(receiver_.get());

	lock.unlock();

	/* 開いている受信機が閉じるまで待つ */
	stream_buf_ = nullptr;
	receiver_.reset();

	lock.lock();

	StopCapture();
	if (available_) {
		dib_->SetGpio(kLedPin, false);
		dib_->Sleep();
	}
	bridge_->Stop();

	dib_.reset();
	bus_.reset();
	bridge_.reset();

	init_ = false;
}

void Zd1Device::SetAvailability(bool available)
{
	std::lock_guard<std::recursive_mutex> lock(lock_);

	/* 切断後の古いカードセッションから USB 制御転送を発行しない */
	if (available_ && !available)
		card_open_ = false;
	available_ = available;
}

int Zd1Device::OpenCard()
{
	std::lock_guard<std::recursive_mutex> lock(lock_);

	if (!available_ || !init_)
		return -ENODEV;
	if (card_open_)
		return -EBUSY;

	/* カードの電源は ResetCard() で入れるため、ここでは使用中の印を付けるだけ */
	card_open_ = true;
	return 0;
}

void Zd1Device::CloseCard()
{
	std::lock_guard<std::recursive_mutex> lock(lock_);

	if (!card_open_)
		return;

	card_open_ = false;
	if (available_ && init_)
		bridge_->CardPowerDown();
}

int Zd1Device::DetectCard(bool &detected)
{
	std::lock_guard<std::recursive_mutex> lock(lock_);

	return (available_ && init_) ? bridge_->CardDetect(detected) : -ENODEV;
}

int Zd1Device::ResetCard()
{
	std::lock_guard<std::recursive_mutex> lock(lock_);

	return (card_open_ && available_) ? bridge_->CardReset() : -ENODEV;
}

int Zd1Device::SetCardBaudrate(std::uint32_t baudrate)
{
	std::lock_guard<std::recursive_mutex> lock(lock_);

	return (card_open_ && available_) ? bridge_->CardSetBaudrate(baudrate) : -ENODEV;
}

int Zd1Device::IsCardDataReady(bool &ready)
{
	std::lock_guard<std::recursive_mutex> lock(lock_);

	return (card_open_ && available_) ? bridge_->CardDataReady(ready) : -ENODEV;
}

int Zd1Device::ReadCardData(std::uint8_t *buf, std::uint8_t &len)
{
	std::lock_guard<std::recursive_mutex> lock(lock_);

	return (card_open_ && available_) ? bridge_->CardRead(buf, len) : -ENODEV;
}

int Zd1Device::WriteCardData(const std::uint8_t *buf, std::uint8_t len)
{
	std::lock_guard<std::recursive_mutex> lock(lock_);

	return (card_open_ && available_) ? bridge_->CardWrite(buf, len) : -ENODEV;
}

ReceiverBase* Zd1Device::GetReceiver(int id) const
{
	if (id != 0)
		throw std::out_of_range("receiver id out of range");

	return receiver_.get();
}

namespace {

/*
 * ロックしたチャンネルの伝送パラメーターは放送局ごとに固定のため、同じ DriverHost_PX4 の中の受信機で共有する。
 * 2 台目以降の初回の選局も、自動探索を省略できる。保存はしないため、DriverHost_PX4 の再起動後は探索から始まる。
 */
std::mutex g_locks_mutex;
std::map<std::uint32_t, px4::Dib8096::LockRecord> g_locks;

} // namespace

void Zd1Device::LoadLocks()
{
	std::vector<px4::Dib8096::LockRecord> records;

	{
		std::lock_guard<std::mutex> lock(g_locks_mutex);

		for (const auto &kv : g_locks)
			records.push_back(kv.second);
	}

	dib_->ImportLocks(records);
}

void Zd1Device::SaveLocks()
{
	std::lock_guard<std::recursive_mutex> lock(lock_);

	if (!dib_ || !dib_->locks_dirty())
		return;

	{
		std::lock_guard<std::mutex> locks_lock(g_locks_mutex);

		for (const auto &r : dib_->ExportLocks())
			g_locks[r.frequency] = r;
	}

	dib_->clear_locks_dirty();
}

int Zd1Device::StartCapture()
{
	std::lock_guard<std::recursive_mutex> lock(lock_);

	if (capture_)
		return 0;
	if (!available_)
		return -ENODEV;

	/* 設定は入力を止めたまま済ませ、読み出しを発行してから入力を許可する */
	int ret = FinishStreamPrepare();
	if (ret) {
		dev_err(&dev_, "px4::Zd1Device::StartCapture: StreamPrepare() failed. (ret: %d)\n", ret);
		return ret;
	}

	framer_.Reset();

	px4::Mb86h47Bridge::StreamConfig sc;

	sc.urb_bytes = config_.usb.urb_bytes;
	sc.urb_num = config_.usb.urb_num;

	ret = bridge_->StreamStart([this](const std::uint8_t *buf, std::size_t len) { return StreamHandler(buf, len); }, sc);
	if (ret) {
		dev_err(&dev_, "px4::Zd1Device::StartCapture: StreamStart() failed. (ret: %d)\n", ret);
		return ret;
	}

	capture_ = true;
	return 0;
}

void Zd1Device::BeginStreamPrepare()
{
	std::lock_guard<std::recursive_mutex> lock(lock_);

	if (capture_)
		return;

	bridge_->StreamPrepareBegin();
	prepare_active_ = true;
	prepare_error_ = 0;
}

/*
 * 選局の待ち時間に、ストリーム設定の続きを 1 ステップずつ進める (spec 04 章 4.4)。
 * 設定は TS エンジンの入力とリング書き込みを止めたままで、選局中の復調器の出力は取り込まれない。
 * 別のスレッドではなく待ちを呼んだスレッドで行うのは、制御転送を I2C のバルク転送と同時に走らせると、
 * 実機で I2C の読み出しが失敗する (150 回中 2 回) ことがあったため。
 */
void Zd1Device::WaitDuringTune(unsigned int ms)
{
	const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);

	while (prepare_active_ && !prepare_error_ && bridge_->StreamPrepareRemaining() &&
		end - std::chrono::steady_clock::now() >= std::chrono::milliseconds(5))
		prepare_error_ = bridge_->StreamPrepareStep();

	const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(end - std::chrono::steady_clock::now());

	if (left.count() > 0)
		px4::Dib8096::PreciseSleepMs(static_cast<unsigned int>(left.count()));
}

int Zd1Device::FinishStreamPrepare()
{
	std::lock_guard<std::recursive_mutex> lock(lock_);

	if (!prepare_active_)
		bridge_->StreamPrepareBegin();
	prepare_active_ = false;

	if (prepare_error_)
		return prepare_error_;

	int ret = bridge_->StreamPrepareEngine();

	if (ret)
		return ret;

	return bridge_->StreamPrepareUsb();
}

void Zd1Device::StopCapture()
{
	std::lock_guard<std::recursive_mutex> lock(lock_);

	/* 途中までの設定は捨てる。受信を始めるときに、最初からやり直す */
	prepare_active_ = false;

	if (!capture_)
		return;

	bridge_->StreamStop();
	capture_ = false;
}

int Zd1Device::StreamHandler(const std::uint8_t *buf, std::size_t len)
{
	auto stream_buf = stream_buf_;

	if (!stream_buf)
		return 0;

	ts_buf_.clear();
	framer_.Feed(buf, len, ts_buf_);

	if (!ts_buf_.empty()) {
		std::size_t size = ts_buf_.size();

		stream_buf->Write(ts_buf_.data(), size);
		stream_buf->NotifyWrite();
	}

	return 0;
}

Zd1Device::Zd1Receiver::Zd1Receiver(Zd1Device &parent)
	: ReceiverBase(0),
	parent_(parent),
	open_(false),
	streaming_(false)
{
}

Zd1Device::Zd1Receiver::~Zd1Receiver()
{
	std::unique_lock<std::mutex> lock(lock_);

	close_cond_.wait(lock, [this] { return !open_; });
}

int Zd1Device::Zd1Receiver::Open()
{
	std::unique_lock<std::mutex> lock(lock_);
	std::lock_guard<std::recursive_mutex> dev_lock(parent_.lock_);

	if (open_)
		return -EALREADY;
	if (!parent_.available_)
		return -ENODEV;

	int ret = parent_.dib_->SetGpio(kLedPin, true);
	if (ret) {
		dev_err(&parent_.dev_, "px4::Zd1Device::Zd1Receiver::Open: SetGpio() failed. (ret: %d)\n", ret);
		return ret;
	}

	open_ = true;
	return 0;
}

void Zd1Device::Zd1Receiver::Close()
{
	if (!open_)
		return;

	SetCapture(false);

	std::unique_lock<std::mutex> lock(lock_);
	std::lock_guard<std::recursive_mutex> dev_lock(parent_.lock_);

	if (parent_.available_) {
		parent_.dib_->SetGpio(kLedPin, false);
		parent_.dib_->Sleep();
	}

	open_ = false;

	lock.unlock();
	close_cond_.notify_all();
}

int Zd1Device::Zd1Receiver::SetFrequency()
{
	if (!open_)
		return -EINVAL;

	if (params_.system != px4::SystemType::ISDB_T)
		return -EINVAL;

	dev_dbg(&parent_.dev_, "px4::Zd1Device::Zd1Receiver::SetFrequency: freq: %u\n", params_.freq);

	std::lock_guard<std::mutex> lock(lock_);
	std::lock_guard<std::recursive_mutex> dev_lock(parent_.lock_);

	if (!parent_.available_)
		return -ENODEV;

	/* 選局中は TS 出力が止まっている必要がある。受信中なら先に TS の読み出しを止める */
	parent_.StopCapture();
	parent_.BeginStreamPrepare();
	parent_.LoadLocks();

	int ret = parent_.dib_->Tune(params_.freq * 1000);
	if (ret)
		dev_err(&parent_.dev_, "px4::Zd1Device::Zd1Receiver::SetFrequency: Dib8096::Tune() failed. (ret: %d)\n", ret);

	parent_.SaveLocks();

	return ret;
}

int Zd1Device::Zd1Receiver::CheckLock(bool &locked)
{
	if (!open_)
		return -EINVAL;

	int ret;
	unsigned int flags = 0;

	{
		std::lock_guard<std::mutex> lock(lock_);
		std::lock_guard<std::recursive_mutex> dev_lock(parent_.lock_);

		if (!parent_.available_)
			return -ENODEV;

		ret = parent_.dib_->GetLockFlags(flags);
	}

	locked = !ret && (flags & px4::Dib8096::STATUS_LOCK);
	if (locked) {
		dev_dbg(&parent_.dev_, "px4::Zd1Device::Zd1Receiver::CheckLock: locked: true\n");
		ret = SetCapture(true);
	}

	return ret;
}

int Zd1Device::Zd1Receiver::SetStreamId()
{
	return -EINVAL;
}

int Zd1Device::Zd1Receiver::SetLnbVoltage(std::int32_t voltage)
{
	return 0;
}

int Zd1Device::Zd1Receiver::SetCapture(bool capture)
{
	if (!open_)
		return -EINVAL;

	dev_dbg(&parent_.dev_, "px4::Zd1Device::Zd1Receiver::SetCapture: capture: %s\n", (capture) ? "true" : "false");

	int ret = 0;
	std::lock_guard<std::mutex> lock(lock_);

	if (capture) {
		ret = parent_.StartCapture();
		if (ret)
			return ret;

		std::size_t size = 188 * parent_.config_.device.receiver_max_packets;

		if (!streaming_) {
			stream_buf_->Alloc(size);
			stream_buf_->SetThresholdSize(size / 10);
			stream_buf_->Start();
			streaming_ = true;
		} else {
			stream_buf_->Purge();
		}
	} else {
		parent_.StopCapture();

		if (streaming_) {
			stream_buf_->Stop();
			streaming_ = false;
		}

		/* 次の選局までは TS 出力を止めておく */
		std::lock_guard<std::recursive_mutex> dev_lock(parent_.lock_);

		if (parent_.available_)
			parent_.dib_->SetOutputMode(px4::Dib8096::OutputMode::HIGH_Z);
	}

	return ret;
}

int Zd1Device::Zd1Receiver::ReadStat(px4::command::StatType type, std::int32_t &value)
{
	if (!open_)
		return -EINVAL;

	int ret = 0;
	std::lock_guard<std::mutex> lock(lock_);
	std::lock_guard<std::recursive_mutex> dev_lock(parent_.lock_);

	value = 0;

	if (!parent_.available_)
		return -ENODEV;

	switch (type) {
	case px4::command::StatType::SIGNAL_STRENGTH:
		// not implemented
		ret = -ENOSYS;
		break;

	case px4::command::StatType::CNR:
	{
		px4::Dib8096::Status st;

		ret = parent_.dib_->GetStatus(st);
		if (!ret && st.cnr_valid)
			value = st.cnr_mdb;

		break;
	}

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

} // namespace px4
