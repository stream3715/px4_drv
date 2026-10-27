# Sony CECH-ZD1 J 対応 (Windows WinUSB 版)

CECH-ZD1 J (USB `054c:04b2`) を、他機種と同じ BonDriver_PX4 / DriverHost_PX4 の構成で扱うための実装です。
Linux カーネルドライバ (`driver/`) は対象外です。

この文書で「検証環境」と書く場合は、末尾の「検証記録」で定義した環境を指します。

## 構成

CECH-ZD1 J は、USB ブリッジ MB86H47A と、DiB8000 (ISDB-T 復調器) + DiB0090 (チューナー) を 1 パッケージにした DiB8096 でできています。
IT930x 系と共通する部分がないため、機種別デバイス `Zd1Device` が 3 つの層をまとめます。

| ファイル | 内容 | ライセンス |
| --- | --- | --- |
| `winusb/src/DriverHost_PX4/mb86h47_bridge.{hpp,cpp}` | USB ベンダー要求 0xBC によるチップ内メモリの読み書き、内蔵 CPU (codec core 0/1) へのプログラム投入、I2C、TS ストリーム、カードスロットの UART | GPL-2.0 |
| `winusb/src/DriverHost_PX4/dib8096.{hpp,cpp}`, `dib8096_tables.h` | DiB8000 / DiB0090 の制御。Linux の `dib8000.c` / `dib0090.c` / `dibx000_common.c` / `dib0700_devices.c` (DiBcom) の ISDB-T 13 セグメント経路の移植 | GPL-2.0-only |
| `winusb/src/DriverHost_PX4/zd1_device.{hpp,cpp}` | `DeviceBase` の実装。受信機 1 つ、カードリーダー、LED | GPL-2.0 |
| `winusb/src/DriverHost_PX4/zd1_ts_framer.hpp` | 192 バイト単位 (4 バイトの到着時刻 + 188 バイトの TS) から TS を取り出し、境界を見失ったら回復する | GPL-2.0 |
| `winusb/src/DriverHost_PX4/zd1_firmware.h` | 内蔵 CPU へ書き込むプログラム (生成物) | GPL-2.0 |
| `winusb/tools/zd1fw/` | 上記プログラムの生成 (`zd1fw_gen.py`)、FR30 アセンブラ兼エミュレーター (`fr30asm.py`)、自動試験 (`zd1fw_test.py`) | GPL-2.0 |
| `winusb/tools/zd1_probe/` | 実機の bring-up 用コマンドライン (`zd1_probe.cpp`) と、BonDriver を LoadLibrary して受信を確かめるクライアント (`bondriver_test.cpp`) | GPL-2.0 |

DiB8096 の層は Linux の GPL-2.0-only ドライバーの派生物のため、`dib8096.*` を含む成果物は GPL-2.0-only として扱います。

### ファームウェア・ファイルが不要な理由

チップは電源投入時に内蔵 ROM/SRAM のコードで動きます。
ホストは起動のたびに、次の小さなプログラムを内蔵 CPU (FR30 系、ビッグエンディアン) へ書き込んで機能を足します。

- 要求ハンドラのパッチ (24 ビットを超えるアドレスと、複数ワードの書き込みを通すため)
- codec core 1: I2C マスターの制御。バルク転送で届くコマンドパケット (レジスタ・アクセスの列) と、メモリ要求経由の mailbox の 2 経路
- codec core 0: TS エンジンが SDRAM に書いた 192 バイト単位のデータを、エンドポイント 0x81 へ送るパケタイザ

これらは `winusb/tools/zd1fw/zd1fw_gen.py` が生成し、`zd1_firmware.h` に埋め込まれます。
再生成と自動試験は次のとおりです。

```powershell
python winusb/tools/zd1fw/zd1fw_test.py
python winusb/tools/zd1fw/zd1fw_gen.py winusb/src/DriverHost_PX4/zd1_firmware.h
```

`zd1fw_test.py` は、FR30 のエミュレーター上で、アドレス拡張ルーチン、core 1 (mailbox とコマンドパケット)、core 0 の初期化を、模擬の I2C マスターと USB FIFO に対して実行します。
実機のレジスタ動作を再現するものではないため、実機試験の代用にはなりません。

## 動作の流れ

1. `Zd1Device::Init()` で、チップが電源投入直後の状態か (要求ハンドラが既知の内容か、前回のパッチ済みか) を確認し、ハンドラのパッチ、クロック、I2C マスター、core 1 の起動、コマンドチャネルの設定を行います。
   前回の DriverHost_PX4 が残したパッチと core のコードは、内容が一致する場合に限って再利用します。そのため、DriverHost_PX4 の再起動に USB の抜き差しは不要です。
2. DiB8096 をリセットし、`Dib8096::Initialize()` で DiB8000 とチューナーを初期化します。その後は受信機を開くまで省電力にします。
3. `Open` で前面 LED (DiB8000 の GPIO 1) を点灯し、`SetFrequency` で選局します。選局の仕組みは次節のとおりです。
4. ロックを検出したら、USB の DMA を設定し、エンドポイント 0x81 の非同期読み出しを先に発行してから、TS の入力を許可します。TS エンジンと core 0 の設定は、手順 3 の待ち時間に済んでいます。
5. `Close` で入力を止めて LED を消し、DiB8096 を省電力にします。

### I2C の経路

メモリ要求 1 回は約 3 ms、mailbox 方式の I2C 転送 1 回は約 9 ms (検証環境で実測) かかります。
DiB8096 の初期化と選局は 200〜600 回の I2C 転送を伴うため、バルク転送のコマンドチャネル (1 パケットに最大 125 ワード、読み出し 1 回につき 1 往復) を使います。
書き込みは溜めて送り、失敗は次の読み出しか、待ち (`Wait`) の前の送出で分かります。
DiB8000 / DiB0090 のレジスタ・アクセス以外の形の転送と、コマンドチャネルを設定できなかった場合は、mailbox にフォールバックします。

### 選局の高速化

初めて受信するチャンネルは、復調器が伝送パラメーター (モード、ガード間隔、各階層) を自動探索するため約 1.4〜1.5 秒かかります。
2 回目からは次の手法で約 0.55〜0.75 秒 (`Dib8096::Tune`) になります。考え方は、公開されている別実装の選局の短縮 (danything/denpa の PR #1577、#1574) の説明を参考にし、このドライバーの構成で実機に当てて決めました。

| 手法 | 内容 |
| --- | --- |
| ロック値の記録 | 一度ロックしたチャンネルの伝送パラメーターを覚え、次回は自動探索を飛ばして、パラメーター設定から始めます。その値でロックしなければ、記録を捨てて自動探索でやり直します。記録は DriverHost_PX4 のメモリ上だけに持ち、複数台の ZD1 J で共有します。ファイルには保存しないため、DriverHost_PX4 の再起動後の初回は自動探索から始まります |
| データロックの待ち | いずれかの階層が MPEG ロックした時点で選局を終えます。日本の放送は 1 セグの階層のインターリーブが深く (実機では A 層 QPSK 2/3、インターリーブ 4)、その階層を待つと約 0.4 秒余分にかかります |
| captrim の再利用 | 周波数ごとの値を覚え、ADC がほぼ同じ (誤差 50 以内) なら二分探索 (約 165 ms) を省きます |
| 温度校正の間引き | 60 秒以内の測定値を使い回します (約 44 ms) |
| DC 校正 | 周波数に依存しないため、選局ではなく `Initialize` で 1 回だけ行います |
| 待ち | 高分解能タイマーで待ちます (既定の `Sleep` は約 15.6 ms に丸められ、選局全体で約 0.5 秒延びていました)。復調器のステージ 4〜8 以外は 3 ms 周期で確認します。ステージ 4〜8 は、書き込み直後の再起動やシンボル数で数えるタイムアウトを 10 ms 周期の確認を前提に決めているため、10 ms を保ちます |
| ストリーム設定の先行実行 | TS エンジンと core 0 の設定を、選局の待ち時間に 1 ステップずつ進めます (spec 04 章 4.4)。USB の DMA の設定は、コマンドチャネルの通信を乱すため、ロックの後に行います |

実機で採用を見送った案は次のとおりです。

- **ダイバーシティ設定の省略**: DiB8000 のリビジョン `0x8002` では、`setDiversityIn` のたびに追加の書き込み (レジスタ 903 の bit 3) と 30 ms の待ちがあります。設定が前回と同じ場合に省くと、ロックしない選局が出ました (18 回中 1 回)。
- **ストリーム設定の別スレッドでの並行実行**: 制御転送を I2C のバルク転送と同時に走らせると、I2C の読み出しが失敗しました (150 回中 2 回: `-5` と時間切れ)。待ちを呼んだスレッドの中でステップを進める方式に改め、同じ試験で失敗は 0 回になりました。

## 設定ファイル

`DriverHost_PX4.ini` の `[DeviceDefinition15]` が CECH-ZD1 J の定義です。`Type="ZD1"` で `Zd1Device` が選ばれます。

| キー | 既定値 | 内容 |
| --- | --- | --- |
| `UrbBytes` | 65536 | TS の 1 回の読み出し長。1536 バイト (8 ユニット) の倍数、最大 64KiB に丸めます |
| `MaxUrbs` | 16 | 先に発行しておく非同期読み出しの本数 |
| `ReceiverMaxPackets` | 2048 | 受信機のリングバッファのパケット数 (188 バイト単位) |

BonDriver は `BonDriver_CECHZD1J.dll` と `BonDriver_CECHZD1J.ini` の組で、チャンネル定義は他の ISDB-T 機種と共通の `BonDriver_PX4-T.ChSet.txt` を使います。

### ドライバーのインストール

`winusb/pkg/inf/CECH-ZD1J.inf` を追加しました。デバイスインターフェース GUID は他機種と重複しない固有値です。
カタログファイル (`px4_drv_winusb.cat`) は、この INF を含めて `winusb/pkg/signing-tools/sign.ps1` で再生成する必要があります。この変更では再生成しておらず、配布前の作業として残っています。

Zadig などで WinUSB を割り当てた場合は、デバイスインターフェース GUID が INF のものと異なります。
その場合は `DriverHost_PX4.ini` の `[DeviceDefinition15]` の `DeviceInterfaceGUID` を、レジストリ
`HKLM\SYSTEM\CurrentControlSet\Enum\USB\VID_054C&PID_04B2\<個体>\Device Parameters\DeviceInterfaceGUIDs`
の値に書き換えてください。

## 実機の識別値

起動時に読んで分岐します。値は検証環境の 1 台で次のとおりでした。

| 項目 | 値 |
| --- | --- |
| DiB8000 レジスタ 896 / 897 | `0x01B3` / `0x8002` (DiB8000C。自動探索は 2 段階、ダイバーシティ設定のたびに 30 ms 眠る) |
| DiB0090 レジスタ `0x1A` | `0x0223` (MP001、P1DEF、P1G ではない。captrim は二分探索) |

## 制約

- Windows (WinUSB) 版のみです。Linux 版には実装していません。
- 170 MHz 以下 (CBAND) の選局は、ソフトウェア AGC を移植していないため拒否します。日本の地上デジタル放送 (UHF 13〜62ch) には影響しません。
- 信号強度 (`SIGNAL_STRENGTH`) は未実装で `-ENOSYS` を返します。CNR は 1/1000 dB で返します (BonDriver が 1000 で割って dB として扱います)。
- ISDB-Tsb、1 セグ専用経路、ダイバーシティ、TMCC の保存による手動モードは未実装です。
- 選局のたびに TS エンジンと core 0、USB の DMA を再設定します (約 60 回のメモリ要求)。TS エンジンと core 0 の分は選局の待ち時間に重ねていますが、USB の分 (約 20 回) はロックの後です。
- ロック値の記録は、受信できない局では増えません。放送局が伝送パラメーターを変更した場合は、記録でロックしないため自動探索でやり直し、記録を更新します。
- カードスロットはメモリ要求で UART を操作するため、APDU 1 往復に数百 ms かかります。

## 実機試験ツール

`winusb/tools/zd1_probe/` の 2 つは、DriverHost_PX4 を介さずに層ごとの動作を確かめるための道具です。
どちらも vcxproj には含めていません。

```powershell
# bring-up 用。デバイスインターフェース GUID は上記のレジストリ値
x86_64-w64-mingw32-g++ -std=c++17 -O2 -Iwinusb/src/DriverHost_PX4 -Idriver winusb/tools/zd1_probe/zd1_probe.cpp winusb/src/DriverHost_PX4/mb86h47_bridge.cpp winusb/src/DriverHost_PX4/dib8096.cpp -o zd1_probe.exe -lsetupapi -lwinusb -static
zd1_probe.exe "{GUID}" info                      # 状態の読み出しだけ (書き込みなし)
zd1_probe.exe "{GUID}" start                     # 起動手順、DiB8000 の識別
zd1_probe.exe "{GUID}" tune 27 10 out.ts         # 物理チャンネル 27ch を選局し、10 秒受信して TS を保存
zd1_probe.exe "{GUID}" card                      # カードスロットの状態、ATR
zd1_probe.exe "{GUID}" bench                     # メモリ要求と I2C の所要時間
```

`bondriver_test.cpp` は BonDriver の DLL を LoadLibrary し、チャンネルを順に切り替えながら、同期ずれ、PAT の数、continuity error を数えます。

```powershell
bondriver_test.exe BonDriver_CECHZD1J.dll 14,12,9 6    # ChSet の通し番号 (14 = 27ch)、各 6 秒
```

## 検証記録

### 検証環境

- Windows 11 Pro 10.0.26300 (x64)
- CECH-ZD1 J 1 台、WinUSB ドライバー (Zadig、v6.1.7600.16385) 割り当て済み
- 東京タワー系の UHF (NHK総合 27ch、日本テレビ 25ch、TBS 22ch、フジテレビ 21ch、テレビ朝日 24ch、テレビ東京 23ch、TOKYO MX 16ch)、アンテナ接続あり
- B-CAS カードは未挿入
- ビルドは llvm-mingw (Clang/libc++、x86_64) で行いました。DriverHost_PX4.exe と BonDriver_CECHZD1J.dll は、検証用に vcxproj を使わずに同じソースからビルドしたものです

### 結果

| 項目 | 結果 |
| --- | --- |
| 識別 (`zd1_probe start`) | DiB8000 `0x01B3` / `0x8002`、チューナー `0x0223`。仕様の記録と一致 |
| 選局 (27ch) | `LOCK`、CNR 約 24 dB |
| TS | 約 11,600 パケット/秒 (約 17.5 Mbps)。5 秒で 58,026 パケット。188 バイト境界のずれ 0、TEI 0、continuity error 0。PAT の TSID は 32736 (NHK 東京) で、サービス 1024 / 1025 / 1408 |
| 経路 | BonDriver → DriverHost_PX4 → 実機。7 局を連続して切り替え、各局 6 秒受信。全局で同期ずれ 0、continuity error 0 |
| 時間 | `Initialize` 約 0.16 秒 (mailbox のみでは 2.1 秒。DC 校正を含めると BonDriver の `OpenTuner` で約 1.7 秒)。`Dib8096::Tune` は初回約 1.4〜1.5 秒、記録・再利用が効く 2 回目から約 0.55〜0.75 秒 (最適化前は約 1.7〜1.9 秒、mailbox のみでは約 8 秒)。BonDriver の `SetChannel` は 2 回目から平均 約 0.9 秒 (150 回の平均 912 ms、最大 約 1.0 秒)。最初の TS は選局完了から 0.13〜0.2 秒後 |
| 選局の繰り返し | BonDriver 経由で 6 局 × 25 回 (150 回) を連続して切り替え、全回ロックして受信できた。各局 1 秒受信し、同期ずれ 0 |
| DriverHost_PX4 の再起動 | チップを抜き差しせずに、パッチ済みの状態から繰り返し起動できることを確認 |

### 未確認の条件

- MSVC (Visual Studio 2022、v143) でのビルド。vcxproj、vcxproj.filters、build.ps1、CI の定義は更新しましたが、実行していません
- x86 (32 ビット) ビルド
- B-CAS カードの検出後の ATR、T=1 の APDU、WinSCard_PX4 経由の動作 (カード未挿入のため)。UART のレジスタ幅は実機で確認しました (BWT は 24 ビット、CWT は 16 ビット)
- 電波が弱い局、受信できない局での挙動、選局のタイムアウト
- USB 切断中の受信、終了処理の競合、複数台の同時接続
- 数時間〜数日の連続受信 (実施した最長の連続受信は 5 分)
- 放送局が伝送パラメーターを変更した場合の、記録の更新 (記録でロックしなかったときの自動探索への切り替えは、ロジックの確認だけで、実機では起きていません)
- INF とカタログによる WinUSB の導入 (検証環境は Zadig による導入)
