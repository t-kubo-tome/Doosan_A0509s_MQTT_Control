# Doosan_A0509s_MQTT_Control マニュアル

MetaworkMQTTプロトコルでの、Doosan A0509sの実機側の制御プログラムの使い方を説明する。

制御プログラムの開発環境での実行を前提とする。

## クイックスタート

**1 ロボットの起動**

- コントローラの底面右のスイッチ（電源ケーブルの近くにある）を倒してコントローラの電源をオンにする
- ティーチペンダントの左上にある電源ボタンを長押ししてロボット、コントローラ、ティーチペンダントの電源をオンにする
- 緊急停止ボタンがOFFである（上に引いてある）ことを確認する。また、ロボット制御時に、緊急停止したい場合は緊急停止ボタンをONにする（押す）ことを確認しておく
- ハンド（qb SoftHand Industry）は、電源ケーブルを接続することで、電源がオンになる

**2 PCでのプログラム実行**

- ロボット接続用PCとロボットコントローラはLAN接続する。開発環境では、ロボットコントローラは`192.168.6.43/24`に設定している。設定方法は、TPの`設定` > `ネットワーク`で、ラジオボタンで`固定アドレス`を設定し、例えば、IPアドレスに`192.168.6.43`、サブネットマスク`255.255.255.0`、基本ゲートウェイ`192.168.6.254`、選好DNSサーバ`192.168.6.254`に設定する
- ロボット接続用PCとハンドはLAN接続する。開発環境では、ハンドは`192.168.5.44/24`に設定しハンド
- 開発環境では、ロボット接続用PCから、2つのLANケーブルをロボットコントローラとハンドに接続し、ロボットコントローラ通信用のPC側のアドレスを`192.168.6.1/24`、ハンド用を`192.168.5.1/24`に設定している。別の方法として、ロボットコントローラとハンドをスイッチなどを介してPCに接続し、同じLANで通信することも考えられる。その場合、ロボットコントローラのIPアドレスを`192.168.5.43/24`に変更することを推奨する（ハンドより変更が容易なため）。変更する場合は、環境変数`src/doosan/.env`の`ROBOT_IP`も変更すること
- プログラムのインストールの方法は、[インストール](#インストール)参照
- インストール後には、ロボット制御コードは以下のコマンドで実行できる

まず仮想環境を有効にする。

```sh
source .venv/bin/activate
```

その状態でロボット制御コードを実行する。

**GUIを使用する場合**

```sh
python src/main.py --gui
```

以下のようなGUI画面が起動する。

![](assets/manual/gui.png)

ロボット制御側で、MQTTでの制御を受け付けるようにするためには、以下の手順で操作を行う。

1. `ConnectRobot`でロボットに接続
2. `ConnectMQTT`でMQTTサーバーに接続
3. `EnableRobot`でロボットの状態に応じて電源ONやロボットコントローラの制御プログラム起動などを行う。すべて成功すれば`Enabled`のランプが緑色に点灯する
4. `StartMQTTControl`でMQTTでの制御を受け付けるようにする。成功すれば`MQTTControl`のランプが緑色に点灯する

MQTTでの制御中に、エラーが起きた場合は、1度自動復帰を試みる。成功した場合、そのままMQTTでの制御が可能である。失敗した場合は、MQTTでの制御が止まる。

エラーを解除して大丈夫な状態（アームが障害物に衝突してエラーが起きた後に、障害物を取り除くなどした安全な状態）であれば、`EnableRobot`でロボットコントローラの制御プログラムを起動すればよい。

`ReleaseHand`でハンドを最大まで開くことができる。`TidyPose`でロボットの先端がロボットの台の中央付近になり、先端が縦向きになる、片付け用の姿勢に移動できる。`ChangeLogFile`でログ出力ディレクトリを現在の時刻の`log/<YY-mm-dd>/<HH-MM-SS>`に切り替えることができる。`DisableRobot`でモーターをOFFにできる。閉じるボタンでロボット制御コードを終了できる。

ジョグ（関節空間でのジョグは`Joint Jog`, ベース座標系（X, Y, X, RX, RY, RZ）でのジョグは`TCP Jog`）が可能である。ジョグは長押しするとジョグ速度が大きくなる。ベース座標系でのジョグは特異姿勢付近でエラーになることがある。

**GUIを使用しない場合**

```sh
python src/main.py
```

起動時に自動的にロボットとMQTTサーバーに接続する。

ロボットの制御は、MQTTトピック`dev/<ROBOT_UUID>/command`（`ROBOT_UUID`は`src/doosan/.env`で環境変数として指定したロボットのUUID）に、JSON形式のコマンドのメッセージをパブリッシュすることで制御できる。

ロボットの状態は、MQTTトピック`robot/<ROBOT_UUID>`にパブリッシュされる。

ロボット制御側で、MQTTでの制御を受け付けるようにするためには、以下の手順で操作を行う。例として、`mosquitto_pub`コマンドを使用する場合を示す。

```sh
# 別ターミナルで実行する
# MQTT_HOSTは環境変数として指定したMQTTサーバーのホスト名、ROBOT_UUIDは環境変数として指定したロボットのUUID
# 1. ロボット有効化。成功すればトピック`robot/<ROBOT_UUID>`中の`enabled`がtrueになる
mosquitto_pub -h <MQTT_HOST> -t "dev/<ROBOT_UUID>/command" -m '{"command": "enable"}'
# 2. MQTT制御開始。成功すればトピック`robot/<ROBOT_UUID>`中の`mqtt_control`がtrueになる
mosquitto_pub -h <MQTT_HOST> -t "dev/<ROBOT_UUID>/command" -m '{"command": "start_mqtt_control"}'
```

コマンドのメッセージはJSON形式で、以下のいずれかの形式で送信する:

```json
{"command": "<コマンド名>"}
{"command": "<コマンド名>", "params": {"<パラメータ名>": <値>, ...}}
```

サポートされるコマンド一覧:

| コマンド | パラメータ | 説明 |
|----------|------------|------|
| enable | - | アームを移動させるための電源をONにする |
| disable | - | アームを移動させるための電源をOFFにする |
| tidy_pose | - | ロボットを待機姿勢に移動する |
| release_hand | - | ハンドを最大まで開く |
| start_mqtt_control | - | MQTTでのリアルタイム制御を開始する |
| stop_mqtt_control | - | MQTTでのリアルタイム制御を停止する |
| change_log_file | - | ログ出力ディレクトリを現在時刻の<br>`log/<YY-mm-dd>/<HH-MM-SS>`に切り替える |
| shutdown | - | ロボット制御プログラムを終了する |
| jog_joint | joint: int,<br>direction: float | 関節角度制御によるジョグ。<br>jointは関節の順番を表し、0-5の値を取りうる。<br>directionは角度の移動量（deg）を表す |
| jog_tcp | axis: int,<br>direction: float | TCP座標系でのジョグ。<br>axisは軸を表し、0:X, 1:Y, 2:Z, 3:RX, 4:RY, 5:RZに対応。<br>directionは移動量（mm）を表す |
| get_command_list | - | サポートされているコマンド一覧を取得する。<br>結果はレスポンストピックに送信される |
| get_joint_names | - | ジョイントの数と名前を取得する。<br>結果はレスポンストピックに送信される |

**レスポンストピック**

`get_command_list`や`get_joint_names`などの情報取得コマンドの結果は、MQTTトピック`dev/<ROBOT_UUID>/response`にJSON形式でパブリッシュされる。

`get_command_list`のレスポンス例:

```json
{
  "devId": "<ROBOT_UUID>",
  "command": "get_command_list",
  "timestamp": 1770281593.469537,
  "supported_commands": {
    "enable": {"params": [], "description": "アームを移動させるための電源をONにする"},
    ...
  }
}
```

`get_joint_names`のレスポンス例:

```json
{
  "devId": "<ROBOT_UUID>",
  "command": "get_joint_names",
  "timestamp": 1770281593.469537,
  "joint_names": ["J1", "J2", "J3", "J4", "J5", "J6"]
}
```

**3 ロボットの終了**

- TPの画面右下の電源ボタンから電源をオフにする
- コントローラの底面右のスイッチ（電源ケーブルの近くにある）を倒してコントローラの電源をオフにする

## インストール

Doosan A0509sの制御には、API-DRFLを使用する。`src/vendor`には、必要なヘッダーファイルとライブラリファイル（Ubuntu 22.04用）を格納している。元の最新のレポジトリ（基本的にダウンロード不要）は、https://github.com/DoosanRobotics/API-DRFL にある。本システムでは、バージョン1.29を使用している。ドキュメントはhttps://manual.doosanrobotics.com/en/api/1.29/Publish/ にある。

ハンド（qb SoftHand Industry）の制御には、qb SoftHand Industry APIを使用する。`src/vendor`には、必要なヘッダーファイルとライブラリファイル（x64 Linux用の共有ライブラリのみ）を格納している。元のライブラリファイルおよびソースコード（基本的にダウンロード不要）は、https://qbrobotics.com/wp-content/uploads/2021/07/qbsofthand_industry_api_1.0.3.zip を使用している（ユーザーマニュアル https://qbrobotics.com/wp-content/uploads/2023/01/qb-SoftHand-Industry-User-Manual-ENG-DOOSAN.pdf を参照）。

まず、Pythonの仮想環境を作成し、起動しておく。

次に、必要なライブラリをインストールする:

```sh
pip install -r src/requirements.txt
```

次に、`API-DRFL`、`qb SoftHand Industry API`およびそのPythonラッパーをビルド、インストールする:

```sh
./build.sh
```

環境変数は、`src/doosan/.env.example`の変数を適宜書き換え、`src/doosan/.env`に変更することで有効になる。

リアルタイムスケジューラを利用するための設定を行う。コマンド`python`のシンボリックリンクをたどった最終的なバイナリに対して、リアルタイムスケジューラを利用するための権限を付与する。例えば、バイナリが`/usr/bin/python3.10`の場合は、

```sh
sudo setcap cap_sys_nice=eip /usr/bin/python3.10
```

とする。但し、システムのバイナリに権限が付与される場合は、本システム以外にも影響を与える可能性があることに注意する必要がある。

`setcap`による設定をしない場合、毎回`sudo`を付けてコマンドを実行することで一時的にバイナリ（`python`）にリアルタイムスケジューラを利用するための権限を付与することができる。

```sh
sudo python src/main.py
```

特に仮想環境の`python`を用いる場合は`python`を`$(which python)`に置き換える必要がある。

## トラブルシューティング

- GUIから`ConnectRobot`ボタンを押しても、ロボット、ハンドに接続できない場合がある。GUIを閉じて、再度起動してから`ConnectRobot`ボタンを押すと接続できる場合がある。何度も接続できない場合は、まず、PC、ロボットコントローラ、ハンドのIPアドレス、LANケーブルの接続を確認して再度接続を試みる。次に、ロボットコントローラまたはハンドを再起動して再度接続を試みる
