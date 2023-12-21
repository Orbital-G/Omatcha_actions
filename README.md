# Omatcha_actions
## 本パッケージについて
- このパッケージは株式会社アールティ様([公式サイト](https://rt-net.jp/))より販売されているCRANE-X7をROS環境で
運用する際に使用できるパッケージです。使用するにはCRANE-X7本体とアールティ様から配布されているCRANE-X7[オリジナルパッケージ](https://github.com/rt-net/crane_x7_ros)、環境構築が必要となります。  
本パッケージは上記の[オリジナルパッケージ](https://github.com/rt-net/crane_x7_ros)を元に2023年度のロボット設計製作論実習3内でチーム大三元が開発を行なったものです。  

- このパッケージのact_matcha.pyとact_chasen.pyは過去のロボット設計製作論実習3のリポジトリから2021-RobotDesign3-team2様の[crane_x7_ros_test](https://github.com/2021-RobotDesign3-team2/crane_x7_ros_test)のmain_move.pyを参考に作成しました。

- 本アプリケーションはCRANE-X7を使ってお抹茶を点てるアプリケーションです。  
CRANE-X7、Intel RealSense、お抹茶の粉が入った容器、茶筅、お湯と茶碗を用意して配置し、パッケージ内のスクリプトを実行することで、お抹茶を点てることができます。  
このOmatcha_actionsと[Omatcha_detection](https://github.com/Orbital-G/Omatcha_detection)の二つのパッケージをセットで使用する必要があります。合わせてのインストールをお願いします。  

- このパッケージはアプリケーションのうち、CRANE-X7の制御を担うパッケージです。[Omatcha_detection](https://github.com/Orbital-G/Omatcha_detection)で物体の検出を行います。

## 動作環境
* OS: Ubuntu20.04  
* ROS Distribution: ROS Noetic

## 使用機材
* CRANE-X7  
* Intel RealSense  
* 茶筅  
* 茶碗  
* 抹茶を入れる容器(直径30mm程度)  

## 環境構築
[こちらのスライド](https://github.com/ryuichiueda/my_slides/blob/master/robotdesign3_2021/lesson1.md)を元に
ROSのインストールとワークスペースの構築を行ってください。  
以降、ワークスペースはcatkiin_wsを例として説明します。  
1. catkin_ws/src内にCRANE-X7のROSパッケージをインストールしてください。
```sh
cd ~/catkin_ws/src  
git clone https://github.com/rt-net/crane_x7_ros.git  
```

2. catkin_ws/src内に二つのパッケージOmatcha_detectionとOmatcha_actionsをインストールしてビルドしてください。
```sh
cd ~/catkin_ws/src
git clone https://github.com/Orbital-G/Omatcha_detection.git
git clone https://github.com/Orbital-G/Omatcha_actions.git
cd ~/catkin_ws  
catkin_make  
```

3. RealSenseのインストール
RealSenseを使用するためにワークスペース内にソースコードをダウンロードしてパッケージをビルドしてください。
```sh
cd ~/catkin_ws/src/
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ~/catkin_ws/
catkin_make
```

## 実行方法
1. CRANE-X7をPCに接続し、以下のコマンドでデバイスドライバに実行権限を与えCRANE-X7とPCが通信できる状態にします。
```sh
sudo chmod 666 /dev/ttyUSB0

```

2. オリジナルのパッケージ内のlaunchファイルを使用してROSノードが通信できるようにします。
```sh
roslaunch crane_x7_bringup demo.launch
```
3. 新規タブを開き以下のコマンドでRealSenseを使用できる状態にしておきます。
```sh
roslaunch realsense2_camera rs_camera.launch  
```

4. CRANE-X7の正面に茶碗を、周囲の離れたところに抹茶の入った容器、茶筅を配置しください。お湯を入れる動作には対応しておりません。お湯は茶碗にあらかじめ入れてください。  
なお、このアプリケーションは液体を扱います。機器の浸水等に注意してください。 

5. 新規タブを開いてOmatcha_detection内のスクリプトを実行してください。  
detect_matcha.py:抹茶の容器を探すスクリプト  
detect_chasen.py:茶筅を探すスクリプト
```sh
rosrun Omatcha_detection detect_matcha.py
rosrun Omatcha_detection detect_chasen.py
```

6. 新規タブを開いてセットのパッケージOmatcha_actions内のスクリプトを実行してください。  
act_matcha.py:抹茶の容器を掴むスクリプト  
act_chasen.py:茶筅を掴むスクリプト
```sh
rosrun Omatcha_actions act_matcha.py
rosrun Omatcha_actions act_chasen.py
```
CRANE-X7が物体を探し、検出すると物体を掴んだのち茶碗の上で所定のアクションを行います。  
実際に本アプリケーションを実行した際の動画です。


https://github.com/Orbital-G/Omatcha_detection/assets/147364200/397026eb-578c-4178-97c3-8467ff378d24


## ライセンス
このパッケージOmatcha_actionsはアールティ様の定めたライセンスに則って開発しております。  
詳しくは**LICENSE**をご確認ください。
