# Omatcha_actions
## 本パッケージについて
- このパッケージは株式会社アールティ様([公式サイト](https://rt-net.jp/))より販売されているCRANE-X7をROS環境で
運用する際に使用できるパッケージです。使用するにはCRANE-X7本体とアールティ様から配布されているCRANE-X7[オリジナルパッケージ](https://github.com/rt-net/crane_x7_ros)、環境構築が必要となります。  
本パッケージは上記の[オリジナルパッケージ](https://github.com/rt-net/crane_x7_ros)を元に2023年度の設計製作論実習3内でチーム大三元が開発を行なったものです。  
本パッケージはアールティ様の定めたライセンスに則って開発しております。詳しくは**LICENSE**をご確認ください。

- 本アプリケーションはCRANE-X7を使ってお抹茶を点てるアプリケーションです。CRANE-X7、Intel RealSense、お抹茶の粉が入った容器、茶筅、お湯と茶碗を用意して配置し、パッケージ内のスクリプトを実行することで、お抹茶を点てることができます。  
この[Omatcha_detection](https://github.com/Orbital-G/Omatcha_detection)とOmatcha_actionsの二つのパッケージをセットで使用する必要があります。合わせてのインストールをお願いします。  
このパッケージはアプリケーションのうち、アームの動作を担うパッケージです。Omatcha_detectionで物体検出を行います。
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
ROSのインストールとワークスペースの構築を例に行なってください。  
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

## 実行方法
roscoreコマンドとオリジナルのパッケージ内のlaunchファイルを使用してROSノードが通信できるようにします。  
その後、Omatcha_detection内のプログラムを実行してください。
```sh
roscore &
roslaunch crane_x7_bringup demo.launch
Omatcha_detect
```

次にターミナルのタブを新たに開いてセットのパッケージOmatcha_actions内のプログラムを実行してください。
```

```