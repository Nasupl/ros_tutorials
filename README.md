# ros_tutorial with turtlesim_mcl
## はじめに
ros_tutorialのturtlesimにおいて，以下の条件を設定し，その条件のもとでMonte Carlo Localizationを行うことができるようにしたパッケージです．
条件:

* turtlesim内では速度計算にノイズが生じるため，移動距離は制御入力の通りにはならない
* turtlesimからは直接turtleの姿勢を取得することはできず，ノイズの加えられた姿勢のみを得ることができる

## 使い方

1. ビルドする
```
catkin build turtlesim turtlesim_mcl
```
turtlesim本体にも手を加えている都合上，turtlesimもビルドする必要があります．
1. 以下のコマンドを実行する
```
roslaunch turtlesim_mcl turtlesim_mcl.launch
```
これでturtlesim本体と，mclに関わるノードが実行されます
2. 別ターミナルで以下のコマンドを実行する
```
rosrun turtlesim turtle_teleop_key
```
これで，turtleを操作することができます．

## 問題点
フォークレポジトリの都合Issuesがないので取り急ぎここに．バグだらけですが大目に見ていただければ…🙏

* どれがパーティクルでどれが本物のカメなのか判別がつかない
    * パーティクルのカメはSetPenをoffにして，軌道が描かれないようにしたい
* リサンプリング本当にされてるのかこれ？
    * ちゃんと重みが変化しているのか検証していないので検証するべき
