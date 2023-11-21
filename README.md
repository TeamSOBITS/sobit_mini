# SOBIT MINI

<!--目次-->
<details>
    <summary>目次</summary>
    <ol>
        <li>
            <a href="概要">概要</a>
        </li>
    </ol>
</details>

<!--レポジトリの概要-->
## 概要
![](sobit_mini/img/sobit_mini.png)

SOBITSが開発した双腕型モバイルマニピュレータ（SOBIT MINI）を動かすためのライブラリです

> [!WARNING]
> 初心者の場合，実機のロボットを扱う際に，先輩方に付き添ってもらいながらロボットを動かしましょう．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## Prerequisites
以下の環境で動作します．
- OS: Ubuntu 20.04 
- ROS distribution: noetic Kame

### How to use
まず，以下のコマンドを入力して，SOBIT MINIを動かすための環境設定を行います．
この設定は，初回のみに行う作業ですので，1度行ったことのある人は飛ばしてください．

※ 開発するPCで，SOBIT EDUやSOBIT PROを動かしたことがある場合も，この作業は必要ありません．

```bash:
$ cd sobit_mini
$ bash sobit_setup.sh
```

以下のコマンドを入力することで，SOBIT MINIを起動することができます．
これにより，SOBIT MINIのモータやRGB-Dカメラ，測域センサ(Lidar)などのデバイスが起動します．
また，それと同時にRvizも起動します．

:warning: ロボットをコンテナで動かす場合，動かしたいデバイスをホストPCと接続してから，コンテナを立ち上げてください．
コンテナを立ち上げてからデバイスとの接続を行う場合，ロボットが動かない場合があります．

```bash:
$ roslaunch sobit_mini_bringup minimal.launch
```
