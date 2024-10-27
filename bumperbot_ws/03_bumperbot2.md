## 1. パッケージの作成 & ビルド

```bash
# パッケージの作成
cd ./bumperbot_ws/src/
ros2 pkg create --build-type ament_cmake bumperbot_description

# ビルド
cd ../
colcon build
```

## 2. urdf用のファイル作成

```bash
# xacroファイルの作成
cd src/bumperbot_description/
mkdir urdf
cd urdf/
touch bumperbot.urdf.xacro

# visualization用のmesh. ダウンロードしたファイルをここに格納する
cd ..
mkdir meshes
```

この時点でのフォルダ構成
```bash
|-- CMakeLists.txt
|-- include
|   `-- bumperbot_description
|-- meshes
|   |-- base_link.STL
|   |-- caster_front_link.STL
|   |-- caster_rear_link.STL
|   |-- imu_link.STL
|   |-- wheel_left_link.STL
|   `-- wheel_right_link.STL
|-- package.xml
|-- src
`-- urdf
    `-- bumperbot.urdf.xacro
```

bumperbot.urdf.xacro　の中身は下記
```xml
<?xml version="1.0"?> <!-- XMLファイルのバージョンを宣言 -->

<robot xml:xacro="http://www.ros.org/wiki/xacro" name="bumperbot"> <!-- ロボットモデルの定義。xacroテンプレートを利用し、名前を "bumperbot" に設定 -->

    <link name="base_footprint/"/> <!-- ロボットのベースとなる基準座標 "base_footprint" を定義。特定の形状や位置情報はない -->

    <link name="base_link"> <!-- ロボットの基本リンク "base_link" を定義。 -->
        <visual> <!-- 視覚情報の定義開始。このリンクがどう見えるかを決定 -->
            <origin rpy="0 0 0" xyz="0 0 0"/> <!-- 視覚情報の基準位置と回転角を設定 (ロール、ピッチ、ヨーは全て0) -->
            <geometry> <!-- 形状情報の定義開始 -->
                <mesh filename="package://bumperbot_description/meshes/base_link.STL"/> <!-- STL形式の3Dモデルファイルを指定 -->
            </geometry> <!-- 形状情報の定義終了 -->
        </visual> <!-- 視覚情報の定義終了 -->
    </link> <!-- "base_link" の定義終了 -->

    <joint name="base_joint" type="fixed"> <!-- "base_footprint" と "base_link" の間のジョイントを定義。このジョイントは固定型 (動かない) -->
        <parent link="base_footprint"/> <!-- ジョイントの親リンクを "base_footprint" に設定 -->
        <child link="base_link"/> <!-- ジョイントの子リンクを "base_link" に設定 -->
        <origin rpy="0 0 0" xyz="0 0 0.33"/> <!-- ジョイントの位置と回転を定義。高さが 0.33m  -->
    </joint> <!-- ジョイントの定義終了 -->

</robot> <!-- ロボットモデル定義終了 -->
```

## 3. ビルドする

CMakeLists.txt に下記を追加

```cmake
# added by koba
install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)
```

ビルドする

```bash
# bumpoerbot_ws/src/ に移動
colcon build
```

## 4. 動作確認する

```bash
# bumpoerbot_ws/src/ に移動
# 環境設定
source ./install/setup.bash

# RViz を立ち上げ、robot framework を表示する
ros2 launch urdf_tutorial display.launch.py model:=/work/bumperbot_ws/src/bumperbot_description/urdf/bumperbot.urdf.xacro
```

#### 直前のコマンドの説明

1. **`ros2`**:
   - これは、ROS 2のコマンドラインツールを呼び出しています。`ros2`コマンドを使用して、さまざまなROS 2の機能を実行できます。

2. **`launch`**:
   - これは、`launch`ファイルを実行するためのサブコマンドです。`launch`ファイルは、複数のノードを一度に立ち上げたり、パラメータの設定を行うために使われるスクリプトです。ROS 2では、`.launch.py`というPythonスクリプト形式で記述されます。

3. **`urdf_tutorial`**:
   - これは、ROS 2のパッケージ名です。ここでは、URDF（Unified Robot Description Format）を使ったロボットモデルを扱うためのパッケージで、URDFに関連するチュートリアルやツールが含まれています。

4. **`display.launch.py`**:
   - これは、`launch`ファイルの名前です。このファイルは、ロボットモデルのビジュアライゼーションを行うために利用されます。このファイルは、ロボットモデルを可視化するためにツール（例えば、RViz）を起動したり、その他の必要なセットアップを行うことが一般的です。

5. **`model:=/work/bumperbot_ws/src/bumperbot_description/urdf/bumperbot.urdf.xacro`**:
   - `model:=` は、launchファイルに渡す引数を指定する形式です。ここでは、URDF（Unified Robot Description Format）のロボットモデルファイルのパスを指定しています。
   - `/work/bumperbot_ws/src/bumperbot_description/urdf/bumperbot.urdf.xacro` は、ロボットモデルのファイルパスです。拡張子 `.xacro` は、Xacro（XML Macros）というURDFファイルを生成するためのXMLベースのマクロファイルです。これを使って、繰り返しや複雑な構造をより簡潔に記述できます。

### 全体の意味:

このコマンドは、ROS 2で提供される`urdf_tutorial`パッケージの`display.launch.py`というlaunchファイルを使って、指定された`bumperbot.urdf.xacro`モデルファイルを読み込み、そのロボットモデルを表示するための設定を行い、ビジュアライゼーションツール（おそらくRViz）を起動するものです。
