## 1. 準備
- コマンド```colcon build```で以下のフォルダが作られる
    - build
    - install
    - log

```bash
mkdir bumperbot_ws
cd bumperbot_ws
mkdir src
colcon build
```

## 2. Python と Cpp 用 の 新しい ROS 2 パッケージを作成
```bash
cd src/
ros2 pkg create --build-type ament_python bumperbot_py_examples
ros2 pkg create --build-type ament_cmake bumperbot_cpp_examples
```

### 2-1. フォルダとファイルの役割

1. **`bumperbot_py_examples/`**
   - これはROS 2パッケージのルートディレクトリです。このディレクトリの中にROS 2で動作するために必要なファイルやディレクトリが含まれています。

2. **`bumperbot_py_examples/__init__.py`**
   - このファイルは、`bumperbot_py_examples`ディレクトリがPythonパッケージであることを示すためのもので、空でも構いません。このファイルを置くことで、Pythonがこのディレクトリをモジュールとして認識します。

3. **`bumperbot_py_examples/simple_publisher.py`**
   - このファイルは、ROS 2のノードを記述したPythonスクリプトです。シンプルなパブリッシャーノードが実装されています。先ほど説明したコードのように、このファイルの中でROS 2のパブリッシュ機能を使ってメッセージをトピックに送信する役割を持ちます。

4. **`resource/`**
   - このフォルダは、ROS 2パッケージに関連するリソースファイル（例えば、設定ファイルやメディアファイルなど）を置くために使用されることが多いです。具体的な中身はここからは見えませんが、通常はパッケージの補助ファイルが格納されます。

5. **`test/`**
   - テスト関連のファイルが格納されるフォルダです。ROS 2では、単体テストや統合テストのスクリプトを含めることが推奨されています。ここには例えば、テスト用のROSノードやシミュレーション環境の設定が含まれることがあります。

6. **`package.xml`**
   - ROS 2パッケージのメタデータを記述するXMLファイルです。このファイルには、パッケージの名前、バージョン、依存関係、メンテナ情報などが記述されます。ROS 2のビルドシステムや他のパッケージがこのパッケージを正しく理解するために重要な役割を果たします。

7. **`setup.cfg`**
   - Pythonパッケージに関する追加の設定を記述するファイルです。通常は、パッケージのビルドやインストールに関する情報が含まれます。例えば、`flake8`などのコードスタイルチェッカーの設定が記載されていることがあります。

8. **`setup.py`**
   - Pythonパッケージをインストールするためのスクリプトです。このファイルには、`setuptools`を使ってパッケージをビルドしたり、依存関係を管理するための指示が含まれています。ROS 2では、Pythonベースのパッケージに関してこのファイルを使って、`colcon build`コマンドでビルドされます。

### 2-2. まとめ
このパッケージは、ROS 2のPythonベースのパブリッシャーノードを実装するためのものです。`simple_publisher.py`がパッケージの主要なノードファイルであり、`package.xml`や`setup.py`などがパッケージのビルド、インストール、依存関係管理に使われます。また、`resource/`と`test/`フォルダは、追加のリソースやテストのために使われます。


## 3. ROS2パッケージをビルドする
- colcon は ROS 2 のためのビルドツールです。このコマンドは、src/ ディレクトリにあるすべてのパッケージをビルドします。ビルドが成功すると、ビルドされた成果物が install/ ディレクトリに配置されます。
```bash
cd ..
colcon build
```

## 4. 現在のシェルセッションに ROS 2 の環境設定が適用する
- setup.bash スクリプトを実行します。
- これにより、現在のシェルセッションに ROS 2 の環境設定が適用されます。
- 環境変数が設定され、ROS 2 のノードやツールにアクセスできるようになります。
- ```. setup.bash``` は ```source setup.bash``` と同等です。

```bash
cd install
source setup.bash
```

## 5. パッケージのリストの表示
- 現在の ROS 2 ワークスペース内で利用可能なすべてのパッケージのリストを表示
- bumperbot_xxx というパッケージは、このターミナルのみで参照可能
- 他のターミナルでパッケージを有効にするには source setup.bash を実行する必要がある

```bash
ros2 pkg list
```

## 6. サンプルのPublisherの作成 (Python)

### 6-1. Pythonコードの作成
- [simple_puslisher.py](./src/bumperbot_py_examples/bumperbot_py_examples/simple_publisher.py) を新規に作成する.
- 詳細はファイルを参照.

```bash
cd ../src/bumperbot_py_examples/bumperbot_py_examples
touch simple_publisher.py
```

### 6-2. setup.py の entry_point の修正
- entry_point で指定する部分は、ROS 2のコンソールから直接コマンドとしてPythonノードを実行できるように設定するものです。
- 具体的には、以下のように simple_publisher コマンドが登録されます：
- setup.py の entry_point を以下のように編集する

```python
entry_points={
        'console_scripts': [
        'simple_publisher = bumperbot_py_examples.simple_publisher:main'
        ],
```

#### 目的-1. コマンドラインでの実行
- simple_publisher をコマンドラインから直接呼び出せるようになります。- これにより、ターミナルから ```ros2 run bumperbot_py_examples simple_publisher``` のように実行できます。

#### 目的-2. スクリプトのエントリーポイントの定義
- bumperbot_py_examples/simple_publisher.py の中で定義されている main() 関数が、このコマンド実行時に呼び出されます。
- simple_publisher:main の部分は、simple_publisher.py ファイル内の main 関数を指定しています。

#### 実行の流れ:
- ```ros2 run``` コマンドが実行されると、ROS 2は entry_point で定義されたパッケージの指定されたスクリプト（main 関数）を起動します。
- これにより、ノードを手動でPythonスクリプトを指定せずに、ROS 2の標準的なインターフェースで実行できるようになります。

### 6-3. package.xml の修正

```xml
<license>TODO: License declaration</license>

<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

#### 目的-1. 依存関係の宣言
- <exec_depend> タグを使って、パッケージが実行されるために必要なライブラリやパッケージを宣言します。この場合、以下の2つの依存関係を追加しています.
  - rclpy: <br>
  ROS 2のPythonクライアントライブラリで、ROS 2ノードを作成・操作するために必要です。

  - std_msgs: <br>
  標準的なメッセージタイプ（この場合は String 型）を使用するためのメッセージパッケージです。

#### 目的-2. パッケージビルドと実行時の依存関係管理:
- ROS 2のビルドツール（例えば colcon）は、package.xml に記載された依存関係を確認し、必要なパッケージを自動的にインストールしたり、ビルド時にそれらが存在するかをチェックします。
- これにより、rclpy や std_msgs がこのパッケージで正しく利用できるようになります。
 
#### 実行の流れ
- パッケージをビルドするときに、この依存関係リストが読み込まれ、システム上に必要なライブラリが揃っているかを確認します。
- ない場合は警告が表示されるか、rosdep などを使って自動的に解決します。

### 7. サンプルの動作確認 (Python)

#### 7-1. simple_publisherノードを実行
```bash
# 新しいターミナルに移動

# ROS 2ワークスペース (bumperbot_ws) に移動する
cd bumperbot_ws  

# colconを使ってワークスペース内の全てのパッケージをビルドする
colcon build  

# ビルド後に生成された環境設定ファイルをソースし、ROS 2環境をセットアップする
source ./install/setup.bash  

# bumperbot_py_examplesパッケージ内のsimple_publisherノードを実行する
ros2 run bumperbot_py_examples simple_publisher  
```

#### 7-2. トピックの確認

```bash
# 新しいターミナルに移動

# 現在アクティブな全てのトピックのリストを表示する
ros2 topic list  

# /chatter トピックのメッセージをコンソールに表示する
ros2 topic echo /chatter  
```

```
data: 'Hello ROS2 - counter: 113'
---
data: 'Hello ROS2 - counter: 114'
---
data: 'Hello ROS2 - counter: 115'
---
data: 'Hello ROS2 - counter: 116'
---
```

```bash
# 新しいターミナルに移動

# /chatter トピックに関する詳細な情報を表示し、パブリッシャーとサブスクライバーの詳細を含む
ros2 topic info /chatter --verbose  
```

```
Type: std_msgs/msg/String

Publisher count: 1

Node name: simple_publisher
Node namespace: /
Topic type: std_msgs/msg/String
Topic type hash: RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18
Endpoint type: PUBLISHER
GID: 01.0f.eb.7d.f3.86.0b.6f.00.00.00.00.00.00.13.03
QoS profile:
Reliability: RELIABLE
History (Depth): UNKNOWN
Durability: VOLATILE
Lifespan: Infinite
Deadline: Infinite
Liveliness: AUTOMATIC
Liveliness lease duration: Infinite

Subscription count: 1

Node name: _ros2cli_35561
Node namespace: /
Topic type: std_msgs/msg/String
Topic type hash: RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18
Endpoint type: SUBSCRIPTION
GID: 01.0f.eb.7d.e9.8a.4c.43.00.00.00.00.00.00.07.04
QoS profile:
Reliability: RELIABLE
History (Depth): UNKNOWN
Durability: VOLATILE
Lifespan: Infinite
Deadline: Infinite
Liveliness: AUTOMATIC
Liveliness lease duration: Infinite
```

```bash
# /chatter トピックのメッセージの発行頻度（Hz）を測定し、表示する
ros2 topic hz /chatter  
```

```
average rate: 1.000
        min: 1.000s max: 1.000s std dev: 0.00003s window: 3
average rate: 1.000
        min: 0.999s max: 1.001s std dev: 0.00052s window: 5
average rate: 1.000
        min: 0.999s max: 1.001s std dev: 0.00048s window: 7
average rate: 1.000
        min: 0.999s max: 1.001s std dev: 0.00044s window: 9
average rate: 1.000
        min: 0.999s max: 1.001s std dev: 0.00043s window: 10
average rate: 1.000
        min: 0.999s max: 1.001s std dev: 0.00041s window: 11
average rate: 1.000
        min: 0.999s max: 1.001s std dev: 0.00040s window: 12
average rate: 1.000
        min: 0.999s max: 1.001s std dev: 0.00039s window: 14
```


## 8. サンプルのPublisherの作成 (C++)

### 8-1. C++コードの作成
- [simple_puslisher.cpp](./src/bumperbot_cpp_examples/src/simple_publisher.cpp) を新規に作成する.
- 詳細はファイルを参照.

```bash
cd ../src/bumperbot_cpp_examples/src/
touch simple_publisher.cpp
```

### 8-2. CMakeLists.txt の修正
- 下記を追加

```cpp
#----------------------------------------------#
# added by koba
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#
add_executable(simple_publisher 
                src/simple_publisher.cpp)
#
ament_target_dependencies(simple_publisher 
                          rclcpp 
                          std_msgs)

install(TARGETS     simple_publisher
        DESTINATION lib/${PROJECT_NAME}
)
#----------------------------------------------#
```

### 8-3. package.xml の修正
- 下記を追加

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

### 9. サンプルの動作確認 (C++)

#### 9-1. simple_publisherノードを実行
```bash
# 新しいターミナルに移動

# ROS 2ワークスペース (bumperbot_ws) に移動する
cd bumperbot_ws  

# colconを使ってワークスペース内の全てのパッケージをビルドする
colcon build  

# ビルド後に生成された環境設定ファイルをソースし、ROS 2環境をセットアップする
source ./install/setup.bash  

# bumperbot_py_examplesパッケージ内のsimple_publisherノードを実行する
ros2 run bumperbot_cpp_examples simple_publisher  
```

#### 9-2. トピックの確認

```bash
# 新しいターミナルに移動

# 現在アクティブな全てのトピックのリストを表示する
ros2 topic list  

# /chatter トピックのメッセージをコンソールに表示する
ros2 topic echo /chatter  
```

#### 9-3. トピックの確認

```bash
# 新しいターミナルに移動

# 現在アクティブな全てのトピックのリストを表示する
ros2 topic list

# /chatter トピックのメッセージをコンソールに表示する
ros2 topic echo /chatter
```

```
data: 'Hello ROS2 - counter: 110'
---
data: 'Hello ROS2 - counter: 111'
---
data: 'Hello ROS2 - counter: 112'
---
data: 'Hello ROS2 - counter: 113'
---
data: 'Hello ROS2 - counter: 114'
---
```

## 10. サンプルの Subscriber の作成 (Python)

### 10-1. Pythonコードの作成
- [simple_subscriber.py](./src/bumperbot_py_examples/bumperbot_py_examples/simple_subscriber.py) を新規に作成する.
- 詳細はファイルを参照.

```bash
cd ../src/bumperbot_py_examples/bumperbot_py_examples
touch simple_subscriber.py
```

### 10-2. setup.py の entry_point の修正
- entry_point で指定する部分は、ROS 2のコンソールから直接コマンドとしてPythonノードを実行できるように設定するものです。
- 具体的には、以下のように simple_subscriber コマンドが登録されます：
- setup.py の entry_point を以下のように編集する

```python
entry_points={
        'console_scripts': [
        'simple_publisher  = bumperbot_py_examples.simple_publisher:main' ,
        'simple_subscriber = bumperbot_py_examples.simple_subscriber:main'
        ],
```

### 10-3. package.xml の修正
- 下記の変更は、publisherで実施済み
```xml
<license>TODO: License declaration</license>

<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

### 11. サンプルの動作確認 (Python)

#### 11-1. simple_subscriber ノードを実行
```bash
# 新しいターミナルに移動

# ROS 2ワークスペース (bumperbot_ws) に移動する
cd bumperbot_ws  

# colconを使ってワークスペース内の全てのパッケージをビルドする
colcon build  

# ビルド後に生成された環境設定ファイルをソースし、ROS 2環境をセットアップする
source ./install/setup.bash  

# bumperbot_py_examplesパッケージ内の simple_subscriber ノードを実行する
ros2 run bumperbot_py_examples simple_subscriber
```

#### 11-2. トピックの確認

```bash
# 新しいターミナルに移動

# 現在アクティブな全てのトピックのリストを表示する
ros2 topic list  

# /chatter トピックのメッセージをコンソールに表示する
ros2 topic echo /chatter  

# /chatter トピックに関する詳細な情報を表示し、パブリッシャーとサブスクライバーの詳細を含む
ros2 topic info /chatter --verbose  
```

- publisher が 0, subscriber が 1.

```
Type: std_msgs/msg/String

Publisher count: 0

Subscription count: 1

Node name: simple_subscriber
Node namespace: /
Topic type: std_msgs/msg/String
Topic type hash: RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18
Endpoint type: SUBSCRIPTION
GID: 01.0f.eb.7d.fe.52.8d.8a.00.00.00.00.00.00.13.04
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite
```

#### 11-3. Python間でのトピックの確認

- publisher からメッセージを送る

```bash
# /chatter トピックにメッセージをパブリッシュするためのコマンド
# ROS2のトピック通信を使用して、他のノードとデータを送受信する際に使用されます。
# このコマンドは、指定したトピックにメッセージを送信します。

# 使用例: /chatter トピックに "Hello ROS2" というメッセージをパブリッシュする
# これにより、/chatter トピックをサブスクライブしている他のノードにメッセージが送信されます。
ros2 topic pub /chatter std_msgs/msg/String "data: 'hello ros2 ok'"
```

```
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='hello ros2 ok')

publishing #2: std_msgs.msg.String(data='hello ros2 ok')

publishing #3: std_msgs.msg.String(data='hello ros2 ok')

publishing #4: std_msgs.msg.String(data='hello ros2 ok')

publishing #5: std_msgs.msg.String(data='hello ros2 ok')

publishing #6: std_msgs.msg.String(data='hello ros2 ok')

publishing #7: std_msgs.msg.String(data='hello ros2 ok')

publishing #8: std_msgs.msg.String(data='hello ros2 ok')
```

- subscriber を起動したシェルに、下記が表示される

```
[INFO] [1729601635.898314739] [simple_subscriber]: I heard: hello ros2 ok
[INFO] [1729601636.900066588] [simple_subscriber]: I heard: hello ros2 ok
[INFO] [1729601637.900191687] [simple_subscriber]: I heard: hello ros2 ok
[INFO] [1729601638.900731698] [simple_subscriber]: I heard: hello ros2 ok
[INFO] [1729601639.901037281] [simple_subscriber]: I heard: hello ros2 ok
[INFO] [1729601640.900626694] [simple_subscriber]: I heard: hello ros2 ok
[INFO] [1729601641.900152807] [simple_subscriber]: I heard: hello ros2 ok
[INFO] [1729601642.900829767] [simple_subscriber]: I heard: hello ros2 ok
```

#### 11-4. C++ と Python 間での通信
- C++ : publisher

```bash
ros2 run bumperbot_cpp_examples simple_publisher 
```
```
[INFO] [1729602133.010207947] [simple_publisher]: publishing at 1 Hz
^C[INFO] [1729602140.648409339] [rclcpp]: signal_handler(signum=2)
```

- Python : subscriber
```
[INFO] [1729602134.013373053] [simple_subscriber]: I heard: Hello ROS2 - counter: 0
[INFO] [1729602135.012704076] [simple_subscriber]: I heard: Hello ROS2 - counter: 1
[INFO] [1729602136.012365336] [simple_subscriber]: I heard: Hello ROS2 - counter: 2
[INFO] [1729602137.012510332] [simple_subscriber]: I heard: Hello ROS2 - counter: 3
[INFO] [1729602138.012412798] [simple_subscriber]: I heard: Hello ROS2 - counter: 4
[INFO] [1729602139.011368979] [simple_subscriber]: I heard: Hello ROS2 - counter: 5
[INFO] [1729602140.012321516] [simple_subscriber]: I heard: Hello ROS2 - counter: 6
```


## 12. サンプルのPublisherの作成 (C++)

### 12-1. C++コードの作成
- [simple_subscriber.cpp](./src/bumperbot_cpp_examples/src/simple_subscriber.cpp) を新規に作成する.
- 詳細はファイルを参照.

```bash
cd ../src/bumperbot_cpp_examples/src/
touch simple_subscriber.cpp
```

### 12-2. CMakeLists.txt の修正
- 下記を追加

```cmake
#----------------------------------------------#
# added by koba
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#
add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher  rclcpp  std_msgs)
#
add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber  rclcpp  std_msgs)
#
install(TARGETS  simple_publisher  simple_subscriber
        DESTINATION lib/${PROJECT_NAME}
)
#----------------------------------------------#
```

### 12-3. package.xml の修正
- 下記を追加. すでに追加済み.

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

### 13. サンプルの動作確認 (C++)

#### 13-1. simple_subscriber ノードを実行
```bash
# 新しいターミナルに移動

# ROS 2ワークスペース (bumperbot_ws) に移動する
cd bumperbot_ws  

# colconを使ってワークスペース内の全てのパッケージをビルドする
colcon build  

# ビルド後に生成された環境設定ファイルをソースし、ROS 2環境をセットアップする
source ./install/setup.bash  

# bumperbot_py_examplesパッケージ内のsimple_publisherノードを実行する
ros2 run bumperbot_cpp_examples simple_subscriber  
```

#### 13-2. トピックの確認

```bash
# 新しいターミナルに移動

# 現在アクティブな全てのトピックのリストを表示する
ros2 topic list  

# /chatter トピックのメッセージをコンソールに表示する
ros2 topic echo /chatter  

# /chatter トピックに関する詳細な情報を表示し、パブリッシャーとサブスクライバーの詳細を含む
ros2 topic info /chatter --verbose  
```

- publisher が 0, subscriber が 1.

```
Type: std_msgs/msg/String

Publisher count: 0

Subscription count: 1

Node name: simple_subscriber
Node namespace: /
Topic type: std_msgs/msg/String
Topic type hash: RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18
Endpoint type: SUBSCRIPTION
GID: 01.0f.eb.7d.9b.e1.77.e9.00.00.00.00.00.00.14.04
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite
```

#### 13-3. C++ 間での動作確認

- publisher からメッセージを送る

```bash
# /chatter トピックにメッセージをパブリッシュするためのコマンド
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello Ros2 CPP'"
```

- subscriber を起動したシェルに、下記が表示される

```
[INFO] [1729605875.770140580] [simple_subscriber]: I heard:Hello Ros2 CPP
[INFO] [1729605876.771767167] [simple_subscriber]: I heard:Hello Ros2 CPP
[INFO] [1729605877.771222510] [simple_subscriber]: I heard:Hello Ros2 CPP
[INFO] [1729605878.771311004] [simple_subscriber]: I heard:Hello Ros2 CPP
[INFO] [1729605879.771305433] [simple_subscriber]: I heard:Hello Ros2 CPP
```
