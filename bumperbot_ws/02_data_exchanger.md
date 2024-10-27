## 1.  パッケージの作成

```bash
cd bumperbot_ws/src/
ros2 pkg create --build-type ament_cmake data_exchanger
```

上記を実行すると、下記が自動的に作成される

```
data_exchanger
|--- include/
|    |---app_a.hpp (これは後で自作する)
|    |---app_b.hpp (これは後で自作する)
|
|--- src/
|    |---app_a.cpp (これは後で自作する)
|    |---app_b.cpp (これは後で自作する)
|
|--- CMakeLists.txt
|--- package.xml
```

## 2. Cpp code の作成
app_a.cpp　と　app_b.cpp　を作成する


## 3. CMakeの編集

下記を追加する

```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include) # includeフォルダをインクルードパスに追加

add_executable(app_a src/app_a.cpp)
ament_target_dependencies(app_a rclcpp std_msgs)

add_executable(app_b src/app_b.cpp)
ament_target_dependencies(app_b rclcpp std_msgs)

install(TARGETS
  app_a
  app_b
  DESTINATION lib/${PROJECT_NAME}
)
```

## 4. package.xmlの編集

下記を追加する

```xml
<build_depend>rclcpp</build_depend>
<exec_depend>rclcpp</exec_depend>

<build_depend>std_msgs</build_depend>
<exec_depend>std_msgs</exec_depend>
```

## 5. 依存関係を解決します。rosdepを使って必要な依存パッケージをインストールします。

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 6. ビルドする

```bash
colcon build --packages-select data_exchanger
```

## 7. 実行する

### 7-1. ROS2の環境をセットアップ：

```
source install/setup.bash
```

### 7-2. appAを実行

```
ros2 run data_exchanger app_a
```

### 7-3. appBを実行

```
ros2 run data_exchanger app_b
```
