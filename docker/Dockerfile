# ベースイメージとしてUbuntu 24.04を使用
FROM ubuntu:24.04

# 環境変数の設定
ENV DEBIAN_FRONTEND=noninteractive

# 必要なパッケージのインストール
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    software-properties-common

# 日本語ロケールの設定
# このコマンドは、日本語（ja_JP）のUTF-8エンコーディングを使ったロケールを生成します。
RUN locale-gen ja_JP.UTF-8 && \
    update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF-8

# ロケールの確認
RUN locale

# ROS2のキーを追加
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# ソフトウェアリポジトリの設定
RUN echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2-latest.list

#===========================================================#
# ホストのユーザー名とIDを設定するための環境変数
ARG USERNAME
ARG USERID

# ubuntu23からデフォルトで追加された ubuntuという名前のユーザーが、uid=1000をもち
# ホストのuidと競合するため、削除する
RUN userdel -r ubuntu

# ユーザーとsudoを設定
RUN apt-get update && \
    apt-get install -y sudo && \
    useradd -m -u $USERID -s /bin/bash $USERNAME && \
    passwd -d $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# コンテナを起動した際にシェルに入るようにエントリーポイントを設定
# これ以降は、$USERNAME のアカウントで RUN が実行される
USER $USERNAME
WORKDIR /home/$USERNAME/
#ENTRYPOINT ["/bin/bash"]
#===========================================================#

# ROS2のインストール
RUN sudo apt-get update && \
    sudo apt-get install -y \
        ros-jazzy-desktop \
        ros-jazzy-ros-base \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-xacro \
        ros-jazzy-ros-gz-* \
        ros-jazzy-*-ros2-control \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-turtlesim \
        ros-jazzy-robot-localization \
        ros-jazzy-joy \
        ros-jazzy-joy-teleop \
        ros-jazzy-tf-transformations

# ROS2の環境セットアップ
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# ワークスペースの準備
RUN sudo mkdir -p /ros2_ws/src

# 必要な依存パッケージのインストール
RUN sudo apt-get install -y \
        python3-rosdep \
        python3-colcon-common-extensions \
        python3-pip \
        python3-transforms3d

# rosdepの初期化と依存関係の解決
RUN sudo rosdep init && \
    sudo rosdep update

# デフォルトのコマンド
CMD ["/bin/bash"]




