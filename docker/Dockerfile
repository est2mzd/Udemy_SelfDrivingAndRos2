# ベースイメージとしてUbuntu 22.04を使用
FROM ubuntu:22.04

# 対話を避ける設定
ENV DEBIAN_FRONTEND=noninteractive

# 必要なパッケージのインストール
RUN apt-get update &&  \
    apt-get install -y \
        locales \
        curl \
        gnupg2 \
        lsb-release \
        build-essential \
        software-properties-common && \
    add-apt-repository universe

# タイムゾーンが自動的に「Asia/Tokyo」に設定
RUN apt-get update &&  \
    apt-get install -y tzdata && \
    ln -fs /usr/share/zoneinfo/Asia/Tokyo /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata

# 日本語ロケールの設定
# このコマンドは、日本語（ja_JP）のUTF-8エンコーディングを使ったロケールを生成します。
RUN locale-gen ja_JP.UTF-8 && \
    update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF-8

# ロケールの確認
RUN locale

# ROS2のキーを追加
#RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# ソフトウェアリポジトリの設定
#RUN echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2-latest.list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

#===========================================================#
# ホストのユーザー名とIDを設定するための環境変数
ARG USERNAME
ARG USERID

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
RUN sudo apt update && \
    sudo apt upgrade && \
    sudo apt install -y \
        ros-humble-desktop \
        ros-dev-tools


# ROS2の環境セットアップ
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

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

#===========================================================#
# 追加のインストール
RUN sudo apt-get install tree && \
    sudo apt-get install -y ros-humble-urdf-tutorial

#===========================================================#    
# デフォルトのコマンド
CMD ["/bin/bash"]




