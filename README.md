# livox_bringup for MID-360

　livox MID-360 の起動、セットアップをサポートするパッケージです。

## 0. 環境準備

1. **ワークスペースを作成**<br>
    ```bash
    mkdir -p ~/colcon_ws/src
    cd ~/colcon_ws/src
    ```

2. **このパッケージをクローンする**<br>
    ```bash
    git clone https://github.com/GAI-313/livox_bringup.git
    cd livox_bringup
    ```

3. **依存関係をワークスペースに追加する**<br>
    　`src` ディレクトリに以下の依存パッケージをクローンします。
    - [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
    - [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
    ```bash
    vcs import .. < ./depend.respo
    ```

    > **`vcs` コマンドが使えない場合** 、以下のコマンドを実行して `vsc` をインストールします。
    > ```bash
    > sudo apt install python3-vcstool
    > ```

4. **livox_ros_driver2 を編集する**<br>
    ```bash
    cd ../livox_ros_driver2
    mv package_ROS2.xml package.xml
    ```

<a id="1"></a>
## 1. ビルド

1. **Livox-SDK2 をビルドする**<br>
    ```bash
    cd ../Livox-SDK2
    mkdir build && cd build
    cmake .. && make -j
    sudo make install
    ```

2. **rosdep**<br>
    以下のコマンドを実行して依存関係を解決します。
    ```bash
    cd ~/colcon_ws
    rosdep install -i -y --from-path src
    ```
    > もしはじめて `rosdep` を使用する場合、上記コマンドを実行前に以下のコマンドを実行してください。
    > ```bash
    > sudo rosdep init && rosdep update
    > ```

3. **ワークスペースをビルドする**<br>
    ```bash
    cd ~/colcon_ws
    colcon build --symlink-install --cmake-args -DROS_EDITION="ROS2" -DHUMBLE_ROS=$ROS_DISTRO
    ```

## 2. Livox MID-360 のセットアップ

1. **MID-360 の接続と接続確認**<br>
    - コンピューターと MID-360 を Ethernet で接続します。
    - MID-360 用のネットワーク設定を行い、MID-360 と通信できているか確認します。
        ```bash
        ping <MID-360 IP Address>
        ```

2. **MID-360.json の編集**<br>
    - [MID-360.json](/config/MID-360.json) のアドレス `192.168.1.23` と `192.168.1.3` をそれぞれコンピューターの IP アドレスと MID-360 の IP アドレスに書き換えてください。
    - もし、[1. ビルド](#1) で `--symlink-install` をつけずに `colcon build` を行った場合、もう一度ワークスペースをビルドしてください。

## 3. 実行する
　source を通して、livox_bringup を起動します。
```bash
source ~/colcon_ws/install/setup.bash
ros2 launch livox_bringup bringup_launch.py
```
　以下の図のように Rviz が起動し、MID-360 からの PointCloud が表示されていれば成功です。

<img width=10% /><img src="/imgs/MID-360.rviz.png" width=80% />

## CycloneDDS に切り替える
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```


## Launch Argument
```bash
ros2 launch livox_bringup bringup_launch.py --show-args
```
