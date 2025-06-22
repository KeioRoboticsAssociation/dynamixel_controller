``` c_cpp_properties.json
"includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/humble/include/**",
                "${workspaceFolder}/src/NHK_2025/dynamixel_controller/include/**",
                "${workspaceFolder}/build/dynamixel_controller/rosidl_generator_cpp"
            ],
```

# dynamixel_controller

dynamixel_controller は、Dynamixel アクチュエータとの通信を行うための ROS2 ノードです。
このパッケージは DynamixelSDK を用いて、Dynamixel の各種命令（PING、READ_DATA、WRITE_DATA、SYNC_READ、SYNC_WRITE など）を送受信し、ROS2 トピック経由でデータのやり取りを実現します。
また、専用のメッセージ定義 (msg/DynamixelController.msg) により、定数を C++ と Python の両側で共有できます。

【特徴】
・Dynamixel 通信
　- 基本命令 (PING、READ_DATA、WRITE_DATA) に加え、SYNC_READ および SYNC_WRITE 命令にも対応

・ROS2 インターフェース
　- 命令は "dynamixel_tx" トピック、応答は "dynamixel_rx" トピックで送受信

・カスタムメッセージ
　- msg/DynamixelController.msg で定義した定数を、C++ と Python 両方のノードで共有可能

・サンプル Python クライアント
　- Python ノードからの命令送信例（SYNC_READ）および応答受信例を提供
・バス設定
　- `config/bus_config.yaml` で TTL と RS485 に接続されたモータ ID を指定可能

【ビルド方法】

＜前提条件＞
- ROS 2 Humble (またはそれ以降)
- DynamixelSDK（システムにインストール済み）
- colcon ビルドツール

＜手順＞
1. ROS 2 ワークスペースの src フォルダに本パッケージを配置またはクローンします。
   例)
   ``` bash
     cd ~/ros2_ws/src
     git clone <repository_url> dynamixel_controller
    ```

2. ワークスペースのルートに移動し、パッケージをビルドします。
   例)
   ``` bash
     cd ~/ros2_ws
     colcon build --packages-select dynamixel_controller
    ```

3. ビルド完了後、以下のコマンドで環境をセットアップしてください。
    ``` bash
     source install/local_setup.bash
    ```

【実行方法】

＜C++ ノードの起動＞
dynamixel_controller ノードは、実行可能ファイル "dynamixel_controller_node" として生成されます。
下記のコマンドで起動してください：
    ``` bash
     ros2 run dynamixel_controller dynamixel_controller_node
    ```
モータがTTL接続かRS485接続かを区別するために、パラメータファイルを使用してIDの割り当てを設定します。サンプルの`config/bus_config.yaml`を編集し、以下のようにノード起動時に読み込みます。

    ``` bash
    ros2 run dynamixel_controller dynamixel_controller_node --ros-args --params-file config/bus_config.yaml
    ```
＜Python クライアントの利用例＞
下記のサンプルは、Python ノードから C++ ノードに対して命令を送信する例です。
この例では、5秒ごとに SYNC_READ 命令（例：2 台の Dynamixel の PRESENT_POSITION を4バイトずつ要求）を送信し、応答を受信します。

【Python サンプルコード例】
``` Python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

class DynamixelControllerClient(Node):
    def __init__(self):
        super().__init__('dynamixel_controller_client')
        # 命令送信用トピック (dynamixel_tx)
        self.tx_publisher = self.create_publisher(UInt8MultiArray, 'dynamixel_tx', 10)
        # C++ ノードからの応答受信用トピック (dynamixel_rx)
        self.rx_subscription = self.create_subscription(
            UInt8MultiArray,
            'dynamixel_rx',
            self.rx_callback,
            10
        )
        # 5秒ごとに命令を送信するタイマー
        self.timer = self.create_timer(5.0, self.timer_callback)

    def timer_callback(self):
        # SYNC_READ 命令のフォーマット：[SYNC_READ, start_address, data_length, id1, id2, ...]
        # ここでは例として、start_address = 132 (例: PRESENT_POSITION)、data_length = 4、対象ID: 1, 2
        msg = UInt8MultiArray()
        SYNC_READ = 130  # DynamixelController.msg で定義した SYNC_READ (0x82)
        start_address = 132
        data_length = 4
        msg.data = [SYNC_READ, start_address, data_length, 1, 2]
        self.tx_publisher.publish(msg)
        self.get_logger().info(f'Published SYNC_READ command: {msg.data}')

    def rx_callback(self, msg):
        self.get_logger().info(f'Received response: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelControllerClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

＜実行方法＞
1. 環境を source した上で実行してください。
   例)
   ``` bash
     source install/local_setup.bash
     python3 <client_node_script.py>
    ```

【メッセージ定義】

DynamixelController.msg ファイルには、以下の定数が定義されています（例）。

    ``` .msg
     uint8 PING = 1
     uint8 READ_DATA = 2
     uint8 WRITE_DATA = 3
     uint8 REG_WRITE = 4
     uint8 ACTION = 5
     uint8 FACTORY_RESET = 6
     uint8 REBOOT = 8
     uint8 SYNC_READ = 130
     uint8 SYNC_WRITE = 131
     // その他の定数（各種アドレスなど）を追加
     ```

自動生成された C++ ヘッダーや Python モジュールから、以下のようにインポートして利用できます。

＜C++ 側＞
     #include "dynamixel_controller/msg/dynamixel_controller.hpp"

＜Python 側＞
     from dynamixel_controller.msg import DynamixelController

【VS Code での開発】

自動生成されたヘッダーが Intellisense に認識されない場合は、.vscode/c_cpp_properties.json の includePath に、以下のディレクトリを追加してください。
   - ${workspaceFolder}/install/dynamixel_controller/include
   - ${workspaceFolder}/build/dynamixel_controller/rosidl_generator_cpp

【注意事項】

- ハードウェア設定：
  使用する Dynamixel のファームウェア、DynamixelSDK のバージョンに合わせ、BAUDRATE、DEVICE_NAME、対象モータ ID (dxl_id_) 等を適切に設定してください。

- 環境セットアップ：
  各ノード実行前に必ず ROS2 の環境（例: /opt/ros/humble/setup.bash および install/local_setup.bash）を source してください。

【ライセンス】
このプロジェクトは LICENSE ファイルに基づいてライセンスされています。

【コンタクト】
質問やフィードバックは、your-email@example.com までご連絡ください.
