# Grasp Planner 最終課題 提出物

## 概要
grasp_planner ノードの動作確認と修正を行った成果物です。
主な修正点:
- src/grasp_planner/grasp_planner/node.py に tf2 import を追加
- publish_from_yaml.py に header.frame_id を強制設定
- static TF を用いて /grasp_point の publish を確認

## 動作環境
- Ubuntu 22.04 (Jammy)
- ROS 2 Humble

## 起動手順
各ターミナルで必ず実行:
\`\`\`bash
cd /workspace
source install/setup.bash
\`\`\`

T2: static TF
\`\`\`bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_link
\`\`\`

T1: YAML publisher
\`\`\`bash
python3 /workspace/publish_from_yaml.py
\`\`\`

T3: grasp_planner
\`\`\`bash
ros2 run grasp_planner grasp_planner_node --ros-args --log-level info
\`\`\`

## 変更点
- node.py に以下を追加:
\`\`\`py
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
\`\`\`

## 検証コマンド
\`\`\`bash
ros2 topic echo /detected_objects --once
ros2 topic echo /tf --once
ros2 topic echo /grasp_point --once
\`\`\`
