#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
import os, time, threading

# メッセージ型をインポート（パッケージ名が my_msgs の想定）
try:
    from my_msgs.msg import DetectedObjectArray
    from geometry_msgs.msg import Point
except Exception as e:
    print("Import error:", e)
    print("Ensure my_msgs and geometry_msgs are available in the environment.")
    raise

class VizNode(Node):
    def __init__(self):
        super().__init__('viz_node')
        self.outdir = '/workspace/frames'
        os.makedirs(self.outdir, exist_ok=True)
        self.count = 0
        self.detected = None
        self.grasp = None
        self.create_subscription(DetectedObjectArray, '/detected_objects', self.cb_detected, 10)
        # grasp point topic (geometry_msgs/Point) がある想定
        self.create_subscription(Point, '/grasp_point', self.cb_grasp, 10)
        # 定期的にフレームを保存
        self.timer = self.create_timer(0.2, self.save_frame)  # 5 fps

    def cb_detected(self, msg):
        # msg の構造に合わせて必要なフィールドを抽出する
        # ここでは各オブジェクトに .x, .y, .z, .label がある想定
        objs = []
        for o in getattr(msg, 'objects', []) if hasattr(msg, 'objects') else getattr(msg, 'detected_objects', []):
            # try common fields
            x = getattr(o, 'x', None) or getattr(o, 'pose', None) and getattr(o.pose, 'position', None) and getattr(o.pose.position, 'x', 0)
            y = getattr(o, 'y', None) or getattr(o, 'pose', None) and getattr(o.pose, 'position', None) and getattr(o.pose.position, 'y', 0)
            label = getattr(o, 'label', '') if hasattr(o, 'label') else getattr(o, 'name', '')
            objs.append((float(x or 0), float(y or 0), str(label)))
        self.detected = objs

    def cb_grasp(self, msg):
        # geometry_msgs/Point
        self.grasp = (msg.x, msg.y, msg.z)

    def save_frame(self):
        fig, ax = plt.subplots(figsize=(6.4,4.8))
        ax.set_title('Detected objects and grasp points')
        ax.set_xlim(-1.0, 1.0)
        ax.set_ylim(-1.0, 1.0)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        # draw detected objects
        if self.detected:
            xs = [o[0] for o in self.detected]
            ys = [o[1] for o in self.detected]
            labels = [o[2] for o in self.detected]
            ax.scatter(xs, ys, c='C0', s=80, label='detected')
            for xi, yi, lab in zip(xs, ys, labels):
                ax.text(xi, yi, lab, fontsize=8, color='C0')
        # draw grasp point
        if self.grasp:
            gx, gy, gz = self.grasp
            ax.scatter([gx], [gy], c='red', s=120, marker='x', label='grasp')
        ax.legend(loc='upper right')
        ax.grid(True, linestyle='--', alpha=0.3)
        fname = os.path.join(self.outdir, f'frame_{self.count:06d}.png')
        fig.savefig(fname, dpi=100)
        plt.close(fig)
        self.count += 1
        # stop after enough frames to avoid infinite run if desired
        if self.count >= 300:  # 300 frames ~ 60s at 5fps
            self.get_logger().info('Reached 300 frames, shutting down.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = VizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
