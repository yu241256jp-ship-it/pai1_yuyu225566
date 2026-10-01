#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import os, time

class GraspViz(Node):
    def __init__(self):
        super().__init__('grasp_viz')
        self.outdir = '/workspace/frames'
        os.makedirs(self.outdir, exist_ok=True)
        self.count = 0
        self.grasp = None
        self.create_subscription(Point, '/grasp_point', self.cb_grasp, 10)
        self.create_timer(0.2, self.save_frame)  # 5 fps

    def cb_grasp(self, msg):
        self.grasp = (msg.x, msg.y, msg.z)

    def save_frame(self):
        fig, ax = plt.subplots(figsize=(6.4,4.8))
        ax.set_title('Grasp point (red X)')
        ax.set_xlim(-1.0, 1.0)
        ax.set_ylim(-1.0, 1.0)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        if self.grasp:
            gx, gy, gz = self.grasp
            ax.scatter([gx], [gy], c='red', s=120, marker='x', label=f'grasp z={gz:.2f}')
            ax.legend(loc='upper right')
        ax.grid(True, linestyle='--', alpha=0.3)
        fname = os.path.join(self.outdir, f'frame_{self.count:06d}.png')
        fig.savefig(fname, dpi=100)
        plt.close(fig)
        self.count += 1
        # 自動停止（任意）
        if self.count >= 300:
            self.get_logger().info('Reached 300 frames, shutting down.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GraspViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
