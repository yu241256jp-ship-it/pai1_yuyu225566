#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math, os, time
import matplotlib.pyplot as plt
import numpy as np

# --- 設定: 平面アームのリンク長（URDFが無ければここを編集） ---
DEFAULT_LINK_LENGTHS = [0.3, 0.25, 0.15]

# --- シンプル MLP 推論（固定重み）: 特徴 -> スコア ---
# 入力特徴: [x, y, area(normalized)] を想定（area が無ければ 1.0）
# 重みは小さな固定値（擬似的に「学習済み」を模す）
W1 = np.array([[1.2, -0.5, 0.8],
               [0.3,  0.9, -0.2],
               [-0.4, 0.2, 0.5]])
b1 = np.array([0.1, -0.1, 0.0])
W2 = np.array([0.6, -0.3, 0.9])
b2 = 0.05

def mlp_score(x):
    # x: numpy array shape (3,)
    h = np.tanh(W1.dot(x) + b1)
    s = W2.dot(h) + b2
    return float(s)

# 前進運動学（平面）
def forward_kinematics_2d(joint_angles, link_lengths):
    x = 0.0; y = 0.0; theta = 0.0
    points = [(x,y)]
    for i, angle in enumerate(joint_angles):
        theta += angle
        lx = link_lengths[i] * math.cos(theta)
        ly = link_lengths[i] * math.sin(theta)
        x += lx; y += ly
        points.append((x,y))
    return points

class RobotVizAI(Node):
    def __init__(self):
        super().__init__('robot_viz_ai')
        self.outdir = '/workspace/frames'
        os.makedirs(self.outdir, exist_ok=True)
        self.count = 0
        self.joint_names = []
        self.joint_positions = []
        self.link_lengths = DEFAULT_LINK_LENGTHS.copy()
        self.grasp = None
        self.detected = []  # list of dicts: {'x':..,'y':..,'area':..,'label':..}
        self.start_time = time.time()
        # subscriptions
        self.create_subscription(JointState, '/joint_states', self.cb_joint, 10)
        # detected_objects: try to subscribe generically; if type differs, callback may not be called
        try:
            from my_msgs.msg import DetectedObjectArray
            self.create_subscription(DetectedObjectArray, '/detected_objects', self.cb_detected, 10)
        except Exception:
            # fallback: no detected_objects type available
            pass
        # grasp point
        try:
            self.create_subscription(Point, '/grasp_point', self.cb_grasp, 10)
        except Exception:
            pass
        # timer: 5 fps
        self.create_timer(0.2, self.save_frame)

    def cb_joint(self, msg: JointState):
        if not self.joint_names:
            self.joint_names = list(msg.name)
        name_to_pos = {n:p for n,p in zip(msg.name, msg.position)}
        # keep order of joint_names if available, else use msg.name
        if set(self.joint_names) != set(msg.name):
            self.joint_names = list(msg.name)
        self.joint_positions = [float(name_to_pos.get(n, 0.0)) for n in self.joint_names]
        if len(self.link_lengths) < len(self.joint_positions):
            self.link_lengths += [0.1] * (len(self.joint_positions) - len(self.link_lengths))

    def cb_detected(self, msg):
        # try to extract objects robustly
        objs = []
        # common field names: objects or detected_objects
        seq = getattr(msg, 'objects', None) or getattr(msg, 'detected_objects', None) or []
        for o in seq:
            # try common attributes
            x = None; y = None; area = None; label = ''
            # pose.position
            pose = getattr(o, 'pose', None)
            if pose is not None:
                pos = getattr(pose, 'position', None)
                if pos is not None:
                    x = getattr(pos, 'x', None)
                    y = getattr(pos, 'y', None)
            # direct x,y
            if x is None:
                x = getattr(o, 'x', None)
            if y is None:
                y = getattr(o, 'y', None)
            # area or size
            area = getattr(o, 'area', None) or getattr(o, 'size', None) or getattr(o, 'bbox_area', None)
            label = getattr(o, 'label', '') or getattr(o, 'name', '')
            objs.append({'x': float(x or 0.0), 'y': float(y or 0.0), 'area': float(area or 1.0), 'label': str(label)})
        self.detected = objs

    def cb_grasp(self, msg: Point):
        self.grasp = (msg.x, msg.y, msg.z)

    def save_frame(self):
        fig, ax = plt.subplots(figsize=(6.4,4.8))
        ax.set_title('Robot model (AI-scored detected objects)')
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
        ax.grid(True, linestyle='--', alpha=0.3)
        ax.plot([0],[0], marker='o', color='k')  # base

        # draw robot if joint data exists
        if self.joint_positions:
            angles = self.joint_positions
            Ls = self.link_lengths[:len(angles)]
            pts = forward_kinematics_2d(angles, Ls)
            xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
            ax.plot(xs, ys, '-o', color='C1', linewidth=3, markersize=6)
            ax.text(xs[-1], ys[-1], 'EE', color='red')
        else:
            ax.text(0.1, 0.9, 'Waiting for /joint_states...', transform=ax.transAxes)

        # AI: score detected objects and pick best
        best = None; best_score = -1e9
        for d in self.detected:
            feat = np.array([d['x'], d['y'], d['area']], dtype=float)
            # normalize area roughly
            feat[2] = feat[2] / (feat[2] + 1.0)
            score = mlp_score(feat)
            # draw object
            ax.scatter([d['x']], [d['y']], c='C0', s=80, alpha=0.8)
            ax.text(d['x'], d['y'], d['label'] or f'{score:.2f}', fontsize=8)
            if score > best_score:
                best_score = score; best = (d, score)
        # draw best as green circle and set grasp if none provided
        if best:
            d, s = best
            ax.scatter([d['x']], [d['y']], c='green', s=140, marker='o', edgecolors='k', linewidths=1.2, label=f'best {s:.2f}')
            # if no /grasp_point published, set grasp to best
            if not self.grasp:
                self.grasp = (d['x'], d['y'], 0.0)

        # draw grasp point if available
        if self.grasp:
            gx, gy, gz = self.grasp
            ax.scatter([gx], [gy], c='red', s=120, marker='x', label='grasp')
            ax.legend(loc='upper right')

        # timestamp
        t = time.time() - self.start_time
        ax.text(0.02, 0.95, f't={t:.1f}s', transform=ax.transAxes)

        ax.relim(); ax.autoscale_view(); ax.set_aspect('equal', adjustable='datalim')
        fname = os.path.join(self.outdir, f'frame_{self.count:06d}.png')
        fig.savefig(fname, dpi=100, bbox_inches='tight')
        plt.close(fig)
        self.count += 1
        if self.count >= 300:
            self.get_logger().info('Reached 300 frames, shutting down.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RobotVizAI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
