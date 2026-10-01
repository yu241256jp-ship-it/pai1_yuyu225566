import rclpy
from rclpy.node import Node
from my_msgs.msg import DetectedObject, DetectedObjectArray
from std_msgs.msg import Header

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_detected_publisher')
        self.pub = self.create_publisher(DetectedObjectArray, '/detected_objects', 1)
        self.timer = self.create_timer(1.0, self.timer_cb)

    def timer_cb(self):
        obj = DetectedObject()
        # header
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = 'camera_frame'
        try:
            obj.header = h
        except Exception:
            pass

        # 基本フィールド（my_msgs/msg/DetectedObject 定義に合わせています）
        try:
            obj.id = 1
        except Exception:
            pass
        if hasattr(obj, 'label'):
            obj.label = 'cup'
        if hasattr(obj, 'confidence'):
            obj.confidence = 0.95
        if hasattr(obj, 'x1'):
            obj.x1 = 100.0
            obj.y1 = 120.0
            obj.x2 = 200.0
            obj.y2 = 240.0
        if hasattr(obj, 'center_x'):
            obj.center_x = 150.0
            obj.center_y = 180.0
        if hasattr(obj, 'width_px'):
            obj.width_px = 100.0
            obj.height_px = 120.0

        arr = DetectedObjectArray()
        if hasattr(arr, 'objects'):
            arr.objects = [obj]
        else:
            for field in arr.__slots__:
                try:
                    setattr(arr, field, [obj])
                    break
                except Exception:
                    continue

        self.pub.publish(arr)
        self.get_logger().info('published DetectedObjectArray')

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
