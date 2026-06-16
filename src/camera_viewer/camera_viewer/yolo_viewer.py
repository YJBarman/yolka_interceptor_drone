import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import time
from ultralytics import YOLO

from geometry_msgs.msg import Point

class YOLOViewer(Node):

    def __init__(self):
        super().__init__('yolo_viewer')

        # Converts ROS Image messages to OpenCV images
        self.bridge = CvBridge()
        self.target_locked = False

        self.lock_x = 0
        self.lock_y = 0

        self.max_jump = 120

        # Load YOLO model
        self.model = YOLO("/home/joy/yolo_test/yolov8n.pt")

        # Use GTX 1650 GPU
        self.model.to("cuda")
        
        self.error_pub = self.create_publisher(
            Point,
            '/tracking/error',
            10
        )

        # Subscribe to Gazebo drone camera topic
        self.subscription = self.create_subscription(
            Image,
            '/world/kthspacelab/model/x500_mono_cam_0/link/camera_link/sensor/camera/image',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        start = time.time()
        # Convert ROS image -> OpenCV image
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        results = self.model(
            frame,
            verbose=False
        )
        annotated = results[0].plot()
        height, width, _ = frame.shape

        center_x = width // 2
        center_y = height // 2

        best_box = None
        best_area = 0
        best_distance = 999999

        for box in results[0].boxes:

            x1, y1, x2, y2 = map(int, box.xyxy[0])

            area = (x2 - x1) * (y2 - y1)

            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            if area < 400:
                continue

            if not self.target_locked:

                self.target_locked = True

                self.lock_x = cx
                self.lock_y = cy

                best_box = (x1, y1, x2, y2)
                best_area = area

                print("TARGET LOCKED")

                break

            distance = (
                (cx - self.lock_x) ** 2 +
                (cy - self.lock_y) ** 2
            )

            if distance < best_distance:

                best_distance = distance

                best_box = (x1, y1, x2, y2)
                best_area = area

        if best_box is not None:

            x1, y1, x2, y2 = best_box

            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            self.lock_x = cx
            self.lock_y = cy

            error_x = cx - center_x
            error_y = cy - center_y

            msg = Point()

            msg.x = float(error_x)
            msg.y = float(error_y)
            msg.z = float(best_area)

            self.error_pub.publish(msg)

            cv2.rectangle(
                annotated,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                3
            )

            cv2.circle(
                annotated,
                (cx, cy),
                5,
                (0, 0, 255),
                -1
            )
        else:

            self.target_locked = False

            msg = Point()
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 0.0

            self.error_pub.publish(msg)


        annotated = results[0].plot()

        cv2.imshow("YOLO Camera", annotated)
        cv2.waitKey(1)
        elapsed = time.time() - start

        print(f"FPS = {1.0/elapsed:.1f}")


def main(args=None):

    rclpy.init(args=args)

    node = YOLOViewer()

    rclpy.spin(node)

    cv2.destroyAllWindows()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()