import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
from geometry_msgs.msg import Point


class CameraViewer(Node):

    def __init__(self):
        super().__init__('camera_viewer')

        self.bridge = CvBridge()
        self.error_pub = self.create_publisher(
            Point,
            '/tracking/error',
            10
        )

        self.subscription = self.create_subscription(
            Image,
            '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image',
            self.image_callback,
            10
        )

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        height, width, _ = frame.shape
        center_x = width // 2
        center_y = height // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = (0, 120, 70)
        upper_red1 = (10, 255, 255)

        lower_red2 = (170, 120, 70)
        upper_red2 = (180, 255, 255)

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = mask1 + mask2

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # Draw image center (red dot)
        cv2.circle(
            frame,
            (center_x, center_y),
            5,
            (0, 0, 255),
            -1
        )

        command = "NO TARGET"

        if contours:

            largest = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest) > 100:

                x, y, w, h = cv2.boundingRect(largest)
                area = w * h

                cv2.rectangle(
                    frame,
                    (x, y),
                    (x + w, y + h),
                    (0, 255, 0),
                    2
                )

                cx = x + w // 2
                cy = y + h // 2

                error_x = cx - center_x
                error_y = cy - center_y
                
                msg = Point()

                msg.x = float(error_x)
                msg.y = float(error_y)
                msg.z = float(area)

                self.error_pub.publish(msg)

                # Draw target center (blue dot)
                cv2.circle(
                    frame,
                    (cx, cy),
                    5,
                    (255, 0, 0),
                    -1
                )

                # Movement decision
                if error_x < -30:
                    command = "MOVE LEFT"

                elif error_x > 30:
                    command = "MOVE RIGHT"

                else:
                    command = "CENTERED"

                cv2.putText(
                    frame,
                    f"Target: ({cx},{cy})",
                    (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f"EX={error_x} EY={error_y}",
                    (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 0, 255),
                    2
                )
                cv2.putText(
                    frame,
                    f"AREA={area}",
                    (10, 160),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255,255,0),
                    2
                )

                print(
                    f"Target=({cx},{cy}) "
                    f"EX={error_x} EY={error_y} "
                    f"AREA={area} "
                    f"{command}"
                )

        cv2.putText(
            frame,
            command,
            (10, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 255),
            2
        )

        cv2.imshow("Drone Camera", frame)
        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = CameraViewer()

    rclpy.spin(node)

    cv2.destroyAllWindows()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()