import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os

from ament_index_python.packages import get_package_share_directory

class PalletGroundNode(Node):
    def __init__(self):
        super().__init__('pallet_ground_node')

        package_share_dir = get_package_share_directory('pallet_ground_package')
        models_dir = os.path.join(package_share_dir, 'models')
        self.ground_model = YOLO(os.path.join(models_dir, 'ground_seg.pt'))
        self.pallet_model = YOLO(os.path.join(models_dir, 'pallet_det.pt'))


        self.bridge = CvBridge()

        # === Subscribe to camera feed ===
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("🟢 pallet_ground_node started (ground + pallet detection)")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # === Ground Segmentation ===
        ground_result = self.ground_model(cv_image, verbose=False)[0]
        if ground_result.masks:
            for mask_tensor in ground_result.masks.data:
                mask = mask_tensor.cpu().numpy().astype("uint8") * 255
                contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2)

        # === Pallet Detection ===
        pallet_result = self.pallet_model(cv_image, verbose=False)[0]
        if pallet_result.boxes:
            for box in pallet_result.boxes.xyxy.cpu().numpy():
                x1, y1, x2, y2 = map(int, box)
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(cv_image, "Pallet", (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # === Display the result ===
        cv2.imshow("YOLOv8 Inference (Pallet + Ground)", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PalletGroundNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()