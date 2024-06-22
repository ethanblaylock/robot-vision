from typing import List, Dict

from ultralytics import YOLO
import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

from cv_bridge import CvBridge
import cv2 
import PIL

from torch import cuda
from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks
from ultralytics.engine.results import Keypoints

from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class Yolov8Node(LifecycleNode):

    def __init__(self) -> None:
        super().__init__("yolov8_node")

        # params
        self.declare_parameter("model", "yolov8m.pt")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("enable", True)
        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().info("Yolov8 Node created")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring {self.get_name()}")

        self.model = self.get_parameter(
            "model").get_parameter_value().string_value

        self.device = self.get_parameter(
            "device").get_parameter_value().string_value

        self.threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value

        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value

        self.reliability = self.get_parameter(
            "image_reliability").get_parameter_value().integer_value

        self.image_qos_profile = QoSProfile(
            reliability=self.reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.publisher = self.create_lifecycle_publisher(
            CompressedImage, "output_image", 10)
        
        self.cv_bridge = CvBridge()
        self.get_logger().info("done configureing")
        return TransitionCallbackReturn.SUCCESS

    def enable_cb(self, request, response):
        self.enable = request.data
        response.success = True
        return response

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating {self.get_name()}")

        self.yolo = YOLO(self.model)

        if "v10" not in self.model:
            self.yolo.fuse()
        self.get_logger().info("creating sub")
        # subs
        self.subscription = self.create_subscription(
            CompressedImage,
            "camera/image_raw/compressed",
            self.image_cb,
            10
            
        )
        self.get_logger().info("created sub")
        super().on_activate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating {self.get_name()}")

        del self.yolo
        if "cuda" in self.device:
            self.get_logger().info("Clearing CUDA cache")
            cuda.empty_cache()

        self.destroy_subscription(self.subscription)
        self.subscription = None

        super().on_deactivate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up {self.get_name()}")

        self.destroy_publisher(self.publisher)

        del self.image_qos_profile

        return TransitionCallbackReturn.SUCCESS


    def image_cb(self, msg: CompressedImage) -> None:
        self.get_logger().info("recieve image")
        if self.enable:

            # convert image + predict
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
            results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=self.threshold
            )
            for result in results:
                
                image_thing = result.plot()
                image_publish = self.cv_bridge.cv2_to_compressed_imgmsg(image_thing)
                # publish detections
                ms = String()
                ms.data = "I published"
                self.publisher.publish(image_publish)

            del results
            del cv_image

def main():
    rclpy.init()
    node = Yolov8Node()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()