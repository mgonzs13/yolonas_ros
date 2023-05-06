
import cv2
import random

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from cv_bridge import CvBridge

from super_gradients.training import models
from super_gradients.common.object_names import Models

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Detection2DArray
from std_srvs.srv import SetBool


class YoloNasNode(Node):

    def __init__(self) -> None:
        super().__init__("yolonas_node")

        # params
        self.declare_parameter("model", Models.YOLO_NAS_S)
        model = self.get_parameter(
            "model").get_parameter_value().string_value

        self.declare_parameter("pretrained_weights", "coco")
        pretrained_weights = self.get_parameter(
            "pretrained_weights").get_parameter_value().string_value

        self.declare_parameter("num_classes", -1)
        num_classes = self.get_parameter(
            "num_classes").get_parameter_value().integer_value

        self.declare_parameter("checkpoint_path", "")
        checkpoint_path = self.get_parameter(
            "checkpoint_path").get_parameter_value().string_value

        self.declare_parameter("device", "cuda:0")
        device = self.get_parameter(
            "device").get_parameter_value().string_value

        self.declare_parameter("threshold", 0.5)
        self.threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value

        self.declare_parameter("enable", True)
        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value

        if num_classes < 0:
            num_classes = None

        if len(checkpoint_path) == 0:
            checkpoint_path = None

        self._class_to_color = {}
        self.cv_bridge = CvBridge()
        self.yolo = models.get(
            model,
            pretrained_weights=pretrained_weights,
            num_classes=num_classes,
            checkpoint_path=checkpoint_path
        )
        self.yolo.to(device)

        # topcis
        self._pub = self.create_publisher(Detection2DArray, "detections", 10)
        self._dbg_pub = self.create_publisher(Image, "dbg_image", 10)
        self._sub = self.create_subscription(
            Image, "image_raw", self.image_cb,
            qos_profile_sensor_data
        )

        # services
        self._srv = self.create_service(SetBool, "enable", self.enable_cb)

    def enable_cb(self,
                  req: SetBool.Request,
                  res: SetBool.Response
                  ) -> SetBool.Response:
        self.enable = req.data
        res.success = True
        return res

    def image_cb(self, msg: Image) -> None:

        if self.enable:

            # convert image + predict
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
            results = list(self.yolo.predict(
                cv_image)._images_prediction_lst)[0]

            # create detections msg
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header

            for pred_i in range(len(results.prediction)):
                class_id = int(results.prediction.labels[pred_i])
                label = str(results.class_names[class_id])
                score = float(results.prediction.confidence[pred_i])

                if score < self.threshold:
                    continue

                x1 = int(results.prediction.bboxes_xyxy[pred_i, 0])
                y1 = int(results.prediction.bboxes_xyxy[pred_i, 1])
                x2 = int(results.prediction.bboxes_xyxy[pred_i, 2])
                y2 = int(results.prediction.bboxes_xyxy[pred_i, 3])

                # get boxes values
                x_s = float(x2 - x1)
                y_s = float(y2 - y1)
                x_c = x1 + x_s / 2
                y_c = y1 + y_s / 2

                detection = Detection2D()

                detection.bbox.center.position.x = x_c
                detection.bbox.center.position.y = y_c
                detection.bbox.size_x = x_s
                detection.bbox.size_y = y_s

                # get hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = label
                hypothesis.hypothesis.score = score
                detection.results.append(hypothesis)

                # draw boxes for debug
                if label not in self._class_to_color:
                    r = random.randint(0, 255)
                    g = random.randint(0, 255)
                    b = random.randint(0, 255)
                    self._class_to_color[label] = (r, g, b)
                color = self._class_to_color[label]

                min_pt = (round(detection.bbox.center.position.x - detection.bbox.size_x / 2.0),
                          round(detection.bbox.center.position.y - detection.bbox.size_y / 2.0))
                max_pt = (round(detection.bbox.center.position.x + detection.bbox.size_x / 2.0),
                          round(detection.bbox.center.position.y + detection.bbox.size_y / 2.0))
                cv2.rectangle(cv_image, min_pt, max_pt, color, 2)

                label = "{} ({:.3f})".format(label, score)
                pos = (min_pt[0] + 5, min_pt[1] + 25)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(cv_image, label, pos, font,
                            1, color, 1, cv2.LINE_AA)

                # append msg
                detections_msg.detections.append(detection)

            # publish detections and dbg image
            self._pub.publish(detections_msg)
            self._dbg_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image,
                                                               encoding=msg.encoding))


def main():
    rclpy.init()
    node = YoloNasNode()
    rclpy.spin(node)
    rclpy.shutdown()
