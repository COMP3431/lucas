import rclpy
import cv2
import numpy as np
import math

from cv_bridge import CvBridge
from cv_bridge.core import CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image

HALF = 0.5
GREEN = (0, 255, 0)

def ptDist(p1, p2):
    dx=p2[0]-p1[0]; dy=p2[1]-p1[1]
    return math.sqrt( dx*dx + dy*dy )

def ptMean(p1, p2):
    return ((int(p1[0]+p2[0])/2, int(p1[1]+p2[1])/2))

def rect2centerline(rect):
    p0=rect[0]; p1=rect[1]; p2=rect[2]; p3=rect[3];
    width=ptDist(p0,p1); height=ptDist(p1,p2);

    # centerline lies along longest median
    if (height > width):
        cl = ( ptMean(p0,p1), ptMean(p2,p3) )
    else:
        cl = ( ptMean(p1,p2), ptMean(p3,p0) )

    return cl

# Define a function to show the image in an OpenCV Window
def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

def process_image(cv_image):
    resized = cv2.resize(cv_image, None, fx=HALF, fy=HALF)
    blurred = cv2.GaussianBlur(resized, (5, 5), 0)
    edges = cv2.Canny(blurred, threshold1=30, threshold2=100)
    return edges

def convert_to_binary(cv_image):
    ret, thresh = cv2.threshold(cv_image, 150, 255, cv2.THRESH_BINARY)
    return thresh

def find_contours(cv_image):
    thresh = convert_to_binary(cv_image)
    # determine primary axis, using largest contour
    im2, contours, h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    maxC = max(contours, key=lambda c: cv2.contourArea(c))
    boundRect = cv2.minAreaRect(maxC)
    centerline = rect2centerline(cv2.boxPoints(boundRect))
    cv2.line(cv_image, centerline[0], centerline[1], (0,0,255))
    return cv_image


class VisionSubscriber(Node):
    def __init__(self):
        super().__init__('vision_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.subscription

    def image_callback(self, ros_image):
        self.get_logger().info("Image received on subscribed topic")

        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(ros_image, "mono8")
            print(cv_image)
        except CvBridgeError as e:
            print(e)

        show_image(process_image(cv_image))
        
        

def main(args=None):
    rclpy.init(args=args)
    vision_subscriber = VisionSubscriber()
    try:
        rclpy.spin(vision_subscriber)
    except KeyboardInterrupt:
        vision_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("Program is terminated :)")

if __name__ == "__main__":
    main()