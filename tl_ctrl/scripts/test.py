#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class CameraSmokeTest:
    def __init__(self):
        # 파라미터: 이미지 토픽(필요시 launch에서 remap 가능)
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.show_window = rospy.get_param("~show_window", True)

        self.bridge = CvBridge()
        self.last_t = None
        self.fps_avg = 0.0
        self.n = 0

        rospy.loginfo(f"[CAM TEST] Subscribing: {self.image_topic}")
        self.sub = rospy.Subscriber(self.image_topic, Image, self.cb, queue_size=1, buff_size=2**24)

    def cb(self, msg):
        # FPS 계산
        now = time.time()
        if self.last_t is not None:
            fps = 1.0 / max(1e-6, (now - self.last_t))
            self.n += 1
            self.fps_avg = (self.fps_avg * (self.n - 1) + fps) / self.n
            if self.n % 30 == 0:
                rospy.loginfo(f"[CAM TEST] fps(inst)={fps:.1f}, fps(avg)={self.fps_avg:.1f}")
        self.last_t = now

        # 이미지 디코드
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn(f"[cv_bridge] conversion failed: {e}")
            return

        # 윈도우 표시(헤드리스면 False로)
        if self.show_window:
            try:
                cv2.imshow("ROS Camera SmokeTest", bgr)
                cv2.waitKey(1)
            except Exception:
                pass  # X display 없는 환경

def main():
    rospy.init_node("camera_smoketest")
    node = CameraSmokeTest()
    rospy.loginfo("[CAM TEST] Ready. Ctrl+C to exit.")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

if __name__ == "__main__":
    main()
