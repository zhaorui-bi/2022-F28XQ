import cv2
import robomaster
import time
from robomaster import robot
from robomaster import vision
from robomaster import led

class RobotInfo:

    def __init__(self, x, y, w, h):
        self._x = x
        self._y = y
        self._w = w
        self._h = h

    @property
    def pt1(self):
        return int((self._x - self._w / 2) * 1280), int((self._y - self._h / 2) * 720)

    @property
    def pt2(self):
        return int((self._x + self._w / 2) * 1280), int((self._y + self._h / 2) * 720)

    @property
    def center(self):
        return int(self._x * 1280), int(self._y * 720)

robots = []                                     # 视野中机器人的信息
distance = []                                   # 传感器距离的信息
def sub_data_sensor(sub_info):
    global distance
    distance = sub_info
    # print("tof1:{0}  tof2:{1}  tof3:{2}  tof4:{3}".format(distance[0], distance[1], distance[2], distance[3]))
#获取机器人位置信息与传感器信息

def on_detect_car(car_info):
    number = len(car_info)
    robots.clear()
    if number > 0:
        for i in range(0, number):
            x, y, w, h = car_info[i]             #获取机器人的位置等信息
            robots.append(RobotInfo(x, y, w, h))
            print("robot: x:{0}, y:{1}, w:{2}, h:{3}".format(x, y, w, h))
    else:
        print('Nothing i can see')


if __name__ == '__main__':
    # 机器人初始化
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_robot.set_robot_mode(mode=robot.CHASSIS_LEAD)

    ep_sensor = ep_robot.sensor
    ep_sensor.sub_distance(freq=5, callback=sub_data_sensor)
    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_led = ep_robot.led
    ep_chassis = ep_robot.chassis
    ep_camera.start_video_stream(display=False)
    ep_gimbal.recenter();
    result = ep_vision.sub_detect_info(name="robot", callback=on_detect_car)

    maxv = 0.4
    #error = robots.copy()
    time.sleep(3)
    while True:

        img = ep_camera.read_cv2_image(strategy="newest", timeout=5)
        for j in range(0, len(robots)):
            cv2.rectangle(img, robots[j].pt1, robots[j].pt2, (255, 255, 255))
        cv2.imshow("robots", img)
        cv2.waitKey(1)
        current = robots.copy()

        if len(current)>0:
            position_x=current[0]._x
            position_y=current[0]._y
            err_x = 3 * (position_x-0.5)
            ep_chassis.drive_speed(x=0, y=0, z=150*err_x, timeout=5)
            time.sleep(0.03)
            if len(distance)>0:
                print('distance=', distance[0])
                v = (distance[0]-350) * 0.0015
                if v > maxv:
                    v = maxv
                elif v < -maxv:
                    v = -maxv
                print('v=', v)
                ep_chassis.drive_speed(x=v, y=0, z=0, timeout=5)
                time.sleep(0.05)
        else :
            # = current
            ep_chassis.drive_speed(x=0, y=0, z=50, timeout=5)
            time.sleep(0.01)
    ep_sensor.unsub_distance()
    result = ep_vision.unsub_detect_info(name="robot")
    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()