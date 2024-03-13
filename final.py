import cv2
import time
import numpy as np
import robomaster
from robomaster import robot
from robomaster import led
from robomaster import camera
from robomaster import vision


# 巡线模块
class PointInfo:
    def __init__(self, x, y, theta, c):
        self._x = x
        self._y = y
        self._theta = theta
        self._c = c

    @property
    def pt(self):
        return int(self._x * 1280), int(self._y * 720)

    @property
    def color(self):
        return 255, 255, 255


lines = []


def on_detect_line(line_info):
    number = len(line_info)  # 看识别到了多少线信息
    lines.clear()
    if number > 0:
        line_type = line_info[0]
        for i in range(1, number):
            x, y, theta, c = line_info[i]  # ceta 为切线角 c为曲率
            lines.append(PointInfo(x, y, theta, c))
    else:
        print("未识别到线信息")


# 红灯停模块
def process(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    low_hsv = np.array([0, 43, 46])
    high_hsv = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lowerb=low_hsv, upperb=high_hsv)  # 取值函数
    cv2.namedWindow('shibie_image', cv2.WINDOW_AUTOSIZE)  # 新建颜色提取显示窗口
    cv2.imshow('shibie_image', mask)  # 显示颜色提取后的图像
    dst = cv2.blur(mask, (1, 16))  # 均值模糊 : 去掉提取完颜色的随机噪声图片
    circles = cv2.HoughCircles(dst, cv2.HOUGH_GRADIENT, 1, 40, param1=100, param2=25, minRadius=60, maxRadius=70)
    pos = [0, 0]
    if circles is not None:
        ret = True
        print('get')
        ep_led.set_led(comp=led.ARMOR_BOTTOM_ALL, r=255, g=255, b=255, effect=led.EFFECT_ON)
        time.sleep(0.5)
        ep_led.set_led(comp=led.COMP_BOTTOM_ALL, r=0, g=0, b=0, effect=led.EFFECT_OFF)
    else:
        print('未识别到圆形图案')
        ret = False
    return image, pos, ret


def follow():
    result = ep_vision.unsub_detect_info(name="marker")
    result = ep_vision.sub_detect_info(name="line", color="blue", callback=on_detect_line)
    lines_tmp = lines.copy()
    for j in range(0, len(lines_tmp)):
        cv2.circle(img, lines_tmp[j].pt, 3, lines_tmp[j].color, -1)
        """ 进行PID控制
                1.实现巡线功能
                2.实现优化，即直线加速，弯道减速效果
            """
    if len(lines_tmp) > 0:
        point_x_3 = lines_tmp[4]._x
        error_3 = point_x_3 - 0.5
        angle_output = 125 * error_3
        point_x_8 = lines_tmp[8]._x
        error_8 = 0.5 - point_x_8
        speed_output = 0.2 - 0.25 * error_8  # 0.8即初始速度，0.7为比例系数， error_8即我们用来调控速度的误差
        ep_chassis.drive_speed(x=speed_output, y=0, z=angle_output)
# 云台射击模块
class MarkerInfo:
    def __init__(self, x, y, w, h, info):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._info = info

    @property
    def pt1(self):
        return [int((self._x - self._w / 2) * 1280), int((self._y - self._h / 2) * 720)]

    @property
    def pt2(self):
        return int((self._x + self._w / 2) * 1280), int((self._y + self._h / 2) * 720)

    @property
    def center(self):
        return int(self._x * 1280), int(self._y * 720)

    @property
    def text(self):
        return self._info


markers = []
marker_id = []


def on_detect_marker(marker_info):
    number = len(marker_info)
    markers.clear()
    for i in range(0, number):
        x, y, w, h, info = marker_info[i]
        markers.append(MarkerInfo(x, y, w, h, info))
        # print("marker:{0} x:{1}, y:{2}, w:{3}, h:{4}".format(info, x, y, w, h))


def sub_data_handler(sub_info):
    global distance
    distance = sub_info


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera
    ep_chassis = ep_robot.chassis
    ep_gimbal = ep_robot.gimbal
    ep_led = ep_robot.led
    ep_camera.start_video_stream(display=False)
    ep_gimbal.moveto(pitch=-20, yaw=0).wait_for_completed()
    ep_robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
    # sub_detect_info 即订阅智能识别信息，在这里可以识别到蓝色的线，callback即返回一个包含线信息的一个数组,这里用的是自己写的一个方法
    i=0
    c=['a','b','c','d','e','f','g','h','i','j','k','l','m','n']
    while True:

        result = ep_vision.unsub_detect_info(name="line")
        result = ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.3)
        marker_copy = markers.copy()
        for j in range(0, len(marker_copy)):
            cv2.rectangle(img, marker_copy[j].pt1, marker_copy[j].pt2, (255, 255, 255))
            cv2.putText(img, marker_copy[j].text, marker_copy[j].center, cv2.FONT_HERSHEY_SIMPLEX, 1.5,
                        (255, 255, 255), 3)
            print(marker_copy[0]._x)
        if len(marker_copy) > 0:  # 云台射击
            # ep_robot.set_robot_mode(mode=robot.FREE)
            # 在进行拍照前停下来
            if marker_copy[0]._info not in marker_id:
                ep_chassis.drive_speed(x=0, y=0, z=0)
                position_x = marker_copy[0]._x
                position_y = marker_copy[0]._y
                errorx = position_x - 0.5
                errory = 0.5 - position_y
                outputx = 160 * errorx
                outputy = 100 * errory
                # print(output)
                ep_gimbal.drive_speed(pitch_speed=outputy, yaw_speed=outputx)
                time.sleep(0.5);
                ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0);
                # time.sleep(0.5)
                if 0 <= abs(errorx) <= 0.05 and 0 <= abs(errory) <= 0.05:
                    # ep_led.set_led(comp='all', r=0, g=0, b=0, effect='off')
                    cv2.putText(img, 'Team 05 detected a mark with ID of ' + marker_copy[0]._info, (140, 300),
                                cv2.FONT_ITALIC, 1.5,
                                (255, 255, 255), 3)
                    cv2.imwrite('D:\\image\\' + marker_copy[0]._info + '.jpg', img)  # 记住在考试时应该重新改地址
                    # 拍照完以后将标签信息记录，已经记录过的不会拍照了
                    marker_id.append(marker_copy[0]._info),
                    ep_gimbal.moveto(pitch=-20, yaw=0).wait_for_completed()
                    ep_robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
            else:
                follow()
        img = ep_camera.read_cv2_image(strategy="newest")
        # if img is not None:  # 对照片进行判断，进行红绿灯监测
        result, pos, p_ret = process(img)
        cv2.imshow("result", result)
        key = cv2.waitKey(1)
        if p_ret is True:
            ep_chassis.drive_speed(x=0, y=0, z=0)
            cv2.putText(img, 'Team 06 detected a Red Light', (140, 300), cv2.FONT_ITALIC, 1.5,
                        (255, 255, 255), 3)
            cv2.imwrite('D:\\image\\' 'Team 05 detected a Red Light_'+c[i] +'.jpg',img)
            i=i+1
            time.sleep(1.5)


        else:  # 巡线
            follow()


        # if key == ord('q'):
        # break
        # else:
        #    print('未获取到图像信息')
        #    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)
    result = ep_vision.unsub_detect_info(name="line")
    result = ep_vision.unsub_detect_info(name="marker")
    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()