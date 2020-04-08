# -*- coding: utf-8 -*-
import math
import time
import rospy
import cv2

try:
    from clover             import srv
    from clover.srv         import SetLEDEffect
except:
    from clever             import srv
    from clever.srv         import SetLEDEffect
from std_srvs.srv           import Trigger
from mavros_msgs.srv        import CommandBool
from sensor_msgs.msg        import Image
from std_msgs.msg           import String
from pyzbar                 import pyzbar
from cv_bridge              import CvBridge

class Recognition:
    def __init__(self):
        self.color = "none"
        self.barcodeData = "QRNONE"
        self.bridge = CvBridge()
        self.cv_image = None
        self.image_pub = rospy.Publisher('~debug', Image)
        self.image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, self.image_callback)
        
    def image_callback(self, data):
        self.cv_image = cv2.resize(self.bridge.imgmsg_to_cv2(data, 'bgr8'), (320, 240))

    def most_frequent(self, arr):
        try:
            return max(set(arr), key = arr.count)
        except:
            return "none"
    
    def waitDataQR(self):
        #QR
        arr = []
        for _ in range(3):
            gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            barcodes = pyzbar.decode(gray)
            for barcode in barcodes:
                (x, y, w, h) = barcode.rect
                self.barcodeData = barcode.data.decode("utf-8")
                xc = x + w/2
                yc = y + h/2
                self.cv_image = cv2.circle(self.cv_image, (xc, yc), 15, (0, 0, 0), 30)
                arr.append(self.barcodeData)
                print(self.barcodeData)
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, 'bgr8'))
        return self.most_frequent(arr)

    def waitDataColor(self):
        #COLOR
        arr = []
        for _ in range(5):
            height, width, _ = self.cv_image.shape
            height, width = height // 2, width // 2
            dots = [[140, 100], [140, 120], [140, 140], [160, 100], [160, 120], [160, 140], [180, 100], [180, 120], [180, 140]]
            for i in range(9):
                height, width = dots[i]
                b, g, r = self.cv_image[dots[i][0]][dots[i][1]]
                mi = min(r, g, b)
                if r + g + b - 3 * mi < 60:
                    continue
                else:
                    break
            # b, g, r = self.cv_image[height][width]
            if r > g + b and r > g * 1.8 and r > b * 1.8:
                self.color = "red"
            elif g > r + b and g > r * 1.8 and g > b * 1.8:
                self.color = "green"
            else:
                self.color = "yellow"
            print(r, g, b)
            cv2.circle(self.cv_image, (height, width), 10, (0,0,0), -1)
            cv2.putText(self.cv_image, self.color, (height * 2 // 3, width * 2 // 3), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 255, 255), 10) 
            print(self.color)
            arr.append(self.color)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, 'bgr8'))
        print(self.most_frequent(arr))
        return self.most_frequent(arr)

# initialisation #

rospy.init_node('flight')
rc = Recognition()
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
land_serv = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)


def most_frequent(List): 
    return max(set(List), key = List.count)

def navigate_aruco(x=0, y=0, z=0, yaw=float('nan'), speed=0.2,  floor=False):
    return navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id='aruco_map')

def get_telemetry_aruco():
    telem = get_telemetry(frame_id="aruco_map")
    return telem

def takeoff(z):
    telem = get_telemetry_aruco()
    navigate(z=z, speed=0.3, frame_id="body", auto_arm=True)
    rospy.sleep(2)
    navigate_aruco(x=telem.x, y=telem.y, z=z, speed=0.3, floor=True)

def navigate_wait(x, y, z, speed=0.2, tolerance=0.18):
    navigate_aruco(x=x, y=y, z=z, speed=speed)
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        print(telem.x, telem.y, telem.z)
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def land(disarm = True):
    land_serv()
    rospy.sleep(5)
    arming(False)

# initialisation #

d = [(0.295, 0.295),
    (0.885, 0.295),
    (0.295, 0.885),
    (0.885, 0.885),
    (0.295, 1.475),
    (0.885, 1.475),
    (0.295, 2.065),
    (0.885, 2.065),
    (0.59, 2.655)] # coordinates

print("Done INIT")

r1 = []
r2 = []
r3 = []
takeoff(0.7)
rospy.sleep(12)
for i in range(len(d)):
    navigate_wait(d[i][0], d[i][1], 0.6)
    color = rc.waitDataColor()
    if color == "red" or color == "yellow":
        set_effect(r=128, g=0, b=128)
        rospy.sleep(5)
        set_effect(r=0, g=0, b=0)
        print("доставлено")
    if color == "red":
        r1.append(d[i])
        r2.append([d[i], "+"])
        # r3.append([d[i], "+"])
    elif color == "yellow":
        r1.append(d[i])
        r2.append([d[i], "?"])
    else:
        # r1.append(d[i])
        # r2.append(["-"])
        r2.append([d[i], "-", "Healthy"])
    print("d[i]:", d[i], "color:", color, "r1[-1]:", r1[-1], "r2[-1]:", r2[-1])
    
navigate_wait(0.2, 0.2, 0.6)    
navigate_wait(0, 0, 0.6) #to home
land()
dis = []
for i in range(len(r1)):
    if i == 0:
        dis.append(math.sqrt(r1[i][0] ** 2 + r1[i][1] ** 2))
    else:
        dis.append(math.sqrt((r1[i][0] - r1[i - 1][0]) ** 2 + (r1[i][1] - r1[i - 1][1]) ** 2))
        
for i in range(len(r1) - 1):
    for i1 in range(len(r1) - i - 1):
        if dis[i1] > dis[i1 + 1]:
            dis[i1], dis[i1 + 1] = dis[i1 + 1], dis[i1]
            r1[i1], r1[i1 + 1] = r1[i1 + 1], r1[i1]
            r2[i1], r2[i1 + 1] = r2[i1 + 1], r2[i1]


rospy.sleep(120)

takeoff(1)

for i in range(len(r1)):
    navigate_wait(r1[i][0], r1[i][1], 0.6)
    text = rc.waitDataQR()
    if text == "COVID - 19":
        r2[i].append("COVID - 2019")
        set_effect(r=255, g=0, b=0)
        rospy.sleep(5)
        set_effect(r=0, g=0, b=0)
    elif text == "healthy":
        r2[i].append("Healthy")
    else:
        r2[i].append("non COVID - 2019")

navigate_wait(0.2, 0.2, 0.6)
navigate_wait(0, 0, 0.6)

land()



str1, str2, str3, str4 = "", "", "", ""

for i in range(len(r2)):
    str1 = str1 + str(i + 1) + ";"
    str2 = str2 + "x = " + str(r2[i][0][0]) + " y = " + str(r2[i][0][1]) + ';'
    str3 = str3 + str(r2[i][1]) + ";"
    str4 = str4 + str(r2[i][2]) + ";"

str1, str2, str3, str4 = str1[:-1], str2[:-1], str3[:-1], str4[:-1]
otchet_per = str1 + "\n" + str2 + "\n" + str3 + "\n" + str4
otchet = open("report.csv", "w")
otchet.write(otchet_per)
otchet.close()

# print(r1)
