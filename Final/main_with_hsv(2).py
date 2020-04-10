# -*- coding: utf-8 -*-
import math
import time
import rospy
import cv2
import numpy as np

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
        self.barcodeData = "none"
        self.bridge = CvBridge()
        self.cv_image = None
        self.image_pub = rospy.Publisher('~debug', Image, queue_size=1)
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
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, 'bgr8'))
            rospy.sleep(0.3)
        arr = list(filter(lambda a: a != 'none', arr))
        return self.most_frequent(arr)

    def waitDataColor(self):
        #COLOR
        arr = []
        for _ in range(3):
            self.cv_image = cv2.resize(self.cv_image, (160, 120))
            # for i in range(len(self.cv_image)):
            #     for i1 in range(len(self.cv_image[0])):
            #         self.cv_image[i][i1] = self.cv_image[i][i1][0], self.cv_image[i][i1][1], self.cv_image[i][i1][2] // 2
            height, width, _ = self.cv_image.shape
            height, width = height // 2, width // 2
            hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
            
            lower_yellow = np.array([10,50,80])
            upper_yellow = np.array([35,255,255])
            lower_red = np.array([-18,80,80])
            upper_red = np.array([10,255,255])
            lower_green = np.array([35,80,80])
            upper_green = np.array([80,255,255])
            
            res1 = cv2.inRange(hsv, lower_green, upper_green)
            res2 = cv2.inRange(hsv, lower_red, upper_red)
            res3 = cv2.inRange(hsv, lower_yellow, upper_yellow)
            gre, yel, red = 0, 0, 0
            for i in range(len(res1) // 3, len(res1) // 3 * 2):
                for i1 in range(len(res1[0]) // 3, len(res1[0]) // 3 * 2):
                    if res1[i][i1] != 0:
                        gre += 1
                    if res2[i][i1] != 0:
                        red += 1
                    if res3[i][i1] != 0:
                        yel += 1
            if max(gre, yel, red) == gre and gre != 0:
                self.color = "green"
            elif max(gre, yel, red) == yel and yel != 0:
                self.color = "yellow"
            else:
                self.color = "red"
            arr.append(self.color)
            rospy.sleep(0.5)
        self.color = self.most_frequent(arr)
        height, width = res1.shape
        cv2.putText(res1, "green", (0, height // 8),cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1) 
        cv2.putText(res2, "red", (0, height // 8),cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1) 
        cv2.putText(res3, "yellow", (0, height // 8),cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)  
        image = np.zeros((height,width), np.uint8)
        cv2.putText(image, str("Result: "), (height // 4, width // 4),cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
        cv2.putText(image, str(self.color), (height // 4, width // 4 + 20),cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)

        h1, w1 = res1.shape[:2]
        h2, w2 = res2.shape[:2]
        result1 = np.zeros((max(h1, h2), w1+w2), np.uint8)
        result1[:h1, :w1] = res1
        result1[:h2, w1:w1+w2] = res2
        
        h1, w1 = res3.shape[:2]
        h2, w2 = image.shape[:2]
        result2 = np.zeros((max(h1, h2), w1+w2), np.uint8)
        result2[:h1, :w1] = res3
        result2[:h2, w1:w1+w2] = image
        
        h1, w1 = result1.shape[:2]
        h2, w2 = result2.shape[:2]
        result = np.zeros((h1 + h2, max(w1, w2)), np.uint8)
        result[:h1, :w1] = result1
        result[h1:h1+h2, :w2] = result2
        
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(result, cv2.COLOR_GRAY2BGR), 'bgr8'))
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

def navigate_aruco(x=0, y=0, z=0, yaw=float('nan'), speed=0.4,  floor=False):
    return navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id='aruco_map')

def get_telemetry_aruco():
    telem = get_telemetry(frame_id="aruco_map")
    return telem

def takeoff(z):
    telem = get_telemetry_aruco()
    navigate(z=z, speed=0.4, frame_id="body", auto_arm=True)
    rospy.sleep(2)
    navigate_aruco(x=telem.x, y=telem.y, z=z, speed=0.4, floor=True)

def navigate_wait(x, y, z, speed=0.4, tolerance=0.13):
    navigate_aruco(x=x, y=y, z=z, speed=speed)
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        # print(telem.x, telem.y, telem.z)
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

print("Done Takeoff and waiting 12 seconds")

rospy.sleep(12)
for i in range(len(d)):
    navigate_wait(d[i][0], d[i][1], 0.6)
    rospy.sleep(0.2)
    color = rc.waitDataColor()
    if color == "red" or color == "yellow":
        print("PURPLE LED COLOR ON")
        set_effect(r=128, g=0, b=128)
        rospy.sleep(5)
        set_effect(r=0, g=0, b=0)
        print("PURPLE LED COLOR OFF")
        print("доставлено")
    if color == "red":
        r1.append(d[i])
        r2.append([d[i], "+"])
        print("[DEBUG]", "+")
        # r3.append([d[i], "+"])
    elif color == "yellow":
        r1.append(d[i])
        r2.append([d[i], "?"])
        print("[DEBUG]", "?")
    else:
        r2.append([d[i], "-", "Healthy"])
        print("[DEBUG]", "-")
    
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
            # r2[i1], r2[i1 + 1] = r2[i1 + 1], r2[i1]

print("WAITING FOR 2 MINS")
rospy.sleep(120)
print("TAKEOFF")
takeoff(1)

for i in range(len(r1)):
    navigate_wait(r1[i][0], r1[i][1], 0.6)
    rospy.sleep(1.5)
    text = rc.waitDataQR()
    print(text)
    if text == "COVID - 19":
        r2[d.index(r1[i])].append("COVID - 2019")
        print("RED LED COLOR ON")
        set_effect(r=255, g=0, b=0)
        rospy.sleep(5)
        set_effect(r=0, g=0, b=0)
        print("RED LED COLOR OFF")
    elif text == "healthy":
        r2[d.index(r1[i])].append("Healthy")
    else:
        r2[d.index(r1[i])].append("non COVID - 2019")

navigate_wait(0.2, 0.2, 0.6)
navigate_wait(0, 0, 0.6)

land()

str1, str2, str3, str4 = "", "", "", ""

for i in range(len(r2)):
    str1 = str1 + str(i + 1) + ","
    str2 = str2 + "x = " + str(r2[i][0][0]) + " y = " + str(r2[i][0][1]) + ','
    str3 = str3 + str(r2[i][1]) + ","
    str4 = str4 + str(r2[i][2]) + ","

str1, str2, str3, str4 = str1[:-1], str2[:-1], str3[:-1], str4[:-1]
otchet_per = str1 + "\n" + str2 + "\n" + str3 + "\n" + str4
otchet = open("report.csv", "w")
otchet.write(otchet_per)
otchet.close()

# print(r1)