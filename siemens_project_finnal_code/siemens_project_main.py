# coding=utf-8
import pickle
import json
import socket
import time
import rospy
import cv2
from dashboard_client import DashboardClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from agv_base import Agv
from get_point_cloud import GetPointCloud


def trans_image_client():
    global img_suber
    rospy.init_node('image_client')
    img_suber = rospy.Subscriber('/camera/color/image_raw', Image, callback=callback, queue_size=1)
    while not rospy.is_shutdown():
        rospy.sleep(1)
    s.close()


def callback(data):
    img_suber.unregister()
    br = CvBridge()
    global img_suber
    im_array = br.imgmsg_to_cv2(data)
    im_array = cv2.resize(im_array, (640, 480), interpolation=cv2.INTER_AREA)
    im_array = cv2.cvtColor(im_array, cv2.COLOR_BGR2RGB)
    cv2.imwrite("image_raw.png", im_array)
    senddata = pickle.dumps(im_array)
    print "len(senddata):", len(senddata)
    global s
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.connect(('192.168.1.20', 3000))
    except Exception as e:
        print e
    s.send(senddata)
    print 'send over'
    res = s.recv(1024)
    print res
    global res
    global count
    s.close()
    global bounding_box
    bounding_box = True
    parse()
    if not bounding_box:
        img_suber = rospy.Subscriber('/camera/color/image_raw', Image, callback=callback, queue_size=1)
        return
    print 'count----------------------------: ', count
    count += 1
    if count >= 5:
        rospy.signal_shutdown("yes")
        try:
            ur_c.loadURP("0621_siemens_close_the_door.urp")
            print " load 0621_siemens_close_the_door.urp ok..."
            # time.sleep(3)
            ur_c.play()
            print " running 0621_siemens_close_the_door.urp"
            time.sleep(3)
            while True:
                if ur_c.programState().split(" ")[0] == "STOPPED":
                    print "close the door programe run over"
                    break

            agv.navigation("LM2")
            while True:
                print agv.get_navigation_state()
                if agv.get_navigation_state() == 4:
                    print "AGV has got the begin position"
                    break

            ur_program = None
            for k in range(1, 6):
                if k == 1:
                    ur_program = "0708_put_bottles_on_rack_to_table_01.urp"
                if k == 2:
                    ur_program = "0708_put_bottles_on_rack_to_table_02.urp"
                if k == 3:
                    ur_program = "0708_put_bottles_on_rack_to_table_03.urp"
                if k == 4:
                    ur_program = "0708_put_bottles_on_rack_to_table_04.urp"
                if k == 5:
                    ur_program = "0708_put_bottles_on_rack_to_table_05.urp"

                ur_c.loadURP(ur_program)
                time.sleep(1.5)
                ur_c.play()
                time.sleep(3)
                while True:
                    if ur_c.programState().split(" ")[0] == "STOPPED":
                        print "put bottle {} to table ok".format(k)
                        break
            print "**********************    all steps run over    *****************"

        except Exception as e:
            print e
    else:
        img_suber = rospy.Subscriber('/camera/color/image_raw', Image, callback=callback, queue_size=1)


def parse():
    global bounding_box
    try:
        result = json.loads(res)
        if not result["boundbox"]:
            print "boundbox datas is empty"
            bounding_box = False
            return
        # if result['class_conf'] < 0.8:
        #     print "error:class_conf < 0.8"
        #     return
        x_min = result["boundbox"]["xmin"]
        x_max = result["boundbox"]["xmax"]
        y_min = result["boundbox"]["ymin"]
        y_max = result["boundbox"]["ymax"]
        res_x = (x_min+x_max)/2
        res_y = (y_min+y_max)/2
        print "xmin:", x_min, "xmax:", x_max, "ymin:", y_min, "ymax:", y_max

        # 一个方法里面只能有一个rospy.init_node（）,前面已经有了rospy.init_node('image_client')
        # rospy.init_node('get_sample_pointcloud')

        get_point_cloud_obj = GetPointCloud(res_x, res_y, count)
        get_point_cloud_obj.wait_grab()

        # rospy.spin()
    except Exception as e:
        print e
        return


def capture_target():
    trans_image_client()


def main():
    try:
        # make sure the agv current position
        if agv.confirm_location():
            print " confirm location ok"
        else:
            print " confirm location failed"
            return
        if agv.query_robot_pose() != "LM2":
            print "AGV not at the begin position"
            agv.navigation("LM2")
            while True:
                print agv.get_navigation_state()
                if agv.get_navigation_state() == 4:
                    print "AGV has got the begin position"
                    break
        else:
            print "AGV is at the begin position"
        time.sleep(1)

        if agv.query_robot_pose() != "LM1":
            agv.navigation("LM1")
            while True:
                print agv.get_navigation_state()
                if agv.get_navigation_state() == 4:
                    print "AGV has got the work position"
                    break
        else:
            print "AGV is at the work position"

        # make UR do sth
        ur_c.connect()
        if ur_c.isConnected():
            print "UR is connected"
        else:
            print "UR connected error"
            return
        ur_c.loadURP("0708_siemens_open_the_door.urp")
        time.sleep(1)
        ur_c.play()
        time.sleep(3)
        while True:
            if ur_c.programState().split(" ")[0] == "STOPPED":
                print "open the door programe run over"
                break
        time.sleep(3)
        capture_target()
    except Exception as e:
        print e


if __name__ == '__main__':
    # create a agv obj
    agv = Agv()

    # ur control obj
    ur_ip = "192.168.1.10"
    ur_c = DashboardClient(ur_ip)

    # create client socket in main function
    s = socket.socket()

    count = 1

    main()

