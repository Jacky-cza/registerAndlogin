# coding=utf-8
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf import transformations
import numpy as np
import rtde_control
import cv2
import time
import rtde_receive
from dashboard_client import DashboardClient


class GetPointCloud(object):
    def __init__(self, res_x, res_y, count):
        self.pcsuber = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pointcloud_callback)
        self.res_x = res_x
        self.res_y = res_y
        self.grab = False
        self.count = count

    def wait_grab(self):
        while not self.grab:
            rospy.sleep(1)

    def pointcloud_callback(self, pc):
        """
        ÕâÀïµãÔÆÊýŸÝÊÇÒ»žöÒ»Î¬Êý×é£¬³€¶ÈÎª640*480*20žö×ÖœÚ¡£
        read_pointsµÄ²ÎÊýuvs±íÊŸºá×Ý×ø±ê£¬
        µ±ÐèÒª»ñÈ¡×ø±êÎª(x,y)µÄµãÔÆÊ±£šÕâžöxy×ø±êŽÓ1¿ªÊŒŒÆÊý£©£¬
        uvsŒÆËã·œÊœ£º[1280 * (y -1) + x * 2£¬0]
        xmin: 307
        ymin: 267
        xmax: 416
        ymax: 364
        """
        # mypc = PointCloud2()
        # mypc.header.frame_id = '/camera_color_optical_frame'
        # mypc.header.stamp = rospy.Time.now()
        uvs_list = list([int(1280 * (y - 1) + x), 0]
                        for x in range(int((self.res_x-5) * 2), int((self.res_x+5) * 2))
                        for y in range(int((self.res_y-5) * 1.5), int((self.res_y+5) * 1.5)))
        pcdata = pc2.read_points(pc, uvs=uvs_list,  skip_nans=True, field_names=('x', 'y', 'z'))
        x_list = []
        y_list = []
        z_list = []
        for p in pcdata:
            # print "x:", p[0], "y:", p[1], "z:", p[2]
            if 1.1 > p[2] > 0.2:
                x_list.append(p[0])
                y_list.append(p[1])
                z_list.append(p[2])
        if (len(x_list) == 0) or (len(y_list) == 0) or (len(z_list) == 0):
            print 'empty list'
            return
        x_average = np.mean(x_list)
        y_average = np.mean(y_list)
        z_average = np.mean(z_list)
        print "x average:", x_average
        print "y average", y_average
        print "z average", z_average

        self.grab_bottles(x_average, y_average, z_average)
        self.pcsuber.unregister()
        self.grab = True

    def grab_bottles(self, x, y, z):
        # if self.count > 3:
        # print " count will out of list range,return................."
        # return
        toolToCamera = np.array([[1, 0, 0, -0.029], [0, 1, 0, -0.11], [0, 0, 1, 0.033], [0, 0, 0, 1]])
        rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.10")
        ur_c = DashboardClient("192.168.1.10")
        ur_c.connect()
        tagPose = transformations.euler_matrix(0, 3.14, 1.57)
        # for i in [x-0.103, x-0.07, x-0.043, x-0.009, x+0.03]:
        # l = [x-0.103, x-0.07, x-0.043, x-0.009, x+0.03]
        l = [x-0.103, x-0.075, x-0.103, x-0.07]
        try:
            tagPose[0][3] = l[self.count-1]
            tagPose[1][3] = y
            tagPose[2][3] = z-0.02
            # tagpose:the QRcode's 6dof in camera coordinate system
            print 'detect tag pose: \n', tagPose

            # current TCP 6dof in Base coordinate system
            tcpPoseVec = rtde_r.getActualTCPPose()

            # cv2.Rodrigues():rotation vector to rotation matrix
            RtcpPose = cv2.Rodrigues(np.array(tcpPoseVec[3:]))[0]

            tcpPose = np.eye(4)
            tcpPose[:3, :3] = RtcpPose
            tcpPose[0][3] = tcpPoseVec[0]
            tcpPose[1][3] = tcpPoseVec[1]
            tcpPose[2][3] = tcpPoseVec[2]
            print 'tcpPose: \n', tcpPose

            targetPose = np.dot(tcpPose, np.dot(toolToCamera, tagPose))
            grabPose = np.zeros((4, 4))
            print grabPose
            grabPose[0][1] = -1
            grabPose[1][0] = -1
            grabPose[2][2] = -1
            grabPose[3][3] = 1
            print 'grabPose: \n', grabPose
            targetPose = np.dot(targetPose, grabPose)
            print 'targetPose:\n', targetPose

            tool = np.eye(4)
            tool[2][3] = -0.20
            lastTargetPose = np.dot(targetPose, tool)
            print 'lastTargetPose: \n', lastTargetPose

            targetTranslate = lastTargetPose[:3, 3]
            targetRotVec = cv2.Rodrigues(lastTargetPose[:3, :3])[0]

            print 'targetTranslate: \n', targetTranslate
            print 'targetRotateVector: \n', targetRotVec

            motion = [targetTranslate[0], targetTranslate[1], targetTranslate[2], targetRotVec[0][0],
                      targetRotVec[1][0], targetRotVec[2][0]]
            print 'motion:\n', motion
            print "*******************   start move robot   ***********************"

            ur_c.loadURP("onRobot_gripper_begin_pos.urp")
            time.sleep(1)
            ur_c.play()
            print " grippers get ready"
            time.sleep(1)

            rtde_c = rtde_control.RTDEControlInterface('192.168.1.10')
            if rtde_c.moveJ_IK([targetTranslate[0], targetTranslate[1], targetTranslate[2]+0.08, targetRotVec[0][0],
                      targetRotVec[1][0], targetRotVec[2][0]], 0.3, 0.2):
                print "get above the bottles 1 "
            else:
                print "get above the bottles 1 error"
                return
            time.sleep(1.5)

            if rtde_c.moveJ_IK([targetTranslate[0], targetTranslate[1], targetTranslate[2]+0.03, targetRotVec[0][0],
                      targetRotVec[1][0], targetRotVec[2][0]], 0.3, 0.2):
                print "get above the bottles 1 "
            else:
                print "get above the bottles 1 error"
                return
            time.sleep(1.5)

            ur_c.loadURP("onRobot_gripper_end_pos.urp")
            time.sleep(1)
            ur_c.play()
            print " grippers get the bottles"
            # time.sleep(1)
            rtde_c.disconnect()
            time.sleep(1.5)

            rtde_c = rtde_control.RTDEControlInterface('192.168.1.10')

            if rtde_c.moveJ_IK([targetTranslate[0], targetTranslate[1], targetTranslate[2] + 0.08, targetRotVec[0][0],
                                targetRotVec[1][0], targetRotVec[2][0]], 0.3, 0.2):
                print "get above the bottles 2 "
            else:
                print "get above the bottles 2 error"
                return
            time.sleep(1.5)

            if rtde_c.moveJ_IK([targetTranslate[0]+0.1, targetTranslate[1], targetTranslate[2] + 0.08, targetRotVec[0][0],
                                targetRotVec[1][0], targetRotVec[2][0]], 0.3, 0.2):
                print "get above the bottles 3 "
            else:
                print "get above the bottles 3 error"
                return
            time.sleep(1.5)

            # ur_camera_pose = [0.12090463, 0.2331876, 0.71827791, -1.27309327, -1.30593696, 1.11929615]
            arm_back_pose = [-0.22101674, 0.11030762, 0.57338447, -1.21220339, -1.19251144, 1.22308712]
            if rtde_c.moveJ_IK(arm_back_pose, 0.3, 0.2):
                print "get the arm back position "
            else:
                print "get the arm back position error"
                return
            rtde_c.disconnect()

            program = None
            print "start put grabbed bottles to rack............."
            if self.count == 1:
                program = "0708_put_grabbed_bottles_to_rack_1.urp"
            else:
            # elif self.count == 2:
                program = "0708_put_grabbed_bottles_to_rack_5.urp"
            # elif self.count == 3:
            #     program = "0708_put_grabbed_bottles_to_rack_3.urp"
            # elif self.count == 4:
            #     program = "0708_put_grabbed_bottles_to_rack_4.urp"
            # else:
            #     program = "0708_put_grabbed_bottles_to_rack_5.urp"
            try:
                ur_c.loadURP(program)
                print "load****"+program+"***ok"
                time.sleep(1.5)
                ur_c.play()
                time.sleep(3)
                while True:
                    if ur_c.programState().split(" ")[0] == "STOPPED":
                        print program+"***run over"
                        break
            except Exception as e:
                print e
                print "put grabbed bottles to rack error"
                return
        except Exception as e:
            print e
