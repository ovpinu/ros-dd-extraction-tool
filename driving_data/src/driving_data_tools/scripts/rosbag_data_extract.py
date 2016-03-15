__author__ = 'patty'
import sys
import os
import rospy
import numpy as np
import cv2
import pcl
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from can_msg.msg import CANPacket
from nmea_msgs.msg import Sentence

save_path = None
camera_img_0 = None
camera_img_1 = None
camera_img_2 = None
cloud = None
nmea = None
can = None
camera_img_0_ready = False
camera_img_1_ready = False
camera_img_2_ready = False
cloud_ready = False
can_ready = False
nmea_ready = False

def img_loader_0(image_msg):
    global camera_img_0
    global camera_img_0_ready
    camera_img_0_ready = False
    bridge = CvBridge()
    camera_img_0 = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    camera_img_0_ready = True

def img_loader_1(image_msg):
    global camera_img_1
    global camera_img_1_ready
    camera_img_1_ready = False
    bridge = CvBridge()
    camera_img_1 = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    camera_img_1_ready = True

def img_loader_2(image_msg):
    global camera_img_2
    global camera_img_2_ready
    camera_img_2_ready = False
    bridge = CvBridge()
    camera_img_2 = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    camera_img_2_ready = True

def can_loader(msg):
    global can
    global can_ready
    can_ready = False
    can = msg
    can_ready = True

def nmea_loader(msg):
    global nmea
    global nmea_ready
    nmea = msg.sentence
    nmea_ready = True

def cloud_loader(msg):
    global cloud
    global cloud_ready
    cloud_ready = False
    cloud = msg
    cloud_ready = True
    save_data()
    reset_flags()

def save_pcd(cloud, timestamp, path):
    p = pcl.PointCloud(np.array(list(pc2.read_points(cloud)), dtype=np.float32)[:, 0:3])
    p.to_file(path + '/pcd' + '/pcd' + '_' + "{:.5f}".format(timestamp) + '.pcd')

def save_image(img, timestamp, path, sfx):
    cv2.imwrite(path + '/camera' + sfx + '/camera_' + sfx + '_' + "{:.5f}".format(timestamp) + '.png', img)

def save_can(can, timestamp, path):
    f = open(path+'/can'+'/can.bin', 'ab')
    f.write("{:.5f}".format(timestamp)+','+str(can.id)+','+str(can.dat))
    f.close()

def save_nmea(nmea, timestamp, path):
    f = open(path + '/nmea' + '/nmea.csv', 'a')
    f.write("{:.5f}".format(timestamp) + ',' + nmea)
    f.write('\n')
    f.close()

def save_data():
    if cloud_ready and camera_img_0_ready and camera_img_1_ready and camera_img_2_ready and can_ready and nmea_ready:
        this_cloud = cloud
        this_camera_img_0 = camera_img_0.copy()
        this_camera_img_1 = camera_img_1.copy()
        this_camera_img_2 = camera_img_2.copy()
        this_can = can
        this_nmea = nmea

        timestamp = cloud.header.stamp.secs + ((cloud.header.stamp.nsecs + 0.0) / 1000000000)
        save_pcd(this_cloud, timestamp, save_path)
        save_image(this_camera_img_0, timestamp, save_path, '0')
        save_image(this_camera_img_1, timestamp, save_path, '1')
        save_image(this_camera_img_2, timestamp, save_path, '2')
        save_can(this_can, timestamp, save_path)
        save_nmea(this_nmea, timestamp, save_path)

def reset_flags():
    globals()['camera_img_0_ready'] = False
    globals()['camera_img_1_ready'] = False
    globals()['camera_img_2_ready'] = False
    globals()['cloud_ready'] = False
    # globals()['can_ready'] = False
    # globals()['nmea_ready'] = False

def rosbag_data_extract():
    global save_path
    try:
        save_path = sys.argv[1]
    except Exception, e:
        sys.exit("Please specify the save path. Example: rosbag_data_extract.py /media/0/output/")

    if not os.path.exists(save_path):
        os.makedirs(save_path)
    if not os.path.exists(save_path + '/camera0'):
        os.makedirs(save_path + '/camera0')
    if not os.path.exists(save_path + '/camera1'):
        os.makedirs(save_path + '/camera1')
    if not os.path.exists(save_path + '/camera2'):
        os.makedirs(save_path + '/camera2')
    if not os.path.exists(save_path + '/can'):
        os.makedirs(save_path + '/can')
    if not os.path.exists(save_path + '/pcd'):
        os.makedirs(save_path + '/pcd')
    if not os.path.exists(save_path + '/nmea'):
        os.makedirs(save_path + '/nmea')

    rospy.init_node('rosbag_data_extract', anonymous=True)

    rospy.Subscriber("/camera0/image_raw", Image, img_loader_0)
    rospy.Subscriber("/camera1/image_raw", Image, img_loader_1)
    rospy.Subscriber("/camera2/image_raw", Image, img_loader_2)
    rospy.Subscriber("/points_raw", PointCloud2, cloud_loader)
    rospy.Subscriber("/can_raw", CANPacket, can_loader)
    rospy.Subscriber("/nmea_sentence", Sentence, nmea_loader)
    rospy.spin()

if __name__ == '__main__':
    rosbag_data_extract()
