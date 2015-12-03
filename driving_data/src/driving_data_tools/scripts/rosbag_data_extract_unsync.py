__author__ = 'patty'
import sys
import os
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from can_msg.msg import CANPacket
from nmea_msgs.msg import Sentence
import pcl
import sensor_msgs.point_cloud2 as pc2

save_path=None

def img_loader_0(image_msg):
    bridge = CvBridge()
    camera_img_0 = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    timestamp=image_msg.header.stamp.secs+((image_msg.header.stamp.nsecs+0.0)/1000000000)
    save_image(camera_img_0,timestamp,save_path,'0')

def img_loader_1(image_msg):
    bridge = CvBridge()
    camera_img_1 = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    timestamp=image_msg.header.stamp.secs+((image_msg.header.stamp.nsecs+0.0)/1000000000)
    save_image(camera_img_1,timestamp,save_path,'1')

def img_loader_2(image_msg):
    bridge = CvBridge()
    camera_img_2 = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    timestamp=image_msg.header.stamp.secs+((image_msg.header.stamp.nsecs+0.0)/1000000000)
    save_image(camera_img_2,timestamp,save_path,'2')

def can_loader(msg):
    timestamp=msg.header.stamp.secs+((msg.header.stamp.nsecs+0.0)/1000000000)
    save_can(msg.dat,timestamp,save_path)

def nmea_loader(msg):
    timestamp=msg.header.stamp.secs+((msg.header.stamp.nsecs+0.0)/1000000000)
    save_nmea(msg.sentence,timestamp,save_path)

def cloud_loader(msg):
    timestamp=msg.header.stamp.secs+((msg.header.stamp.nsecs+0.0)/1000000000)
    save_pcd(msg,timestamp,save_path)

def save_pcd(cloud,timestamp,path):
    p = pcl.PointCloud(np.array(list(pc2.read_points(cloud)),dtype=np.float32)[:,0:3])
    p.to_file(path+'/pcd'+'/pcd'+'_'+str(timestamp)+'.pcd')

def save_image(img,timestamp,path,sfx):
    cv2.imwrite(path+'/camera'+sfx+'/camera_'+sfx+'_'+str(timestamp)+'.png',img)

def save_can(can,timestamp,path):
    f = open(path+'/can'+'/can.bin', 'ab')
    f.write(can)
    f.close()

def save_nmea(nmea,timestamp,path):
    f = open(path+'/nmea'+'/nmea.csv', 'a')
    f.write(str(timestamp)+','+nmea)
    f.write('\n')
    f.close()

def rosbag_data_extract_unsync():
    global save_path
    try:
        save_path=sys.argv[1]
    except Exception,e:
        sys.exit("Please specify the save path. Example: rosbag_data_extract_unsync.py /media/0/output/")

    if not os.path.exists(save_path):
        os.makedirs(save_path)
    if not os.path.exists(save_path+'/camera0'):
        os.makedirs(save_path+'/camera0')
    if not os.path.exists(save_path+'/camera1'):
        os.makedirs(save_path+'/camera1')
    if not os.path.exists(save_path+'/camera2'):
        os.makedirs(save_path+'/camera2')
    if not os.path.exists(save_path+'/can'):
        os.makedirs(save_path+'/can')
    if not os.path.exists(save_path+'/pcd'):
        os.makedirs(save_path+'/pcd')
    if not os.path.exists(save_path+'/nmea'):
        os.makedirs(save_path+'/nmea')

    rospy.init_node('rosbag_data_extract_unsync', anonymous=True)

    rospy.Subscriber("/camera0/image_raw", Image, img_loader_0)
    rospy.Subscriber("/camera1/image_raw", Image, img_loader_1)
    rospy.Subscriber("/camera2/image_raw", Image, img_loader_2)
    rospy.Subscriber("/points_raw", PointCloud2, cloud_loader)
    rospy.Subscriber("/can_raw", CANPacket, can_loader)
    rospy.Subscriber("/nmea_sentence", Sentence, nmea_loader)
    rospy.spin()

if __name__ == '__main__':
    rosbag_data_extract_unsync()
