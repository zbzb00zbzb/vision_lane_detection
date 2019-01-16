#!/usr/bin/python
import os
import argparse
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy

mode = 0
src_pt = np.ones([4, 2], dtype='float32')
dst_pt = np.ones([4, 2], dtype='float32')
dst_pt[0:,] = [174., 448.]
dst_pt[1:,] = [174., 299.] 
dst_pt[2:,] = [274., 299.]
dst_pt[3:,] = [274., 448.]

def gps_callback(gps):

    if mode == 's':
        f.write('%1.10f %1.10f %1.10f %1.10f %1.10f %1.10f\n'%\
                (gps.pose.pose.position.x, \
                 gps.pose.pose.position.y, \
                 gps.pose.pose.position.z, \
                 gps.pose.pose.orientation.x, \
                 gps.pose.pose.orientation.y, \
                 gps.pose.pose.orientation.z))
    elif mode == 'r':
        print '%1.10f, %1.10f' % (gps.pose.pose.position.x, gps.pose.pose.position.y)
        # plt.plot(gps.pose.pose.position.x, gps.pose.pose.position.y, '.')
        plt.plot(gps.pose.pose.position.x, gps.pose.pose.position.y, 'r.')
        plt.axis('equal')
        plt.pause(0.001)


def OnMouseAction(event,x,y,flags,param):
    global mode
    global src_pt
    global dst_pt

    if mode == 0 and event == cv2.EVENT_LBUTTONDOWN:
        print "press mouse 1"
        src_pt[mode,:] = [x,y]
        mode = mode + 1
        return

    if mode == 1 and event == cv2.EVENT_LBUTTONDOWN:
        print "press mouse 2"
        src_pt[mode,:] = [x,y]
        mode = mode + 1
        return

    if mode == 2 and event == cv2.EVENT_LBUTTONDOWN:
        print "press mouse 3"
        src_pt[mode,:] = [x,y]
        mode = mode + 1
        return

    if mode == 3 and event == cv2.EVENT_LBUTTONDOWN:
        print "press mouse 4"
        src_pt[mode,:] = [x,y]
        mode = 4
        return

def image_callback(image):
    global mode
    global flip
    global src_pt
    global dst_pt
    np_data = np.fromstring(image.data, np.uint8)
    im = cv2.imdecode(np_data, 0)
    image = cv2.cvtColor(im, cv2.COLOR_BAYER_BG2BGR)
    if flip == 1:
    	image = cv2.flip(image, -1)
    cv_img = cv2.resize(image, (960, 600))
    #img_data = np.fromstring(msg.data, np.uint8)
    #cv_img = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
    #image = cv2.resize(cv_img, (960, 600))
    
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', OnMouseAction)
    cv2.imshow('image', cv_img)
    k=cv2.waitKey(0)
    if k == ord('n'):
        mode = 0
    elif k == ord('c'):
        if mode == 4:
            print src_pt
            print dst_pt
            src_pt = src_pt * 2
            trans = cv2.getPerspectiveTransform(src_pt, dst_pt)
            dst_img = cv2.warpPerspective(image, trans, (448, 448))
            cv2.imshow("tmp", dst_img)
            print "trans"
            print trans
            print ("image_to_ipm_tf: %f" %trans[0][0])
            print ("image_to_ipm_tf: %f" %trans[0][1])
            print ("image_to_ipm_tf: %f" %trans[0][2])
            print ("image_to_ipm_tf: %f" %trans[1][0])
            print ("image_to_ipm_tf: %f" %trans[1][1])
            print ("image_to_ipm_tf: %f" %trans[1][2])
            print ("image_to_ipm_tf: %f" %trans[2][0])
            print ("image_to_ipm_tf: %f" %trans[2][1])
            print ("image_to_ipm_tf: %f" %trans[2][2])
            np.savetxt(r'/tmp/trans.txt', trans)
            mode = 0
    elif k == ord('q'):
            cv2.destroyWindow('tmp')
    else:  
        mode = 0

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="caculate perspective matrix from ros topic.")
    parser.add_argument("image_topic", help="image topoic.")
    parser.add_argument("image_flip", help="image flip flag.")
    args = parser.parse_args()
    mode = 0
    flip = 0
    rospy.init_node('save_image', anonymous=True)
    if args.image_flip == "1":
	flip = 1
    rospy.Subscriber(args.image_topic, CompressedImage, image_callback)
    rospy.spin()
