#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv

# internal setup 
global_img = cv.UMat() 
global_img_populated = False
bridge = CvBridge()

# checks if the pixel is in the threshold    
def checkEdge(image, r, c):
    blue = image.item(r, c, 0)
    green = image.item(r, c, 1)
    red = image.item(r, c, 2)
    return (green - red >= threshold_difference_g and
            green - blue >= threshold_difference_g and
            abs(red - blue) <= threshold_difference_br)

# sets the image global variable when receiving it from the subscription
def set_image(msg):
    global global_img_populated
    global_img_populated = True
    global global_img 
    global_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

# main loop
if __name__ == "__main__":
    # initiate node
    rospy.init_node("debugging_image")
    rate = rospy.Rate(30)

    # set up publishers and subscribers
    pub2 = rospy.Publisher("/debugging/threshold_image", Image, queue_size=10)
    rospy.Subscriber("/raspicam_node/image", Image, set_image)
    
    # retrieve tuning variables
    top_height = rospy.get_param("topHeight")
    bot_height = rospy.get_param("botHeight") 
    thickness = rospy.get_param("lineThickness") 
    threshold_difference_g = rospy.get_param("greenDifference")
    threshold_difference_br = rospy.get_param("blueRedDifference") 
    bot_line_multiplier = rospy.get_param("bottomLineMultiplier") 
    centering_multiplier = rospy.get_param("centeringMultiplier") 
    resize_h = rospy.get_param("resizeHeight") 
    resize_w = rospy.get_param("resizeWidth")

    # main loop
    while not rospy.is_shutdown():
        if global_img_populated:
            # debugging image generator (CAUSES HEAVY, ~1 SECOND LATENCY)
            cv_img = cv.resize(global_img, (resize_w, resize_h), interpolation = cv.INTER_AREA)
            tempflatten = cv_img.copy()
            for r in range(resize_h):
                for c in range(resize_w):
                    if(checkEdge(cv_img, r, c)):
                        tempflatten[r, c] = (0,255,0)
                    else:
                        tempflatten[r, c] = (0,0,0)
            for i in range(resize_w):
                tempflatten.itemset((resize_h - top_height - 1, i, 0), 255)
                tempflatten.itemset((resize_h - bot_height - 1, i, 0), 255)
            pub2.publish(bridge.cv2_to_imgmsg(tempflatten, encoding="passthrough"))
        rate.sleep()