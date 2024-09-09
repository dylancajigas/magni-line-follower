#!/usr/bin/python3
import rospy
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2 as cv

# internal variable setup 
global_img = cv.UMat() 
global_img_populated = False
bridge = CvBridge()
state = 0

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
    
# sets the state global variable when receiving it from the subscription
def set_state(msg):
    global state
    state = int(msg.data)

if __name__ == "__main__":
    # node initialization
    rospy.init_node("camera")
    rate = rospy.Rate(30)

    # set up publishers and subscribers
    pub_angle = rospy.Publisher("/line_follower/camera_angle", String, queue_size=10) # 0 is straight ahead, left is negative, right is positive
    pub_code = rospy.Publisher("/line_follower/code", String, queue_size=10)
    rospy.Subscriber("/raspicam_node/image", Image, set_image)
    rospy.Subscriber("/line_follower/state", String, set_state)
    
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
    
    rospy.loginfo("Camera initialized")

    while not rospy.is_shutdown():
        # don't run if no proper image is in the global_image variable
        if global_img_populated:
            msg_angle = "none"
            msg_code = "none"

            if state == 0:
                # qr/barcode detecting
                code_list = decode(global_img)
                if len(code_list) >= 1:
                    msg_code = code_list[0].data.decode()
                    if len(msg_code) <= 0:
                        msg_code = "none"
            else:
                # line detecting
                cv_img = cv.resize(global_img, (resize_w, resize_h), interpolation = cv.INTER_AREA)

                # calculate line centers
                bot_trailing = 0
                top_trailing = 0
                bot_line_index = -1
                top_line_index = -1
                for i in range(resize_w):
                    if checkEdge(cv_img, resize_h - top_height - 1, i):
                        top_trailing += 1
                        if top_trailing >= thickness:
                            top_line_index = i - (top_trailing / 2.0)
                    else:
                        top_trailing = 0

                    if checkEdge(cv_img, resize_h - bot_height - 1, i):
                        bot_trailing += 1
                        if bot_trailing >= thickness:
                            # account for perspective/warping from the camera on the bottom edge, essentially adjusting the bottom line index to match the top
                            bot_line_index = ((i - (bot_trailing / 2.0) - (resize_w/2.0)) * bot_line_multiplier) + (resize_w/2.0)
                    else:
                        bot_trailing = 0

                # calculate and publish angle
                if top_line_index != -1 and bot_line_index != -1:
                    msg_angle = str(top_line_index - bot_line_index + ((top_line_index + bot_line_index - resize_w)/2.0 * centering_multiplier))

            # publish to topics
            pub_angle.publish(msg_angle)
            pub_code.publish(msg_code)
                
        rate.sleep()