#!/usr/bin/python3
import rospy
from oled_display_node.msg import DisplayOutput
from std_msgs.msg import String

STATE_MEANINGS = ["Scanning  ","Following ","Advancing ","Halted    ","Manual    "]
code = "none      "
state = 0
angle = "none      "

# sets the code global variable when receiving it from the subscription
def set_code(msg):
    global code
    if msg.data != "none":
        code = msg.data
        code = code + "          "

# sets the state global variable when receiving it from the subscription
def set_state(msg):
    global state
    state = int(msg.data)

# sets the angle global variable when receiving it from the subscription
def set_angle(msg):
    global angle
    angle = msg.data

# creates and returns a display message to publish (taken from ubiquity's oled_display_node)
def create_display_msg(row, column, text, comment):
       numChars = len(text)
       if numChars > 15:
         numChars = 15

       dispMsg = DisplayOutput()
       dispMsg.actionType    = 2               # Just use MSG_DISPLAY_SUBSTRING
       dispMsg.row           = row             # Display row with 0 as top row
       dispMsg.column        = column          # Display column in pixels
       dispMsg.numChars      = numChars        # Number of chars to be written
       dispMsg.attributes    = 0               # Just write with no attributes
       dispMsg.text          = text            # The text to be written
       dispMsg.comment       = comment         # Comment in the message, no functional use
       return dispMsg

if __name__ == "__main__":
    # initializes node
    rospy.init_node("display")
    rate = rospy.Rate(10)

    # set up publishers and subscribers
    pub_display = rospy.Publisher("/display_node", DisplayOutput, queue_size=10)
    rospy.Subscriber("line_follower/state", String, set_state)    
    rospy.Subscriber("line_follower/code", String, set_code)    
    rospy.Subscriber("line_follower/camera_angle", String, set_angle)    

    # main loop
    while not rospy.is_shutdown():
        if state == 0:
            code = "none      "

        # publish new lines to display
        pub_display.publish(create_display_msg(4,0, "AL: " + angle[:4] + "      ", ""))
        pub_display.publish(create_display_msg(2,0, "ST: " + STATE_MEANINGS[state], ""))
        pub_display.publish(create_display_msg(3,0, "CD: " + code[:10], ""))

        rate.sleep()
