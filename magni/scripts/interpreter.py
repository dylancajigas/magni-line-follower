#!/usr/bin/python3
import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Range
from sensor_msgs.msg import Joy


# internal setup
camera_angle = 0.0
ahead_sonar = 1.0
diag_l_sonar = 1.0
diag_r_sonar = 1.0
left_sonar = 1.0
stop_timer = 0.0
code_data = "none"
controller_button_held = False

# sets the camera angle global variable when receiving it from the subscription
def set_camera_angle(msg):
    global camera_angle
    if not msg.data == "none":
        camera_angle = float(msg.data)
    else:
        camera_angle = 180.0
    
# sets the sonar global variables when receiving them from the subscription
def set_sonars(msg):
    global ahead_sonar
    global left_sonar
    global diag_l_sonar
    global diag_r_sonar
    if msg.header.frame_id == "sonar_1":
        diag_l_sonar = msg.range
    elif msg.header.frame_id == "sonar_2":
        diag_r_sonar = msg.range
    elif msg.header.frame_id == "sonar_3":
        ahead_sonar = msg.range
    elif msg.header.frame_id == "sonar_4":
        left_sonar = msg.range

# sets the code global variable when receiving it from the subscription
def set_code(msg):
    global code_data
    code_data = msg.data

# sets the controller input global variable when receiving it from the subscription
def set_joy(msg):
    global controller_button_held
    controller_button_held = msg.buttons[6] == 1
    rospy.loginfo(msg.buttons)



if __name__ == "__main__":
    # node setup
    rospy.init_node("interpreter")
    rate = rospy.Rate(30)

    # set up publisher/subscribers
    pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    pub_state = rospy.Publisher("/line_follower/state", String, queue_size=10)
    rospy.Subscriber("line_follower/camera_angle", String, set_camera_angle)
    rospy.Subscriber("line_follower/code", String, set_code)
    rospy.Subscriber("/sonars", Range, set_sonars)
    rospy.Subscriber("/joy", Joy, set_joy)

    # retrieve tuning variables
    ahead_speed = rospy.get_param("forwardSpeed")
    ahead_threshold = rospy.get_param("angleThresholdForward")
    angle_multiplier = rospy.get_param("angleMultiplier") 
    turn_speed = rospy.get_param("turnSpeed")
    max_turn_speed = rospy.get_param("maxTurnSpeed")
    stop_proximity = rospy.get_param("stopProximity")
    stop_time_seconds = rospy.get_param("passthroughStopTime")
    slow_turn_angle = rospy.get_param("slowTurnAngle")
    slow_turn_speed = rospy.get_param("slowTurnSpeed")
    
    # internal variable setup
    prev_state = 0
    controller_button_held_last_tick = False
    begin_time = time.time()
    delta_time = 0.0
    num_stations = 0
    current_station = 0
    passthrough = 0
    state = 0
    # 0 - at home (qrcode/barcode)
    # 1 - moving
    # 2 - between stops advancing (no line)
    # 3 - at a stop

    rospy.loginfo("Interpreter initialized")

    # main loop
    while not rospy.is_shutdown():
        msg = Twist()
        if state == 0:
            # QR/Barcode control
            # L=#:S=#,#,#
            # L is the number of stations in the loop (not including home/start)
            # S is the stations to stop at
            if code_data != "none" and "L=" in code_data and ":S=" in code_data:
                # parses QR/Barcode data
                stations = []
                num_stations = int(code_data[2:code_data.find(":")])
                code_data = code_data[code_data.find(":") + 3:]
                while "," in code_data != -1:
                    stations.append(int(code_data[:code_data.find(",")]))
                    code_data = code_data[code_data.find(",") + 1:]
                if len(code_data) > 0:
                    stations.append(int(code_data))

                state = 2

        elif state == 1:
            # line following
            if camera_angle != 180.0:
                stop_timer = stop_time_seconds
                if ahead_sonar > stop_proximity and diag_l_sonar > stop_proximity and diag_r_sonar > stop_proximity:
                    # moves at a different speed if turning, turns slower when angle is closer to 0 for more precision
                    if abs(camera_angle) <= ahead_threshold:
                        msg.linear.x = ahead_speed
                    elif abs(camera_angle) > slow_turn_angle:
                        msg.linear.x = turn_speed
                        msg.angular.z = max(min(max_turn_speed, camera_angle * -angle_multiplier), -max_turn_speed) # clamps turning speed
                    else:
                        if(camera_angle < 0):
                            msg.linear.x = turn_speed
                            msg.angular.z = slow_turn_speed
                        else:
                            msg.linear.x = turn_speed
                            msg.angular.z = -slow_turn_speed
            else:
                # reaching end of a line, counts down until advancing or going into standby
                if stop_timer > 0:
                    stop_timer -= delta_time
                else:
                    if current_station in stations:
                        state = 3
                    elif current_station >= num_stations + 1:
                        current_station = 0
                        state = 0
                    else:
                        stop_timer = stop_time_seconds
                        state = 2

        elif state == 2: 
            # advancing while at a station
            if camera_angle == 180:
                msg.linear.x = ahead_speed
            else:
                stop_timer = stop_time_seconds
                current_station += 1
                state = 1

        elif state == 3:
            # advances robot after reaching a stop and being unloaded
            if left_sonar < 0.1:
                stop_timer = stop_time_seconds
                state = 2

        # controller halts process
        if controller_button_held:
            controller_button_held_last_tick = True
            state = 4
        elif controller_button_held_last_tick:
            controller_button_held_last_tick = False
            state = prev_state

        # publish to topics
        if state != 4:
            prev_state = state
            pub_vel.publish(msg)
        pub_state.publish(str(state))

        # calculates time between cycles
        delta_time = time.time() - begin_time
        begin_time = time.time()

        rate.sleep()