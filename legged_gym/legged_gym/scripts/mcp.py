import rospy
import numpy as np
import time
import copy
from mcp_helper import *

from std_msgs.msg import Float32MultiArray
from svan_msgs.msg import Trajectory

last_joystick_msg_time_max = 10

def get_joystick_data(msg):
    for i in range(len(msg.data)):
        joy_data[i] = msg.data[i]
    
    # global last_joy_msg
    # last_joy_msg = 0

    # last_joy_msg = rospy.Time.now()
    # print("joystick_data:", joy_data)

rospy.init_node('motor_command_publisher', anonymous=True)

ee_ref_pub = rospy.Publisher('/svan/ee_ref', Trajectory, queue_size=1)
rospy.Subscriber("/svan/joystick_data", Float32MultiArray, get_joystick_data)

joy_data = [0] * 9

def command_publisher():
    rate = rospy.Rate(250)

    start = rospy.Time.now()
    curr = start.secs + 1e-9 * start.nsecs
    freq = 1.0
    done = 0

    # 0 is stance to sleep, and 1 is sleep to stance
    mode = 1
    Tp_vert = 2
    Tp_feet = 0.5

    if (mode == 0):
        t_adjust_feet = 0
        t_vert = t_adjust_feet + Tp_feet
    elif (mode == 1):
        t_vert = 0
        t_adjust_feet = t_vert + Tp_vert
    else:
        print("Please input mode as 0 or 1.")
        return

    t0 = [t_adjust_feet, t_adjust_feet, t_adjust_feet, t_adjust_feet]
    
    start_motion = False
    motion_id = 0
    prev_motion_id = 0
    switch_motion = False
    switch_time = 0

    adjust_z_height_direction = False
    target_z_height = 0

    alpha = 0.1
    x_step = 0
    y_step = 0

    gait_des = gait(t0 = t0, phi_thresh=[0.5, 0.5, 0.5, 0.5], phi_offset=[0.5, 0, 0.5, 0], gait_period=Tp_feet, name="sleep")

    # motion flags
    flag_pushup = False
    flag_twirl = False
    flag_close = False
    in_stop = False
    not_switch_motion_id = False

    adjust_z = False

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        curr = (now.secs - start.secs) + 1e-9 * (now.nsecs - start.nsecs)

        if (mode == 0):
            ee_ref_data.gait_id = 1
            # stance to sleep: first adjust feet, then get down
            if (curr > t_adjust_feet and curr <= t_vert):
                gait_des.scheduled_contact(curr)
                adjust_feet(curr, gait_des, mode)
            elif (curr > t_vert):
                stance(curr, dir = 'dn', t0 = t_vert, Tp = Tp_vert)
        else:
            ee_ref_data.gait_id = 1
            # sleep to stance: first get up, then adjust feet 
            if (curr > t_vert and curr <= t_adjust_feet):
                stance(curr, dir = 'up', t0 = t_vert, Tp = Tp_vert)
            elif (curr > t_adjust_feet):
                gait_des.scheduled_contact(curr)
                adjust_feet(curr, gait_des, mode)

        if (curr > Tp_vert + Tp_feet and (not start_motion)):
            ee_ref = [0, 0, 0.34, 0, 0, 0, 0.2017, -0.138, 0, 0.2017, 0.138, 0,  -0.2017, 0.138, 0,  -0.2017, -0.138, 0]
            ee_ref_vec.data = copy.copy(ee_ref)
            ee_ref_data.position = copy.copy(ee_ref)
            ee_ref_data.gait_id = 1
            start_motion = True
            in_stop = True
            motion_id = 1

        # timeout safety
        # last_joystick_msg_time = (last_joy_msg.secs - start.secs)
        # time_elapsed_since_last_joy_msg = curr - last_joystick_msg_time

        # if (time_elapsed_since_last_joy_msg > last_joystick_msg_time_max):
        #     joy_data = [0] * 9
        #     motion_id = 1
        #     print("No joystick data received. Setting the robot to stance.")

        # now use the joystick commands
        not_switch_motion_id = (joy_data[0] == 2 or joy_data[0] == 3 or joy_data[0] == 4) and (motion_id != 1)
        if not not_switch_motion_id:
            if joy_data[0] != 0:
                motion_id = copy.copy(joy_data[0]) # possible values A1, X2, B3, Y4, RB5

        right_value = -0.2 * joy_data[1]
        forward_value = 0.3 * joy_data[2]
        roll_value = 1 * joy_data[3]
        pitch_value = 2 * joy_data[4]
        yaw_value = -(joy_data[5] - joy_data[6])

        if (motion_id != prev_motion_id):
            switch_motion = True
            switch_time = curr
            if motion_id == 4:
                trot.switch = 0
                trot.theta = 0            
        else:
            switch_motion = False
        
        # update the flags
        flag_pushup = (motion_id == 3) and in_stop
        flag_twirl = (motion_id == 2) and in_stop
        flag_close = (motion_id == 6) and in_stop

        if (start_motion):
            if (motion_id == 4):
                in_stop = False
                x_step =  (1 - alpha) * x_step + alpha * forward_value
                y_step =  (1 - alpha) * y_step + alpha * right_value
                trot(curr, x_length = x_step, y_length = y_step, z_height = 0.10, f = 3  * freq, t0 = switch_time)

                # Z-height adjustment
                if joy_data[8] and not adjust_z:
                    adjust_z = True
                    adjust_z_height_direction = joy_data[8]
                    target_z_height = ee_ref_data.position[2] + adjust_z_height_direction * delta_z_height

                    if target_z_height >= max_z_height:
                        target_z_height = max_z_height

                    if target_z_height <= min_z_height:
                        target_z_height = min_z_height

                if adjust_z:
                    ee_ref_data.position[2] = (1 - alpha_z_height) * ee_ref_data.position[2] + alpha_z_height * target_z_height
                    if (abs(ee_ref_data.position[2] - target_z_height) <= 0.001):
                        adjust_z = False
     
            elif (flag_twirl):
                twirl(curr, alpha = np.pi/24, f = 0.8, t0 = switch_time)

            elif (flag_pushup):
                pushUp(curr, 0.8 * freq, stroke = 0.15, t0 = switch_time)
                
            elif (motion_id == 1):
                in_stop = True
                stand()

            elif (flag_close):
                in_stop = False
                return

            # if (curr > 6):
            #     if not done:
            #         done = Jump(curr, t0 = 6)
            #     else:
            #         stand()

        # adding the orientation correction commands
        ee_ref_data.position[3] = ee_ref_data.position[3] + 0.1 * roll_value
        ee_ref_data.position[4] = ee_ref_data.position[4] + 0.05 * pitch_value
        ee_ref_data.position[5] = ee_ref_data.position[5] + 0.1 * yaw_value

        ee_ref_pub.publish(ee_ref_data)

        # avoiding accumulation that leads to divergence of values
        ee_ref_data.position[3] = ee_ref_data.position[3] - 0.1 * roll_value
        ee_ref_data.position[4] = ee_ref_data.position[4] - 0.05 * pitch_value
        ee_ref_data.position[5] = ee_ref_data.position[5] - 0.1 * yaw_value

        prev_motion_id = motion_id

        rate.sleep()

def send_zero():
    ee_ref_data.gait_id = 0
    ee_ref_data.position = copy.copy(sleep_ee_state)
    ee_ref_data.velocity = np.zeros(18)
    ee_ref_data.acceleration = np.zeros(18)

    ee_ref_pub.publish(ee_ref_data)

    print(ee_ref_data.velocity)
    time.sleep(0.2)
    rospy.loginfo("Trying to send the last message...")

if __name__ == "__main__":
    try:
        rospy.on_shutdown(send_zero)
        command_publisher()
    except rospy.ROSInterruptException:
        pass
