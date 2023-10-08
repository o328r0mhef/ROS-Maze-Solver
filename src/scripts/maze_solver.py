#!/usr/bin/env python
import rospy
import time
import datetime
import pandas as pd
import os
from geometry_msgs.msg import Twist, Pose
from demo_programs.msg import prox_sensor, line_sensor
from tf.transformations import euler_from_quaternion


def get_date_and_time():
    """
    Generates formatted current date (dd.mm.yyyy) and time (HH:MM:SS).
    Returns:
        current_date (str)
        current_time (str)
    """
    now = datetime.datetime.now()
    current_date = now.strftime("%d.%m.%Y")
    current_time = now.strftime("%H:%M:%S")
    return current_date, current_time 


def create_logfile(logfile_name='logfile'):
    """
    Creates a pandas dataframe .txt file with naming format '<date+logfile name>'.
    If an identical logfile exits the new logfile name is incremented.
    Parameter:
        logfile_name ogl(str): logfile name (optional)
    Returns:
        logfile_with_extension (str): Generated logfile name, with incrementation and .txt extension
    """
    columns = {'Date     ': [], 'Time': [], '        Published message': []}
    dataframe = pd.DataFrame(columns)
    date, time = get_date_and_time()
    logfile_without_extension = date + ' ' + logfile_name
    logfile_with_extension = date + ' ' + logfile_name + '.txt'
    counter = 2
    if os.path.isfile(logfile_with_extension):
        while True:
            if os.path.isfile(logfile_without_extension + ' (' + str(counter) + ')' + '.txt'):
                counter += 1
            else:
                logfile_with_extension = logfile_without_extension + ' (' + str(counter) + ')' + '.txt'
                break
            
    dataframe.to_csv(logfile_with_extension, sep='\t', index=False)
    return logfile_with_extension

 

def update_logfile(message, file_name):
    """
    Appends a new row to logfile with current date, time and a custom message.
    Parameters:
        message   (str): Custom log message
        file_name (str): File to be updated
    """
    dataframe = pd.read_csv(file_name, sep='\t')
    date, time = get_date_and_time()
    dataframe.loc[len(dataframe.index)] = [date, time, message]
    dataframe.to_csv(file_name, sep='\t', index=False)

    
file_name = create_logfile("Maze Solver logfile")

# Initialise start time
start_time = time.time()

# 'end' variable determines if finish line has been reached. Initialised as 'False'
end = False

class Movement_:
    """
    Movement class to simplify the logic in prox_callback. each function publishes
    different linear and angular data to the '\cmd_vel' Topic of the robot, causing it
    to move.
    """
    def stop(self, movement, pub):
        """
        Stop robot moving. Called when the robot finishes the maze.
        """
        print("Robot stopped")
        movement.linear.x = 0
        movement.angular.z = 0
        pub.publish(movement)
        update_logfile("linear.x = 0., angular.z = 0", file_name)

    def forward(self, movement, pub):
        """
        Move robot straight without wturning.
        """
        movement.linear.x = 0.2
        movement.angular.z = 0
        pub.publish(movement)
        rospy.sleep(2)
        update_logfile("linear.x = 0.2, angular.z = 0", file_name)
        
    def right_forward(self, movement, pub):
        """
        Rotate robot right while moving forward.
        """
        print("Moving right-forwards")
        movement.linear.x = 0.1
        movement.angular.z = 0.6
        pub.publish(movement)
        update_logfile("linear.x = 0.1, angular.z = 0.6", file_name)
        
    def left_forward(self, movement, pub):
        """
        Rotate robot left while moving forward.
        """
        print("Moving left-forwards")
        movement.linear.x = 0.1
        movement.angular.z = -0.6
        pub.publish(movement) 
        update_logfile("linear.x = 0.1, angular.z = -0.6", file_name)
        
    def orientate(self, movement, pub):
        """
        Move robot towards end of maze and its initial orientation. This function corrects
        the robots orientation when no obstacles are detected.
        """
        if position < -1:
            print("Orienting right")
            movement.linear.x = 0.19
            movement.angular.z = 0.3
            pub.publish(movement)
            update_logfile("linear.x = 0.19, angular.z = 0.3", file_name)
        elif position > 1:
            print("Orienting left")
            movement.linear.x = 0.19
            movement.angular.z = -0.3
            pub.publish(movement)
            update_logfile("linear.x = 0.19, angular.z = -0.3", file_name)
        else:
            print("Moving forwards")
            movement.linear.x = 0.3
            movement.angular.z = 0
            pub.publish(movement)
            update_logfile("linear.x = 0.3, angular.z = 0", file_name)


def prox_callback(m):
    """
    Contains main logic of the robot. Uses the five front facing proximity sensors to determine
    the movement_ function called to subscribe linear and angular movement to the robot.
    
    Parameter: m (prox_sensor.msg): For each proximity sensor, returns 0 if no
               wall detected. returns 0.45 if a wall is detected, with the value decreasing 
               at closer distance.
    """
    global end
    global position
    
    # Execute following logic if finish line is not reached.
    if end == False:
        
        # Assign front sensors variables
        F = m.prox_front
        FL = m.prox_front_left
        FR = m.prox_front_right
        FLL = m.prox_front_left_left
        FRR = m.prox_front_right_right
        
        front_sensors = F + FL + FR
        
        movement = Twist()
        
        # Create Movement_ object
        mov_obj = Movement_()
        
        # Create publish variable to allow Movement_ object to publish Twist message to /cmd_vel
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        #If robot it straight or angled right
        if position >= 0:
        
            if front_sensors == 0:  # If no wall infront, move forward
                mov_obj.orientate(movement, pub)
               
                if FLL != 0:       #If only Left sensor sees wall, nudge away
                   mov_obj.right_forward(movement, pub)
                if FRR != 0:       #If only Left sensor sees wall, nudge away
                   mov_obj.left_forward(movement, pub)
                   
            # Move forward and right if wall is detected infront
            else: 
                mov_obj.right_forward(movement, pub)
             
        # If robot is angled left
        elif position < 0: 
            
            if front_sensors == 0: #If no wall infront, move forward
                mov_obj.orientate(movement, pub)
               
                if FLL != 0:       #If only Left sensor sees wall, nudge away
                   mov_obj.right_forward(movement, pub)
                if FRR != 0:       #If only Left sensor sees wall, nudge away
                   mov_obj.left_forward(movement, pub)
             
            # Move forward and left if wall is detected infront
            else:
                mov_obj.left_forward(movement, pub)
        
   
def position_callback(pos_msg):
    """
    Parameter: pos_msg (Point.msg) Position of robot in free space in Quanternions.
    """
    global position
    
    def get_euler_rotation(msg):
         """
        Convert position from Quantion to degrees in the Y-axis.
        Parameter: (Point.msg) Position of robot in free space in Quanternions.
        """
         pi = 3.141592653589793238
         orientation_q = msg.orientation
         orientation = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
         (roll, pitch, yaw) = euler_from_quaternion(orientation)
         if yaw > 0:
             yaw = (-yaw + pi) * 180/pi
         else:
             yaw = (-yaw - pi) * 180/pi
         return yaw
     
    position = get_euler_rotation(pos_msg) # Define current position
    rospy.loginfo("Position %s", position)# Log position
    
    
def line_callback(line_msg):
    """
    Callback function sets global variable 'end' to True when the finish line is deteected.
    When this is set, the time for the maze completion is calculated and printed in the terminal.
    The robot continues to move forwards after the finish line to reach the end wall of the maze.
    
    Parameter: line_msg (line_sensor.msg): Binary message, True when line is detected.
    """
    global end
 
    movement = Twist()
    mov_obj = Movement_()
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) 
    
    # If line is detected, set global 'end' to True.
    if line_msg.line_middle == True:
        end = True
        mov_obj.forward(movement, pub)
        
        time_ = time.time() - start_time # Calculate time to complete maze.
        print("Maze complete in " + str("%.2f" % time_) + " seconds")

        while(True): # Stop robot 
             mov_obj.stop(movement, pub)
            
    
def subscribers():
    """
    Subscribes to three Topics; pose, line_sensors and prox_sensors. The pose
    subscriber sends movement data to the robot to simulate actuators.
    The line_sensors and prox_sensors subscribers subscripe to sensor Topics
    simulating sensor feedback from Line sensors and Proximity sensors.
    """
    rospy.init_node("run")
    rospy.Subscriber("/cop/pose", Pose, position_callback, queue_size=1) # Position data
    rospy.Subscriber("/cop/line_sensors", line_sensor, line_callback)
    rospy.Subscriber("/cop/prox_sensors", prox_sensor, prox_callback) # Proximity sensor data
   
    rospy.spin()
    
    
while not rospy.is_shutdown():
    subscribers()