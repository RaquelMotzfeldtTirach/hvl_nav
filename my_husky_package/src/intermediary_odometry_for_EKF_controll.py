#!/usr/bin/env python3
import rospy
import rospkg

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

class RelayNode():
    
    def __init__(self, fix_input: str):
        
        # Initilizes the variables needed
        self.ekf_odometry = Odometry()   #Odometry gained from the EKF filter
        self.gps_fix_status = None
        self.husky_odometry = Odometry() #Current odometry from the husky
        self.husky_odometry_minus_1 = Odometry() #Last odometry from the husky
        self.calculated_odometry = Odometry()    #The difference between the husky odoms added to the EKF odom

        # Initilizes the subscribers
        self.odom_sub = rospy.Subscriber('/husky_velocity_controller/odom', Odometry, self.odom_relay)
        self.ekf_sub = rospy.Subscriber('/global_ekf/odometry/filtered', Odometry, self.odom_from_EKF)
        self.gps_sub = rospy.Subscriber('/odometry/gps', Odometry, self.odom_from_gps_relay)
        self.fix_sub = rospy.Subscriber(fix_input, NavSatFix, self.gps_fix)

        #Initializes all the publishers
        self.odom_pub = rospy.Publisher('/husky_odom/filtered', Odometry, queue_size=10)
        self.gps_pub = rospy.Publisher('/emlid/fix/filtered', Odometry, queue_size=10)

        rospy.loginfo("Relay node started")
        
    # This function is the callback when the odometry is updated.
    # If the GPS data is not sufficient, use only the odometry data
    def odom_relay(self, data):
        
        #Updates the odometry data
        self.husky_odometry_minus_1 = self.husky_odometry
        self.husky_odometry = data  
        
        # Status on the GPS signal
        # 1: Satelite based augmentation, Only GNSS
        # 2: Ground based augmentation, GNSS + RTK
        if(self.gps_fix_status == 2):
            self.odom_pub.publish(self.husky_odometry) # With GNSS + RTK we have sufficiently accurate measurements
        else:
            # Without good enough GPS signal, we use the odometry data layed on top of the EKF data
            # Calculating the difference in movement from the last two odometry measurements
            y_difference = self.husky_odometry.pose.pose.position.y - self.husky_odometry_minus_1.pose.pose.position.y
            x_difference = self.husky_odometry.pose.pose.position.x - self.husky_odometry_minus_1.pose.pose.position.x

            # Adds the difference to the EKF odometry
            # Copies the ekf_odometry
            self.calculated_odometry = self.ekf_odometry
            
            #Adds the difference
            self.calculated_odometry.pose.pose.position.x = self.ekf_odometry.pose.pose.position.x + x_difference
            self.calculated_odometry.pose.pose.position.y = self.ekf_odometry.pose.pose.position.y + y_difference
            
            #Publish it to the EKF.
            self.odom_pub.publish(self.calculated_odometry)
    
    # This function is the callback when the EKF odometry is updated.        
    def odom_from_EKF(self, data):
        #Saves the odometry from the EKF in a global variable used elsewhere
        self.ekf_odometry = data
        
    # This function is the callback when the GPS odometry is updated from the navsat node.
    # The data coming in here is the odometry data in relation to the robot.
    def odom_from_gps_relay(self, data):
        #Checks if there is a GPS signal whith ground augmentation. If not, it sends the last odometry.
        #This might not be needed. Testing required.
        if(self.gps_fix_status == 2):
            self.gps_pub.publish(data)
        else:
            self.gps_pub.publish(self.calculated_odometry)
            
    # This function is the callback when the GPS fix is updated.
    # It updates the status of the GPS signal. Which can be augmented or not.
    def gps_fix(self, data):
        #Checks the status of the GPS sender. We are looking for Status 2, which means it has ground based augmentation.
        self.gps_fix_status = data.status.status
        


if __name__ == '__main__':
    
    rospy.init_node('intermediary_odometry_for_EKF_controll', anonymous=True)
    
    pub_debugger = rospy.Publisher('/debugger', String , queue_size=10)
    
    
    
    # Gets topic name from the launch file. If not found, it uses the default value.
    if rospy.has_param('/fix_input'):
        fix_input = rospy.get_param('/fix_input')
    else:
        fix_input = '/emlid/fix/simulated'
        
    if rospy.has_param("/fix_output"):
        fix_output = rospy.get_param("/fix_output")
    else:
        fix_output = '/gps/fix/filtered'
        
    #pub_debugger.publish(f"fix_input: {fix_input}")
    pub_string = String("hello world")
    #rospy.sleep(1)
    pub_debugger.publish("Hello world")
    relay_node = RelayNode(fix_input)

    # #Initializes all the subscribers
    # odom_sub = rospy.Subscriber('/husky_velocity_controller/odom', Odometry, odom_relay)
    # ekf_sub = rospy.Subscriber('/global_ekf/odometry/filtered', Odometry, odom_from_EKF)
    # gps_sub = rospy.Subscriber('/odometry/gps', Odometry, odom_from_gps_relay)
    # fix_sub = rospy.Subscriber('/emlid/fix', NavSatFix, gps_fix)

    # #Initializes all the publishers
    # odom_pub = rospy.Publisher('/husky_odom/filtered', Odometry, queue_size=10)
    # gps_pub = rospy.Publisher('/emlid/fix/filtered', Odometry, queue_size=10)
    # #fix_pub = rospy.Publisher('/fix_status', Int16, queue_size=10) ##This is for testing purposes

    rospy.loginfo("Intermediary node for EKF filter is ready")

    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        
        rate.sleep()
    
    #rospy.spin()