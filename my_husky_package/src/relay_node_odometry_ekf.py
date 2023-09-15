#!/usr/bin/env python3
import rospy
import rospkg
import threading

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

# This node will relay the odometry from the husky and the EKF filter to the EKF filter.
# And it will relay the GPS data to the EKF filter.
# Based on the status of the GPS signal, the relay will either use the GPS data or the odometry data.


# Class for handling the data across several threads
class DataHandler():
    
    def __init__(self, input_gps_odometry: str, output_gps_odometry: str, fix_input: str, input_onboard_odometry: str, output_onboard_odometry: str, output_kalman_filter: str):
        self.ekf_odometry = Odometry()                          #Odometry gained from the EKF filter
        self.calculated_odometry = Odometry()                   #The difference between the husky odoms added to the EKF odom
        self.gps_fix_status = None                              #Status of the GPS signal
        self.input_gps_odometry = input_gps_odometry            # Input for the GPS odometry
        self.output_gps_odometry = output_gps_odometry          # Output for the GPS odometry
        self.fix_input = fix_input                              #The input for the GPS signal
        self.input_onboard_odometry = input_onboard_odometry    # Input for the onboard odometry
        self.output_onboard_odometry = output_onboard_odometry  # Output for the onboard odometry
        self.output_kalman_filter = output_kalman_filter        # Output for the kalman filter
        self.thread_lock = threading.Lock()                     #Lock for the threads
        
        # Publisher for publishing debug messages
        self.publisher = rospy.Publisher('/debug_relay_node', String, queue_size=10)
        rospy.sleep(1)          # Waits for the publisher to initilize
        
    # Gets the status of the GPS signal
    def get_gps_fix_status(self):
        with self.thread_lock:
            return self.gps_fix_status
        
    # Sets the status of the GPS signal
    def set_gps_fix_status(self, status: int):
        with self.thread_lock:
            self.gps_fix_status = status
            
    def get_ekf_odometry(self):
        with self.thread_lock:
            return self.ekf_odometry
        
    def set_ekf_odometry(self, data: Odometry):
        with self.thread_lock:
            self.ekf_odometry = data
            
    # Returns the topic name for the given topic type, if no valid is given, returns None
    def get_topic_name(self, topic_type: str):
        while self.thread_lock:
            if topic_type == "input_gps_odometry":
                return self.input_gps_odometry
            elif topic_type == "output_gps_odometry":
                return self.output_gps_odometry
            elif topic_type == "fix_input":
                return self.fix_input
            elif topic_type == "input_onboard_odometry":
                return self.input_onboard_odometry
            elif topic_type == "output_onboard_odometry":
                return self.output_onboard_odometry
            elif topic_type == "output_kalman_filter":
                return self.output_kalman_filter
            else:
                return None    
    
    def get_fix_input(self):
        with self.thread_lock:
            return self.fix_input
        
    def set_fix_input(self, fix_input: str):
        with self.thread_lock:
            self.fix_input = fix_input
            
    def get_calculated_odometry(self):
        with self.thread_lock:
            return self.calculated_odometry
            
    def set_calculated_odometry(self, data: Odometry):
        with self.thread_lock:
            self.calculated_odometry = data
            
    def publish(self, message: str):
        with self.thread_lock:            
            self.publisher.publish(message)
    


# Class for handling the subscribers and publishers
class RelayNode():
    
    def __init__(self, data_handler: DataHandler):
        self.data_handler = data_handler
        
    # Main method for the relay node
    def run(self):
        
        # Initilizes the classes
        gps_relay = GPSRelay(self.data_handler)
        ekf_subscriber = EKFSubscriber(self.data_handler)
        gps_status_subscriber = GPSStatusSubscriber(self.data_handler)
        odometry_relay = OdometryRelay(self.data_handler)
        
        # Starts the threads
        self.start_thread(gps_relay)
        self.start_thread(ekf_subscriber)
        self.start_thread(gps_status_subscriber)
        self.start_thread(odometry_relay)
        
    def start_thread(self, class_object):
        thread_object = threading.Thread(target=class_object.run, args=())
        thread_object.start()
  
        
    
# This class relays the odometry from the GPS
# If the status of the GPS signal is sufficient, the relay node will relay the GPS data to the EKF filter
# When the stignal is not good enough, send the last calculated odometry data.
class GPSRelay():
    
    def __init__(self, data_handler: DataHandler):
        # Initializes the variables needed
        self.data_handler = data_handler
        
        
    def run(self):
        
        input_gps_odometry_ = self.data_handler.get_topic_name("input_gps_odometry")
        output_gps_odometry_ = self.data_handler.get_topic_name("output_gps_odometry")
        
        self.sub = rospy.Subscriber(input_gps_odometry_, Odometry, self.callback)
        self.pub = rospy.Publisher(output_gps_odometry_, Odometry, queue_size=10)
        
        self.data_handler.publish("Odometry from GPS relay subscriber running")
        
        rospy.spin()
    
    def callback(self, data: Odometry):
        
        #Check if the GPS signal is sufficient
        
        # With good GPS signal: Relay the GPS data
        # With bad GPS signal: Send the last calculated odometry data
        if(self.data_handler.get_gps_fix_status() == 2):
            self.pub.publish(data)
        else:
            self.pub.publish(self.data_handler.get_calculated_odometry())
        pass
        
        
# This class takes the odometry data from the ekstended kalman filters and saves it.
class EKFSubscriber():
    
    def __init__(self, data_handler: DataHandler):
        self.data_handler = data_handler
        
    def run(self):
        output_kalman_filter_ = self.data_handler.get_topic_name("output_kalman_filter")
        
        sub = rospy.Subscriber(output_kalman_filter_, Odometry, self.callback)
        
        self.data_handler.publish("EKF subscriber running")
        
        rospy.spin()
        
    def callback(self, data: Odometry):
        self.data_handler.set_ekf_odometry(data)
        
# This class takes the status of the GPS signal and saves it.
class GPSStatusSubscriber():
    
    def __init__(self, data_handler: DataHandler):
        self.data_handler = data_handler
        
    def run(self):
        
        fix_input_ = self.data_handler.get_topic_name("fix_input")
        
        sub = rospy.Subscriber(fix_input_, NavSatFix, self.callback)
        
        self.data_handler.publish("GPS status subscriber running")
        
        rospy.spin()
        
    def callback(self, data: NavSatFix):
        # Saves the status of the GPS signal
        
        self.data_handler.set_gps_fix_status(data.status.status)

# This class takes the odometry data from the husky and relays it to the EKF filter
# If the signal is good enough, the relay node will relay the husky odom data to the EKF filter
# IF the signal is not good enough, the relay node will calculate the difference between the last two odometry measurements and add it to the EKF odometry
# --------------------------------------------------------------------------------------------------------------------------------
# Satus of the GPS signal:
# 1: Satelite based augmentation, Only GNSS
# 2: Ground based augmentation, GNSS + RTK
class OdometryRelay():
    
    def __init__(self, data_handler: DataHandler):
        
        # Initilizes the variables needed
        self.data_handler = data_handler
        self.husky_odometry_t_zero = Odometry()
        self.husky_odometry_t_minus_1 = Odometry()        
        
    # Initializes the odometry subscriber, and spins.    
    def run(self):
        
        # Initilizes the subscribers and publishers
        
        input_onboard_odometry_ = self.data_handler.get_topic_name("input_onboard_odometry")
        output_onboard_odometry_ = self.data_handler.get_topic_name("output_onboard_odometry")
        
        self.sub = rospy.Subscriber(input_onboard_odometry_, Odometry, self.callback)
        self.pub = rospy.Publisher(output_onboard_odometry_, Odometry, queue_size=10)
        
        self.data_handler.publish("Odometry relay subscriber running")
        
        rospy.spin()
    
    
    # Takes the odometry data from the husky and relays it to the EKF filter
    # If the GPS signal is too bad, the relay node will calculate the difference between the last two odometry measurements and add it to the EKF odometry
    # --------------------------------------------------------------------------------------------------------------------------------
    # Satus of the GPS signal:
    # 1: Satelite based augmentation, Only GNSS
    # 2: Ground based augmentation, GNSS + RTK    
    def callback(self, data: Odometry):
        # Update the odometry data
        self.husky_odometry_t_minus_1 = self.husky_odometry_t_zero
        self.husky_odometry_t_zero = data
        
        # Gets the status of the GPS signal
        status = data_handler.get_gps_fix_status()
        
        # Handle the odometry data based on the status of the GPS signal
        if(status == 2):
            self.pub.publish(self.husky_odometry_t_zero) # With GNSS + RTK we have sufficiently accurate measurements
        else:
            # Without good enough GPS signal, we use the odometry data added to the EKF data
            # Caculating the difference in movement from the last two odometry measurements
            delta_x = self.husky_odometry_t_zero.pose.pose.position.x - self.husky_odometry_t_minus_1.pose.pose.position.x
            delta_y = self.husky_odometry_t_zero.pose.pose.position.y - self.husky_odometry_t_minus_1.pose.pose.position.y
            
            # Copies the EKF odometry.
            self.calculated_odometry = self.data_handler.get_ekf_odometry()
            
            # Adds the difference
            self.calculated_odometry.pose.pose.position.x = self.calculated_odometry.pose.pose.position.x + delta_x
            self.calculated_odometry.pose.pose.position.y = self.calculated_odometry.pose.pose.position.y + delta_y
            
            self.pub.publish(self.calculated_odometry)
        
    
     
    
# Main method, starts the relay node, keeps it running, and handles the ros parameters
if __name__ == '__main__':
    
    # Initilizes the node
    rospy.init_node('relay_node_for_ekf_odometry', anonymous=True)
    
    
    # Gets the names for the topics:
    # ALl the afflicted topics:
    # Input for the GPS odometry
    # Output for the GPS odometry
    # Topic from the GPS driver
    # Input for the onboard odometry
    # Output for the onboard odometry
    # The kalman filter
    
    # Input for the GPS odometry
    if rospy.has_param('/odometry/gps'):
        input_gps_odometry = rospy.get_param('/odometry/gps')
    else:
        input_gps_odometry = '/odometry/gps'
        
    # Output for the GPS odometry
    if rospy.has_param('/odometry/gps/filtered'):
        output_gps_odometry = rospy.get_param('/odomery/gps/filtered')
    else:
        output_gps_odometry = '/odometry/gps/filtered'
            
    # Topic from the GPS drivert
    if rospy.has_param('/gps/fix'):
        fix_input = rospy.get_param('/gps/fix')
    else:
        fix_input = '/gps/fix'
        
    # Input for the onboard odometry
    if rospy.has_param('/odometry'):
        input_onboard_odometry = rospy.get_param('/odometry')
    else:
        input_onboard_odometry = '/husky_velocity_controller/odom'
        
    # Output for the onboard odometry
    if rospy.has_param('/odometry/filtered'):
        output_onboard_odometry = rospy.get_param('/odometry/filtered')
    else:
        output_onboard_odometry = '/odometry/filtered'
        
    # The kalman filter topic
    if rospy.has_param('/global_ekf/odometry/filtered'):
        output_kalman_filter = rospy.get_param('/global_ekf/odometry/filtered')
    else:
        output_kalman_filter = '/global_ekf/odometry/filtered' 
    
        
    # Initilizes the data handler
    data_handler = DataHandler(input_gps_odometry, output_gps_odometry, fix_input, input_onboard_odometry, output_onboard_odometry, output_kalman_filter)
    
    # Initilizes the relay node
    relay_node = RelayNode(data_handler)
    
    # Runs the relay node
    relay_node.run()
    
    pub = rospy.Publisher('/debug_relay_node', String, queue_size=10)
    pub.publish("Relay node started")
    
    
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        
        #pub.publish("Relay node running")
        
        r.sleep()
    
    
    # Keep the node running
    rospy.spin()