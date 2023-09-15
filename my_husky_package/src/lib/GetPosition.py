import rospy
import time

from sensor_msgs.msg import NavSatFix


# Class for getting the current position of the robot.
class GetPosition:
    
    def __init__(self):
        self.sub = rospy.Subscriber("/emlid/fix", NavSatFix, self.gps_callback)
        self.gps_point = None
        self.recieved_data = False
        #rospy.init_node('get_position', anonymous=True)
        
    def gps_callback(self, data: NavSatFix):
        self.gps_point = (data.latitude, data.longitude)
        self.recieved_data = True
        self.sub.unregister()
        
    #Blocking function
    def get_gps_point(self):
        #rospy.spin() #The node runs until it recieves data
        while not self.recieved_data:
            #Wait for data
            time.sleep(0.1)
        return self.gps_point