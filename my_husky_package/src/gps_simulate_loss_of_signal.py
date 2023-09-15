#!/usr/bin/env python3

import rospy
import utm
import random

from sensor_msgs.msg import NavSatFix

class GpsSignalLossSimulator:
    
    def __init__(self):
        
        #Gps signal
        self.gps_signal = NavSatFix()
        
        #Subscriber for getting the gps signal
        self.sub = rospy.Subscriber('/emlid/fix', NavSatFix, self.gps_callback)
        
        #Publisher for publishing the gps signal
        self.pub = rospy.Publisher('/emlid/fix/simulated', NavSatFix, queue_size=1)
        
        #Timer
        self.time_start = rospy.Time.now()
        self.time_intervall_short = rospy.Duration(30)
        self.time_intervall_long = rospy.Duration(40)
        
        #Flag for shunting the gps signal
        self.shunted = False
        self.shunted_lat = 0
        self.shunted_lot = 0
        
        self.shunted_value = 1
        
        
    def gps_callback(self, msg: NavSatFix):
        
        self.gps_signal = msg
        
        #Get latitude and logitude
        lat = msg.latitude
        lon = msg.longitude
        
        #Gets the curent time
        time_now = rospy.Time.now()
        
        #Prints the time
        rospy.loginfo("Time: " + str(time_now - self.time_start))
        
        if((time_now - self.time_start) > self.time_intervall_short):
            # Shunt the gps signal
            
            # Get the UTM coordinates of the GPS signal
            lat_utm, lon_utm, _, _ = utm.from_latlon(lat, lon)
            
            # Gets the shunted values once
            if (self.shunted == False):
                self.shunted_lat = self.get_random_number()
                self.shunted_lot = self.get_random_number()
                self.shunted = True
            # Shunts the UTM coordinates
            lat_utm = lat_utm + self.shunted_lat
            lon_utm = lon_utm + self.shunted_lot
            
            # Convert the UTM coordinates back to GPS coordinates
            lat_gps, lon_gps = utm.to_latlon(lat_utm, lon_utm, 32, 'U')
            
            self.gps_signal.latitude = lat_gps
            self.gps_signal.longitude = lon_gps
            self.gps_signal.status.status = 1
            
            self.pub.publish(self.gps_signal)
            
            if((time_now - self.time_start) > self.time_intervall_long):
                # Resets the timer
                self.time_start = rospy.Time.now()
                self.shunted = False
        else:
            # Publish the gps signal as it is
            self.gps_signal.status.status = 2
            self.pub.publish(self.gps_signal)
            
            
    def get_random_number(self):
        #Get random number between 1 and 2, both positive and negative
        random_float = random.uniform(0, self.shunted_value)
        
        random_range = random_float * 2
        
        random_number = random_range - self.shunted_value
        
        return random_number
        
        
if __name__ == "__main__":
    
    rospy.init_node('gps_simulate_loss_of_signal')
    
    gps_signal_loss_simulator = GpsSignalLossSimulator()
    
    rospy.spin()