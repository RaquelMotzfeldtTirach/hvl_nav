import rospy
import actionlib
import time

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

class MoveBaseClient:
    
    def __init__(self, publisher: rospy.Publisher=rospy.Publisher('/move_base_client', String, queue_size=1)):
        self.publisher = publisher
        self.publisher.publish('MoveBaseClient: Initialized')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.publisher.publish('MoveBaseClient: Waiting for move_base action server')
        self.client.wait_for_server()
        self.publisher.publish('MoveBaseClient: Connected to move_base action server')
        
        
    # This function follows a path of points by using the ROS action server move_base from the navigation stack    
    def follow_path(self, path: list):
        
        self.publisher.publish('MoveBaseClient: Following path')
        self.publisher.publish('MoveBaseClient: Path length: ' + str(len(path)))
        
        for goal in path:
            
            self.publisher.publish('MoveBaseClient: Going to point')
            self.publisher.publish('MoveBaseClient: Point: ' + str(goal.target_pose.pose.position.x) + ', ' + str(goal.target_pose.pose.position.y))
            
            self.client.send_goal(goal)
            
            self.publisher.publish('MoveBaseClient: Waiting for result')
            
            self.client.wait_for_result()
            
            self.publisher.publish('MoveBaseClient: Target reached')
    
    
    # This function goes to a point by using the ROS action server move_base from the navigation stack        
    def move_to_point(self, point: MoveBaseGoal):
        
        self.publisher.publish('MoveBaseClient: Going to point')
        
        self.publisher.publish('MoveBaseClient: Point: ' + str(point.target_pose.pose.position.x) + ', ' + str(point.target_pose.pose.position.y))
        
        self.client.send_goal(point)
        
        self.publisher.publish('MoveBaseClient: Waiting for result')
        
        self.client.wait_for_result()
        
        self.publisher.publish('MoveBaseClient: Target reached')
        
        time.sleep(0.1)
        
    def testing(self):
        print("testing")