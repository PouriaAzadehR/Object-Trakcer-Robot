#!/usr/bin/python3

# ROS
import rospy
from geometry_msgs.msg import Twist
from turtlebot3_object_tracker.srv import detection


class Controller:
    def __init__(self) -> None:
        self.e_l = 0.08
        # Use these Twists to control your robot
        self.move = Twist()
        self.move.linear.x = 0.1
        self.freeze = Twist()

        # The "p" parameter for your p-controller, TODO: you need to tune this
        self.angular_vel_coef = 0.005

        # TODO: Create a service proxy for your human detection service
        
        # TODO: Create a publisher for your robot "cmd_vel"
        self.cmd_publisher = rospy.Publisher('/follower/cmd_vel' , Twist , queue_size=2)


    def recieve_detection(self,label):
        rospy.wait_for_service('/detection')
        resp = rospy.ServiceProxy('/detection', detection)(label)
        self.exists = resp.exists
        self.pos_x_in_pic = resp.pos_x_in_pic 
        self.pos_y_in_pic = resp.pos_y_in_pic
        self.bb_width = resp.bb_width
        self.bb_height = resp.bb_height
        self.pic_width = resp.pic_width
        self.pic_height = resp.pic_height
        return

    def angular_error(self):
        center_x = self.pos_x_in_pic - self.bb_width/2
        error = self.pic_width/2 - center_x
        return error
    

    def is_close(self):
        a = (self.bb_height * self.bb_width) / (self.pic_height * self.pic_width)
        if a >= self.e_l:
            return True
        else:
            return False
        
    
    
    def run(self) -> None:
        try:
            while not rospy.is_shutdown():
                
                # TODO: Call your service, ride your robot
                self.recieve_detection('person')
                if self.exists is False:
                    self.freeze.angular.z = 0
                    self.cmd_publisher.publish(self.freeze)

                else:
                    is_close = self.is_close()
                    if is_close:
                        twist = self.freeze
                    else:

                        twist = self.move

                    angualr_error = self.angular_error()
                    P = angualr_error * self.angular_vel_coef
                    twist.angular.z = P
                    self.cmd_publisher.publish(twist)
                

        except rospy.exceptions.ROSInterruptException:
            pass
                

if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    
    controller = Controller()
    controller.run()
    

