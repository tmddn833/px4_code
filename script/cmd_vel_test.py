import rospy
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Header
import numpy as np

import matplotlib.pyplot as plt

class CmdVelTest:
    def __init__(self, test_mode = 'eight_shape'):
        self.cur_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.cur_state_cb)
        self.cur_pose = PoseStamped()
        self.init_pose = PoseStamped()
        self.cmd_PositionTarget_pub = rospy.Publisher('/mav_wrapper/setpoint_planning/setpoint_raw', PositionTarget, queue_size=1)
        self.cmd_tmp_pub = rospy.Publisher('/tmp', PoseStamped, queue_size=1)
        self.cmd_PositionTarget = PositionTarget()
        self.cmd_tmp = PoseStamped()
        self.rate = rospy.Rate(100)

        self.test_mode = test_mode
        self.is_center_set = False
        self.trajectory = None

    def cur_state_cb(self, msg : PoseStamped):
        self.cur_pose = msg
        if not self.is_center_set:
            self.is_center_set = True
            self.init_pose = msg
            center = [msg.pose.position.x, msg.pose.position.y]
            self.set_test_trajectory(center)
            print("Center is set to: ", center)
    
    def set_cmd_vel(self, x, y, yaw, linear_x, linear_y, ax, ay):
        self.cmd_PositionTarget.header = Header()
        self.cmd_PositionTarget.header.stamp = rospy.Time.now()
        self.cmd_PositionTarget.header.frame_id = self.init_pose.header.frame_id # 'map' , 별로 안중요함
        self.cmd_PositionTarget.coordinate_frame = PositionTarget.FRAME_LOCAL_NED # TODO 이게 뭐지????

        # self.cmd_PositionTarget.type_mask = 32 + 64 + 128 + 256 + 512 + 2048
        # self.cmd_PositionTarget.type_mask = \
        #     PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + \
        #     PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + \
        #     PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
        #     PositionTarget.FORCE + PositionTarget.IGNORE_YAW_RATE
        self.cmd_PositionTarget.type_mask = \
            PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_YAW_RATE
        # self.cmd_PositionTarget.type_mask =0
        self.cmd_PositionTarget.position.x = x
        self.cmd_PositionTarget.position.y = y
        self.cmd_PositionTarget.position.z = self.init_pose.pose.position.z
        self.cmd_PositionTarget.velocity.x = linear_x
        self.cmd_PositionTarget.velocity.y = linear_y
        self.cmd_PositionTarget.velocity.z = 0
        self.cmd_PositionTarget.acceleration_or_force.x = ax
        self.cmd_PositionTarget.acceleration_or_force.y = ay
        self.cmd_PositionTarget.acceleration_or_force.z = 0
        self.cmd_PositionTarget.yaw = yaw

        self.cmd_tmp.header = self.cmd_PositionTarget.header
        self.cmd_tmp.pose.position.x = x
        self.cmd_tmp.pose.position.y = y
        self.cmd_tmp.pose.position.z = self.init_pose.pose.position.z
        # yaw to quaternion
        x, y, z, w = 0, 0, np.sin(yaw/2), np.cos(yaw/2)
        self.cmd_tmp.pose.orientation.x = x
        self.cmd_tmp.pose.orientation.y = y
        self.cmd_tmp.pose.orientation.z = z
        self.cmd_tmp.pose.orientation.w = w

        self.cmd_PositionTarget_pub.publish(self.cmd_PositionTarget)
        self.cmd_tmp_pub.publish(self.cmd_tmp)
        
    def run(self):
        while not rospy.is_shutdown():
            if self.trajectory is not None:
                x, y, vx, vy, ax, ay = self.trajectory.get_cur_pose_cmd_vel(rospy.Time.now().to_sec())
                self.set_cmd_vel(x, y, 0, vx, vy, ax, ay)
                # print("x: {}, y: {}, vx: {}, vy: {}".format(x, y, vx, vy))
            self.rate.sleep()

    def set_test_trajectory(self, center):
        if self.test_mode == 'eight_shape':
            self.trajectory = EightShapeTrajectory(1, 1, rospy.Time.now().to_sec(), center)
        
        elif self.test_mode == 'circle':
            self.trajectory = CircleTrajectory(1, 1, rospy.Time.now().to_sec(), center)
        else:
            rospy.logerr('Invalid test mode: {}'.format(self.test_mode))

class Trajectory:
    def __init__(self, maximum_speed, maximum_acceleration, start_time, center):
        self.maximum_speed = maximum_speed
        self.maximum_acceleration = maximum_acceleration
        self.start_time = start_time
        self.center = center

    def get_pose_cmd_vel(self, dt):
        pass

    def get_cur_pose_cmd_vel(self, t):
        return self.get_pose_cmd_vel(t - self.start_time)

class EightShapeTrajectory(Trajectory):
    def __init__(self, maximum_speed, maximum_acceleration, start_time, center):
        super(EightShapeTrajectory, self).__init__(maximum_speed, maximum_acceleration, start_time, center)
        self.a = 1
        self.b = 1
        self.omega = 0.5
        self.start_time = start_time
        max_vel = self.omega * np.sqrt(self.a**2 + 4*self.b**2)
        if max_vel > self.maximum_speed:
            print("max limit exceed!!!")
            self.omega = self.maximum_speed / max_vel * self.omega

    def get_pose_cmd_vel(self, t):
        x = self.a * np.cos(self.omega * t ) + self.center[0]
        y = self.b * np.sin(2*self.omega * t ) + self.center[1]
        vx = -self.a * self.omega * np.sin(self.omega * t)
        vy = 2*self.b * self.omega * np.cos(2*self.omega * t)
        ax = -self.a * self.omega**2 * np.cos(self.omega * t)
        ay = -4*self.b * self.omega**2 * np.sin(2*self.omega * t)
        return x, y, vx, vy, ax, ay

class CircleTrajectory(Trajectory):
    def __init__(self, maximum_speed, maximum_acceleration, start_time, center):
        super(CircleTrajectory, self).__init__(maximum_speed, maximum_acceleration, start_time, center)
        self.radius = 1
        self.omega = 1
        self.start_time = start_time
        max_vel = self.omega * self.radius
        if max_vel > self.maximum_speed:
            print("max limit exceed!!!")
            self.omega = self.maximum_speed / max_vel * self.omega

    def get_pose_cmd_vel(self, t):
        x = self.radius * np.cos(self.omega * t) + self.center[0]
        y = self.radius * np.sin(self.omega * t) + self.center[1]
        vx = -self.radius * self.omega * np.sin(self.omega * t)
        vy = self.radius * self.omega * np.cos(self.omega * t)
        ax = -self.radius * self.omega**2 * np.cos(self.omega * t)
        ay = -self.radius * self.omega**2 * np.sin(self.omega * t)
        return x, y, vx, vy, ax, ay

if __name__ == '__main__':
    rospy.init_node('planner_client')
    cmd_vel_test = CmdVelTest()
    cmd_vel_test.run()

    
