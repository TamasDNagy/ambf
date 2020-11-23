# This Python file uses the following encoding: utf-8

# if__name__ == "__main__":
#     pass
from PyKDL import Vector, Rotation, Frame, dot
from psmIK import *
from ambf_client import Client
import time
import rospy
import roslib
import sys
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from joint_space_trajectory_generator import JointSpaceTrajectory
import threading


class ambf_to_ros_psm:

  # Constructor
  def __init__(self):

    self.c = Client('ambf_to_ros_psm')
    self.c.connect()
    time.sleep(2.0)
    print(self.c.get_obj_names())
    self.b = self.c.get_obj_handle('psm/baselink')
    self.target_ik = self.c.get_obj_handle('psm/target_ik')
    self.target_fk = self.c.get_obj_handle('psm/target_fk')
    time.sleep(1.0)

    self.pos_current_pub = rospy.Publisher("/dvrk/PSM1/position_cartesian_current",PoseStamped, queue_size=10)
    self.set_pos_sub = rospy.Subscriber("/dvrk/PSM1/set_position_cartesian",Pose,self.set_pos_callback)

  def home(self):
    # The following are the names of the controllable joints.
    #  'baselink-yawlink', 0
    #  'yawlink-pitchbacklink', 1
    #  'pitchendlink-maininsertionlink', 2
    #  'maininsertionlink-toolrolllink', 3
    #  'toolrolllink-toolpitchlink', 4
    #  'toolpitchlink-toolgripper1link', 5a
    #  'toolpitchlink-toolgripper2link', 5b

    T_7_0 = Frame(Rotation.Quaternion(0.707107, 0.707107,
            0.0, -0.0),
            Vector(0.0, 0.0, -0.1135))

    if self.target_ik is not None:
      P_0_w = Vector(self.b.get_pos().x, self.b.get_pos().y, self.b.get_pos().z)
      R_0_w = Rotation.RPY(self.b.get_rpy()[0], self.b.get_rpy()[1], self.b.get_rpy()[2])
      T_0_w = Frame(R_0_w, P_0_w)
      T_7_w = T_0_w * convert_mat_to_frame(T_7_0)
      self.target_ik.set_pos(T_7_w.p[0], T_7_w.p[1], T_7_w.p[2])
      self.target_ik.set_rpy(T_7_w.M.GetRPY()[0], T_7_w.M.GetRPY()[1], T_7_w.M.GetRPY()[2])

    computed_q = compute_IK(convert_mat_to_frame(T_7_0))
        # computed_q = enforce_limits(computed_q, joint_lims)

    if self.target_fk is not None:
      P_0_w = Vector(self.b.get_pos().x, self.b.get_pos().y, self.b.get_pos().z)
      R_0_w = Rotation.RPY(self.b.get_rpy()[0], self.b.get_rpy()[1], self.b.get_rpy()[2])
      T_0_w = Frame(R_0_w, P_0_w)
      computed_q.append(0)
      T_7_0_fk = compute_FK(computed_q)
      T_7_w = T_0_w * convert_mat_to_frame(T_7_0_fk)
      self.target_fk.set_pos(T_7_w.p[0], T_7_w.p[1], T_7_w.p[2])
      self.target_fk.set_rpy(T_7_w.M.GetRPY()[0], T_7_w.M.GetRPY()[1], T_7_w.M.GetRPY()[2])

    print('SETTING JOINTS: ')
    print(computed_q)

    self.b.set_joint_pos('baselink-yawlink', computed_q[0])
    self.b.set_joint_pos('yawlink-pitchbacklink', computed_q[1])
    self.b.set_joint_pos('pitchendlink-maininsertionlink', computed_q[2])
    self.b.set_joint_pos('maininsertionlink-toolrolllink', computed_q[3])
    self.b.set_joint_pos('toolrolllink-toolpitchlink', computed_q[4])
    self.b.set_joint_pos('toolpitchlink-toolgripper1link', computed_q[5])
    self.b.set_joint_pos('toolpitchlink-toolgripper2link', -computed_q[5])

    time.sleep(1.0)

  def set_pos_callback(self, data):
    # The following are the names of the controllable joints.
    #  'baselink-yawlink', 0
    #  'yawlink-pitchbacklink', 1
    #  'pitchendlink-maininsertionlink', 2
    #  'maininsertionlink-toolrolllink', 3
    #  'toolrolllink-toolpitchlink', 4
    #  'toolpitchlink-toolgripper1link', 5a
    #  'toolpitchlink-toolgripper2link', 5b

    T_7_0 = Frame(Rotation.Quaternion(data.orientation.x, data.orientation.y,
              data.orientation.z, data.orientation.w),
              Vector(data.position.x, data.position.y, data.position.z))

    if self.target_ik is not None:
      P_0_w = Vector(self.b.get_pos().x, self.b.get_pos().y, self.b.get_pos().z)
      R_0_w = Rotation.RPY(self.b.get_rpy()[0], self.b.get_rpy()[1], self.b.get_rpy()[2])
      T_0_w = Frame(R_0_w, P_0_w)
      T_7_w = T_0_w * convert_mat_to_frame(T_7_0)
      self.target_ik.set_pos(T_7_w.p[0], T_7_w.p[1], T_7_w.p[2])
      self.target_ik.set_rpy(T_7_w.M.GetRPY()[0], T_7_w.M.GetRPY()[1], T_7_w.M.GetRPY()[2])

    computed_q = compute_IK(convert_mat_to_frame(T_7_0))
          # computed_q = enforce_limits(computed_q, joint_lims)

    if self.target_fk is not None:
      P_0_w = Vector(self.b.get_pos().x, self.b.get_pos().y, self.b.get_pos().z)
      R_0_w = Rotation.RPY(self.b.get_rpy()[0], self.b.get_rpy()[1], self.b.get_rpy()[2])
      T_0_w = Frame(R_0_w, P_0_w)
      computed_q.append(0)
      T_7_0_fk = compute_FK(computed_q)
      T_7_w = T_0_w * convert_mat_to_frame(T_7_0_fk)
      self.target_fk.set_pos(T_7_w.p[0], T_7_w.p[1], T_7_w.p[2])
      self.target_fk.set_rpy(T_7_w.M.GetRPY()[0], T_7_w.M.GetRPY()[1], T_7_w.M.GetRPY()[2])

    print('SETTING JOINTS: ')
    print(computed_q)

    self.b.set_joint_pos('baselink-yawlink', computed_q[0])
    self.b.set_joint_pos('yawlink-pitchbacklink', computed_q[1])
    self.b.set_joint_pos('pitchendlink-maininsertionlink', computed_q[2])
    self.b.set_joint_pos('maininsertionlink-toolrolllink', computed_q[3])
    self.b.set_joint_pos('toolrolllink-toolpitchlink', computed_q[4])
    self.b.set_joint_pos('toolpitchlink-toolgripper1link', computed_q[5])
    self.b.set_joint_pos('toolpitchlink-toolgripper2link', -computed_q[5])

    time.sleep(1.0)

  def pub_current_pos_thread(self):
    rate = rospy.Rate(5) # ROS Rate at 5Hz
    while not rospy.is_shutdown():
      P_0_w = Vector(self.b.get_pos().x, self.b.get_pos().y, self.b.get_pos().z)
      R_0_w = Rotation.RPY(self.b.get_rpy()[0], self.b.get_rpy()[1], self.b.get_rpy()[2])
      T_0_w = Frame(R_0_w, P_0_w)
      msg = PoseStamped()
      msg.pose.position.x = self.b.get_pos().x
      msg.pose.position.y = self.b.get_pos().y
      msg.pose.position.z = self.b.get_pos().z
      x,y,z,w = T_0_w.M.GetQuaternion()
      msg.pose.orientation.x = x
      msg.pose.orientation.y = y
      msg.pose.orientation.z = z
      msg.pose.orientation.w = w
      msg.header.stamp = rospy.Time.now()

      self.pos_current_pub.publish(msg)
      rate.sleep()

# Main function
def main(args):
  print("Node started")
  #help(cv2.aruco)

  relay = ambf_to_ros_psm()
  worker = threading.Thread(target=relay.pub_current_pos_thread)
  worker.start()
  #rospy.init_node('ambf_to_ros_psm', anonymous=True)
  relay.home()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == "__main__":
    # test_ik()
    main(sys.argv)
