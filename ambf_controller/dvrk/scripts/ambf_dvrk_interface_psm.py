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
import argparse


class ambf_dvrk_interface_psm:

  TOPIC_NAMESPACE_BASE = "dvrk"

  # Constructor
  def __init__(self, arm_name):

    self.c = Client('ambf_to_ros_psm')
    self.c.connect()
    self.arm_name = arm_name
    time.sleep(2.0)
    print(self.c.get_obj_names())
    self.b = self.c.get_obj_handle('psm/baselink')
    self.target_ik = self.c.get_obj_handle('psm/target_ik')
    self.target_fk = self.c.get_obj_handle('psm/target_fk')
    time.sleep(1.0)

    topic_namespace = "/" + self.TOPIC_NAMESPACE_BASE + "/" + arm_name + "/"
    # Publishers
    self.pos_current_pub = rospy.Publisher(topic_namespace + "position_cartesian_current",
                            PoseStamped, queue_size=10)

    self.jaw_current_pub = rospy.Publisher(topic_namespace + "state_jaw_current",
                            JointState, queue_size=10)

    self.joint_current_pub = rospy.Publisher(topic_namespace + "state_joint_current",
                            JointState, queue_size=10)


    # Subscribers
    self.set_pos_sub = rospy.Subscriber(topic_namespace + "set_position_cartesian",
                            Pose,self.set_pos_callback)

    self.set_joint_sub = rospy.Subscriber(topic_namespace + "set_position_joint",
                            JointState,self.set_joint_callback)

    self.set_jaw_sub = rospy.Subscriber(topic_namespace + "set_position_jaw",
                            JointState,self.set_pos_callback)







  def home(self):
    T_7_0 = Frame(Rotation.Quaternion(0.707107, 0.707107,
            0.0, -0.0),
            Vector(0.0, 0.0, -0.1135))

    self.set_pose(T_7_0)




  def set_pos_callback(self, data):

    T_7_0 = rospose2frame(data)
    self.set_pose(T_7_0)


  def set_joint_callback(self, data):
    print('set joint calback')
    # TODO

  def set_jaw_callback(self, data):
    print('set jaw calback')
    # TODO




  def set_pose(self, T_7_0):

    T_0_w = self.get_T_0_w()

    if self.target_ik is not None:
      T_7_w = T_0_w * convert_mat_to_frame(T_7_0)
      self.target_ik.set_pos(T_7_w.p[0], T_7_w.p[1], T_7_w.p[2])
      self.target_ik.set_rpy(T_7_w.M.GetRPY()[0], T_7_w.M.GetRPY()[1], T_7_w.M.GetRPY()[2])

    computed_q = compute_IK(convert_mat_to_frame(T_7_0))
    #computed_q = enforce_limits(computed_q, joint_lims)

    if self.target_fk is not None:
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

    #time.sleep(1.0)



  def get_T_0_w(self):
    pos = self.b.get_pos()
    rpy = self.b.get_rpy()
    P_0_w = Vector(pos.x, pos.y, pos.z)
    R_0_w = Rotation.RPY(rpy[0], rpy[1], rpy[2])
    T_0_w = Frame(R_0_w, P_0_w)
    return T_0_w

  def pub_current_pos_thread(self):
    rate = rospy.Rate(20) # ROS Rate at 20Hz
    while not rospy.is_shutdown():

      stamp = rospy.Time.now()
      measured_q = self.b.get_all_joint_pos()
      measured_v = self.b.get_all_joint_vel()
      measured_e = self.b.get_all_joint_effort()
      joint_n = len(measured_q)
      joint_names = []
      for i in range(len(measured_q)):
        joint_names.append(self.b.get_joint_name_from_idx(i))

      joint_state_msg = q2rosjointstate(measured_q, measured_v, measured_e, joint_names)
      joint_state_msg.header.stamp = stamp

      T_7_0_fk = compute_FK(measured_q)

      pose_msg = PoseStamped()
      pose_msg.pose = frame2rospose(T_7_0_fk)
      pose_msg.header.stamp = rospy.Time.now()

      self.pos_current_pub.publish(pose_msg)
      self.joint_current_pub.publish(joint_state_msg)
      rate.sleep()



# Utilities
def rospose2frame(p):
  F = Frame(Rotation.Quaternion(p.orientation.x, p.orientation.y,
            p.orientation.z, p.orientation.w),
            Vector(p.position.x, p.position.y, p.position.z))
  return F

def frame2rospose(F):
  p = Pose()
  x,y,z,w = F.M.GetQuaternion()
  p.position.x = F.P.x()
  p.position.y = F.P.y()
  p.position.z = F.P.z()
  p.orientation.x = x
  p.orientation.y = y
  p.orientation.z = z
  p.orientation.w = w
  return p

def q2rosjointstate(q,v,e,names):
  joint_state = JointState()
  joint_state.names = names
  joint_state.position = q
  joint_state.velocity = v
  joint_state.effort = e
  return joint_state


# Main function
def main(args):
  parser = argparse.ArgumentParser()
  parser.add_argument("-a", "--arm", required=True)
  args = parser.parse_args()

  print("Node started")

  relay = ambf_dvrk_interface_psm(args.arm)
  worker = threading.Thread(target=relay.pub_current_pos_thread)
  worker.start()
  #rospy.init_node('ambf_to_ros_psm', anonymous=True)
  relay.home()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == "__main__":
    main(sys.argv)
