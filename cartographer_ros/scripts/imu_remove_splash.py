#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Imu


def main():
  rospy.init_node('imu_remove_splash')
  publisher = rospy.Publisher('/imu_no_splash', Imu, queue_size=1)
  #remove_frames = rospy.get_param('~remove_frames', [])

  def callback(msg):
    print msg.header.frame_id
    if msg.header.frame_id[0] is '/':
        # remove '/'
        msg.header.frame_id = msg.header.frame_id[1:]
    publisher.publish(msg)

  rospy.Subscriber('/imu/data', Imu, callback)
  rospy.spin()


if __name__ == '__main__':
  main()