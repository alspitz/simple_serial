import rospy
from simple_serial.msg import FileRequest

if __name__ == "__main__":
  rospy.init_node("file_request_test")

  fr_pub = rospy.Publisher("filerequest", FileRequest, queue_size=1)

  rospy.sleep(1)

  fr_msg = FileRequest()
  #fr_msg.filename.data = "/fs/microsd/testlog"
  #fr_msg.filename.data = "/fs/microsd/etc/rc.txt"
  fr_msg.filename.data = "/fs/microsd/log0004"

  fr_pub.publish(fr_msg)

  rospy.spin()
