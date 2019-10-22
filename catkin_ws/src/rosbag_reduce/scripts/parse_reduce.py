#!/usr/bin/env python3

import sys
import rosbag
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
from cv_bridge import CvBridge, CvBridgeError

in_bag_name = '/home/haydenm2/bagfiles/data_054_2019-08-20T08_57_14.bag'
out_bag_name = '/home/haydenm2/bagfiles/reduced.bag'

img_topic = '/pylon_camera_node/image_raw'

in_bag = rosbag.Bag(in_bag_name)
out_bag = rosbag.Bag(out_bag_name, 'w')

t_begin = in_bag.get_start_time()
t_end = in_bag.get_end_time()
t_total = t_end - t_begin

for topic, msg, t in in_bag.read_messages(topics=[img_topic]):  
  I = Image()
  I = msg

  out_bag.write(img_topic, I, t=t)
    
  # print("MSG: ", msg)
  # print("TOPIC: ", topic)
  # print("T: ", t)
  
  print(f"Percent Complete: {(((t.to_sec()-t_begin)/t_total)*100):.2f}%\t\t", end='\r')
  sys.stdout.flush()
  pass

print("Rosbag Resolution Reduction Complete!")
print("Output bag: ", out_bag_name)

out_bag.close()
in_bag.close()

# try:
#     c = Clock()
#     c.data = 


#     I = Image()
#     I.data = 

#     out_bag.write('chatter', s)
# finally:
#     bag.close()