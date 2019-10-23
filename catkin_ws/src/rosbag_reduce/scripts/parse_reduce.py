#!/usr/bin/env python

import sys, getopt
import rosbag
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main(argv): 
  # initialization variables
  init = True

  # import/export parameters/objects
  in_bag_name = '/home/haydenm2/bagfiles/data_054_2019-08-20T08_57_14.bag'
  out_bag_name = '/home/haydenm2/bagfiles/reduced.bag'
  img_topic = '/pylon_camera_node/image_raw'
  resize_scale = 0.01

  # terminal argument assignment
  try:
    opts, args = getopt.getopt(argv,"i:o:t:r:",["ifile=","ofile=","topic=","ratio="])
  except getopt.GetoptError:
    print('parse_reduce.py -i <inputbagpath> -o <outputbagpath> -t <imagetopic> -r <reductionratio>')
    sys.exit(2)
  for opt, arg in opts:
      if opt in ("-i", "--ifile"):
         in_bag_name = arg
      elif opt in ("-o", "--ofile"):
         out_bag_name = arg
      elif opt in ("-t", "--topic"):
         img_topic = arg
      elif opt in ("-r", "--ratio"):
         resize_scale = float(arg)

  in_bag = rosbag.Bag(in_bag_name)
  out_bag = rosbag.Bag(out_bag_name, 'w')

  # progress readout parameters
  t_begin = in_bag.get_start_time()
  t_end = in_bag.get_end_time()
  t_total = t_end - t_begin

  # CvBridge init
  bridge = CvBridge()

  # iterate through incoming bag
  for topic, msg, t in in_bag.read_messages(topics=[img_topic]):  
    I = Image()
    I = msg
    enc = I.encoding

    # convert to Cv format for reduction
    try:
      cv_image = bridge.imgmsg_to_cv2(I, enc)
    except CvBridgeError as e:
      print(e)

    # initialize reduced image params
    if(init):
      img_h_reduced = int(cv_image.shape[0] * resize_scale)
      img_w_reduced = int(cv_image.shape[1] * resize_scale)
      reduced_dim = (img_w_reduced, img_h_reduced)

    cv_image_reduced = cv2.resize(cv_image, reduced_dim, interpolation = cv2.INTER_AREA)

    # convert back to ROS format and write to bag
    I_new = bridge.cv2_to_imgmsg(cv_image_reduced)
    out_bag.write(img_topic, I_new, t=t)
    
    # print progress percentage
    print "\r \033[1;31m Percent Complete: \033[0;0m %f %%" % (((t.to_sec()-t_begin)/t_total)*100),
    sys.stdout.flush()
    pass

  # completion printout and bag closures
  print("\n \033[1;32m Rosbag Resolution Reduction Complete! \033[0;0m")
  print("Output bag: %s \n" % out_bag_name)

  out_bag.close()
  in_bag.close()

if __name__ == "__main__":
  main(sys.argv[1:])