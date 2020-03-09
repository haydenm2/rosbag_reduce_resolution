#!/usr/bin/env python

import sys, getopt
import rosbag
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def main(argv):
    # initialization variables
    init = True
    show = False
    invert = False

    # input argument parsing
    try:
        opts, args = getopt.getopt(argv, "i:o:t:c:", ["ifile=", "ofile=", "topic=", "contrast="])
    except getopt.GetoptError:
        print('parse_recontrast.py -i <inputbagpath> -o <outputbagpath> -t <imagetopic> -c <contrast>')
        sys.exit(2)
    for opt, arg in opts:
        if opt in ("-i", "--ifile"):
            in_bag_name = arg
        elif opt in ("-o", "--ofile"):
            out_bag_name = arg
        elif opt in ("-t", "--topic"):
            img_topic = arg
        elif opt in ("-c", "--contrast"):
            contrast = float(arg)

    # validate input arguments -------------------------------------------------
    try:
        in_bag_name
    except NameError:
        print("\033[1;31m ERROR: Must define input bag path! \033[0;0m")
        print('parse_recontrast.py -i <inputbagpath> -o <outputbagpath> -t <imagetopic> -c <contrast>')
        sys.exit(2)
    else:
        print("\033[1;33m Input Bag: \033[0;0m %s" % in_bag_name)

    try:
        out_bag_name
    except NameError:
        print("\033[1;31m ERROR: Must define output bag path! \033[0;0m")
        print('parse_recontrast.py -i <inputbagpath> -o <outputbagpath> -t <imagetopic> -c <contrast>')
        sys.exit(2)
    else:
        print("\033[1;33m Output Bag: \033[0;0m %s" % out_bag_name)

    try:
        img_topic
    except NameError:
        print("\033[1;31m ERROR: Must define image topic! \033[0;0m")
        print('parse_recontrast.py -i <inputbagpath> -o <outputbagpath> -t <imagetopic> -c <contrast>')
        sys.exit(2)
    else:
        print("\033[1;33m Image Topic: \033[0;0m %s" % img_topic)

    try:
        contrast
    except NameError:
        print("\033[1;31m ERROR: Must define desired contrast ratio! \033[0;0m")
        print('parse_recontrast.py -i <inputbagpath> -o <outputbagpath> -t <imagetopic> -c <contrast>')
        sys.exit(2)
    else:
        print("\033[1;33m Contrast Ratio: \033[0;0m %s" % contrast)

    if (in_bag_name[-4:] != ".bag"):
        print("\033[1;31m ERROR: Input bag path must be in .bag format \033[0;0m")
        sys.exit(2)

    if (out_bag_name[-4:] != ".bag"):
        print("\033[1;31m ERROR: Output bag path must be in .bag format \033[0;0m")
        sys.exit(2)

    # --------------------------------------------------------------------------

    in_bag = rosbag.Bag(in_bag_name)
    out_bag = rosbag.Bag(out_bag_name, 'w')

    # progress readout parameters
    t_begin = in_bag.get_start_time()
    t_end = in_bag.get_end_time()
    t_total = t_end - t_begin

    # CvBridge init
    bridge = CvBridge()

    test_cont = contrast

    # iterate through incoming bag
    for topic, msg, t in in_bag.read_messages(topics=[img_topic]):
        I = msg
        enc = I.encoding
        try:
            cv_image_t = bridge.imgmsg_to_cv2(I, enc)
        except CvBridgeError as e:
            print(e)
        while init:
            print("Contrast: ", test_cont)
            test_img = cv_image_t.copy()
            cv2.convertScaleAbs(cv_image_t, test_img, test_cont)
            if invert:
                test_img = np.uint8(255-test_img)
            else:
                test_img = np.uint8(test_img)
            print(test_img)
            cv2.imshow('image', test_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            resp = str(raw_input("Is contrast sufficient? (N/Y) "))
            if resp == 'Y':
                init = False
                contrast = test_cont
                cv2.destroyAllWindows()
                break
            else:
                test_cont = float(raw_input("Input Desired Contrast Value: "))

        if not show and not invert:
            if (contrast == 1.0):
                out_bag.write(img_topic, msg, t=t)

                # print progress percentage
                print "\r \033[1;31m Percent Complete: \033[0;0m %f %%" % (((t.to_sec() - t_begin) / t_total) * 100),
                sys.stdout.flush()
                continue

        # convert to Cv format for reduction
        try:
            cv_image = bridge.imgmsg_to_cv2(I, enc)
        except CvBridgeError as e:
            print(e)
        if invert:
            cv_image_contrasted = np.uint8(255-cv_image*contrast)
        else:
            cv_image_contrasted = np.uint8(cv_image * contrast)

        if show:
            cv2.imshow('image', cv_image_contrasted)
            cv2.waitKey(1)

        # convert back to ROS format and write to bag
        I_new = bridge.cv2_to_imgmsg(cv_image_contrasted, enc)
        out_bag.write(img_topic, I_new, t=t)

        # print progress percentage
        print "\r \033[1;31m Percent Complete: \033[0;0m %f %%" % (((t.to_sec() - t_begin) / t_total) * 100),
        sys.stdout.flush()

    # completion printout and bag closures
    print("\n \033[1;32m Rosbag Resolution Reduction Complete! \033[0;0m")
    print("Output bag: %s \n" % out_bag_name)

    out_bag.close()
    in_bag.close()


if __name__ == "__main__":
    main(sys.argv[1:])