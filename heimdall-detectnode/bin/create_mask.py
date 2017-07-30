#!/usr/bin/env python

import sys
import subprocess
import rospy

from heimdall_msgs.srv import AddMaskSrv

if __name__ == "__main__":
    rospy.init_node('heimdall_mask_add_test')

    height = 240
    width = 320

    mask = []

    if not len(sys.argv) == 2:
        print "ERROR: No argument specified!"
        exit()
    for y in range(height):
        for x in range(width):
            if sys.argv[1] == "top":
                if y < width / 2:
                    mask.append(1)
                else:
                    mask.append(0)
            elif sys.argv[1] == "bottom":
                if y < width / 2:
                    mask.append(0)
                else:
                    mask.append(1)
            elif sys.argv[1] == "left":
                if x < height / 2:
                    mask.append(1)
                else:
                    mask.append(0)
            elif sys.argv[1] == "right":
                if x < height / 2:
                    mask.append(0)
                else:
                    mask.append(1)
            elif sys.argv[1] == "middle":
                if y > (height / 4) and y < ((height / 4) * 3) \
                        and \
                        x > (width / 4) and x < ((width / 4) * 3):
                    mask.append(1)
                else:
                    mask.append(0)
            else:
                print "ERROR: Unknown argument \"%s\"!" % sys.argv[1]
                exit()

    rospy.wait_for_service('/heimdall/add_mask', timeout = 3)
    service = rospy.ServiceProxy('/heimdall/add_mask', AddMaskSrv)
    service(sys.argv[1], width, height, mask)

