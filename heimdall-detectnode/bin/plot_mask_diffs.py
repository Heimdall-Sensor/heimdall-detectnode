#!/usr/bin/env python

import rospy
import subprocess


def main():
    #Lookup relevant ROS topics:
    mask_topics = rospy.get_published_topics("/mask")
    diff_topics = filter(lambda a: a[0].split("/")[-1] == "diff", mask_topics)
    #Append to rqt_plot and execute:
    subprocess.call(["rosrun", "rqt_plot", "rqt_plot"] + [d[0] for d in diff_topics])


if __name__ == "__main__":
    main()
