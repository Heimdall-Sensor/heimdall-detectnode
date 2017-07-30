#!/usr/bin/env python

import rospy
import requests
import json

from std_msgs.msg import String


def callback(data):
    rospy.loginfo("Got: %s" % str(data.data))
    notification = json.loads(data.data)
    r = requests.post('http://89.188.14.162:8181/', data = {
            'source':'enacer',
            'message':notification["message"]
            })
    rospy.loginfo("Got response: %s" % str(r))
    

def main():
    rospy.init_node('cloud_notifier', anonymous=True)

    rospy.Subscriber("/rv/notification", String, callback)

    rospy.spin()


if __name__ == '__main__':
    main()
