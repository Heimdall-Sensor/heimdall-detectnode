/*
	Heimdall Detect Node processes RGB and depth data and sent notifications upon deviations.
    Copyright (C) 2017 Christof Oost, Amir Shantia, Ron Snijders, Egbert van der Wal

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "includes.ih"

using namespace std;

namespace Heimdall
{
    void DetectNode::depthCallback(const sensor_msgs::ImageConstPtr& msg) {
        //Check if image encoding corresponds to previous images:
        if (d_depth_encoding == "") {
            d_depth_encoding = msg->encoding;
        }
        if (d_depth_encoding != msg->encoding) {
            ROS_ERROR("Encoding of image changed! Ignoring new image...");
            return;
        }

        //Convert ROS image to CV image:
        cv_bridge::CvImagePtr cv_ptr;
        if (d_depth_encoding == "16UC1" || d_depth_encoding == "32FC1") {
            //Probably a depth image:
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
        } else {
            ROS_ERROR("Specified encoding %s, not supported!", msg->encoding.c_str());
            return;
        } 

        //ROS_DEBUG("depth received: %d", d_depth_count);
        //d_depth_count++;
        
        d_latest_depth_image = cv_ptr->image.clone();
        d_latest_depth_time = ros::Time::now();
    }
}
