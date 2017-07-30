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
    double DetectNode::detectActivity(Mat & img_change, Mat & dev_depth_image) {
        double current_time = ros::Time::now().toSec();

        heimdall_msgs::ActivityArrayStamped activity_array_msg;

        //Calculate activity in each mask and add as new timed entry:
        double mean_activity = 0.0;
        for (vector<Mask>::size_type mask_idx = 0; mask_idx < d_masks.size(); mask_idx++) {
            double activity_value, border_activity_value;
            //d_masks[mask_idx].getDdDetection().detectActivity(img_change, activity_value, border_activity_value, current_time, dev_depth_image);

            heimdall_msgs::Activity activity;
            activity.mask = d_masks[mask_idx].getLabel();
            activity.activity = activity_value;
            activity.border_activity = border_activity_value;

            activity_array_msg.activities.push_back(activity);

            mean_activity += activity_value;
        }
        mean_activity /= d_masks.size();

        //Publish activity in mask and borders:
        activity_array_msg.header.seq = d_activity_header_seq; d_activity_header_seq++;
        ros::Time time = ros::Time::now();
        activity_array_msg.header.stamp.sec = time.sec;
        activity_array_msg.header.stamp.nsec = time.nsec;
        activity_array_msg.header.frame_id = "0";
        d_activities_pub.publish(activity_array_msg);

        return mean_activity;
    }
}
