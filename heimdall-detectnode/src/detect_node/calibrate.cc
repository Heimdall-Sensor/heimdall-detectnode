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
    bool DetectNode::calibrate(void) {
        //Check mask sizes (sometimes it is set to 0 for some reason):
        for (vector<Mask>::size_type mask_idx = 0; mask_idx < d_masks.size(); mask_idx++) {
            const Mat & mask = d_masks[mask_idx].getMask();
            if (mask.rows * mask.cols < 1) {
                ROS_ERROR("Found mask of size 0!");
                return false;
            }
        }

        //Check RGB:
        if (d_enable_rgb) {
            if (d_latest_rgb_time.toSec() < 0.00001) {
                ROS_ERROR("Can't calibrate; no RGB input image received yet!");
                return false;
            }
            if ((ros::Time::now().toSec() - d_latest_rgb_time.toSec()) > 1.5) {
                ROS_ERROR("Can't calibrate; latest RGB input image is too old!");
                return false;
            }
        }
        //Check Depth:
        if (d_enable_depth) {
            if (d_latest_depth_time.toSec() < 0.00001) {
                ROS_ERROR("Can't calibrate; no Depth input image received yet!");
                return false;
            }
            if ((ros::Time::now().toSec() - d_latest_depth_time.toSec()) > 1.5) {
                ROS_ERROR("Can't calibrate; latest Depth input image is too old!");
                return false;
            }
        }
        //Check Points:
        if (d_enable_pcl) {
            if (d_latest_points_time.toSec() < 0.00001) {
                ROS_ERROR("Can't calibrate; no Pointcloud received yet!");
                return false;
            }
            if ((ros::Time::now().toSec() - d_latest_points_time.toSec()) > 1.5) {
                ROS_ERROR("Can't calibrate; latest Pointcloud is too old!");
                return false;
            }
        }

        d_mode = DetectNodeMode::CALIBRATE;

        d_calibration_start_time = ros::Time::now();

        if (d_enable_pcl) {
            //Not required for PCL detection since calibration only uses one cloud (for now):
            //d_last_cal_points_time = ros::Time::now();
        }
        if (d_enable_rgb) {
            d_dd_rgb.setConditionSize(d_dd_rgb.getConditionSize() + 1);
            d_last_cal_rgb_time = ros::Time::now();
        }
        if (d_enable_depth) {
            d_dd_depth.setConditionSize(d_dd_depth.getConditionSize() + 1);
            d_last_cal_depth_time = ros::Time::now();
        }

        return true;
    }
}
