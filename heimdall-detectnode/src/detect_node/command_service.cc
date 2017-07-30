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
#include <algorithm>

using namespace std;

namespace Heimdall
{
    bool DetectNode::commandService(heimdall_msgs::CommandSrv::Request &req, heimdall_msgs::CommandSrv::Response &res) {
        ROS_DEBUG("Received command: %s", req.command.c_str());
        string mask_prefix = "mask:remove:";
        if (req.command == "calibrate") {
            ROS_INFO("Switching to calibrate mode.");
            if (! calibrate()) {
                ROS_ERROR("Calibration failed, switching to idle mode.");
                idle();
                return false;
            }
        } else if (req.command == "calibrate:reset") {
            ROS_WARN("Removing calibration data, switching to IDLE mode.");
            d_dd_rgb.reset();
            d_dd_depth.reset();
            d_next_cal_condition = 0;
            idle();
        } else if (req.command == "calibrate:find") {
            ROS_INFO("Trying to refind surfaces for non-floor types.");
            for (auto & mask : d_masks) {
                if (mask.getType() != "floor") {
                    mask.getPclDetection().findSurface(d_latest_points);
                }
            }
        } else if (req.command == "detect:on") {
            ROS_INFO("Switching to detect mode.");
            detect();
        } else if (req.command == "detect:off") {
            ROS_INFO("Switching to idle mode.");
            idle();
        } else if (req.command == "status") {
            switch (d_mode)
            {
                case DetectNodeMode::IDLE:
                    res.result = "{'mode': 'idle'}";
                    break;
                case DetectNodeMode::CALIBRATE:
                    res.result = "{'mode': 'calibrate'}";
                    break;
                case DetectNodeMode::DETECT:
                    res.result = "{'mode': 'detect'}";
                    break;
                default:
                    res.result = "{'mode': null}";
            }
        } else if (req.command == "masks:save") {
            ROS_INFO("Saving mask information.");
            saveMasks();
        } else if (req.command == "masks:load") {
            ROS_INFO("Loading mask information.");
            removeMasks();
            loadMasks();
        } else if (req.command == "masks:remove") {
            ROS_INFO("Removing masks.");
            vector<string> labels;
            for (vector<Mask>::size_type mask_idx = 0; mask_idx < d_masks.size(); ++mask_idx) {
                labels.push_back(d_masks[mask_idx].getLabel());
            }
            for (vector<string>::size_type label_idx = 0; label_idx < labels.size(); ++label_idx) {
                removeMask(labels[label_idx]);
            }
        } else if (equal(mask_prefix.begin(), mask_prefix.end(), req.command.begin())) {
            string mask_name = req.command.substr(mask_prefix.length());
            ROS_INFO("Removing mask %s", mask_name.c_str());
            removeMask(mask_name);
        } else {
            ROS_ERROR("Unknown command received!");
            return false;
        }
        return true;
    }
}
