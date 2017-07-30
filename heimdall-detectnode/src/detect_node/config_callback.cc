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

    void DetectNode::setMasksConfig(heimdall_detectnode::DetectNodeConfig &config) {
        //Apply upper/lower bound thresholds for each mask type:
        for (vector<Mask>::size_type mask_idx = 0; mask_idx < d_masks.size(); mask_idx++) {
            if (d_masks[mask_idx].getType() == "mani") {
                d_masks[mask_idx].setUpperBound(config.mask_mani_ub);
                d_masks[mask_idx].setLowerBound(config.mask_mani_lb);
                d_masks[mask_idx].getPclDetection().setMinVolumeCalc(config.mask_mani_min);
                d_masks[mask_idx].getPclDetection().setMaxVolumeCalc(config.mask_mani_max);
            } else if (d_masks[mask_idx].getType() == "chair") {
                d_masks[mask_idx].setUpperBound(config.mask_chair_ub);
                d_masks[mask_idx].setLowerBound(config.mask_chair_lb);
                d_masks[mask_idx].getPclDetection().setMinVolumeCalc(config.mask_chair_min);
                d_masks[mask_idx].getPclDetection().setMaxVolumeCalc(config.mask_chair_max);
            } else if (d_masks[mask_idx].getType() == "door") {
                d_masks[mask_idx].setUpperBound(config.mask_door_ub);
                d_masks[mask_idx].setLowerBound(config.mask_door_lb);
                d_masks[mask_idx].getPclDetection().setMinVolumeCalc(config.mask_door_min);
                d_masks[mask_idx].getPclDetection().setMaxVolumeCalc(config.mask_door_max);
            } else if (d_masks[mask_idx].getType() == "floor") {
                d_masks[mask_idx].setUpperBound(config.mask_floor_ub);
                d_masks[mask_idx].setLowerBound(config.mask_floor_lb);
                d_masks[mask_idx].getPclDetection().setMinVolumeCalc(config.mask_floor_min);
                d_masks[mask_idx].getPclDetection().setMaxVolumeCalc(config.mask_floor_max);
            } else if (d_masks[mask_idx].getType() == "bed") {
                d_masks[mask_idx].setUpperBound(config.mask_bed_ub);
                d_masks[mask_idx].setLowerBound(config.mask_bed_lb);
                d_masks[mask_idx].getPclDetection().setMinVolumeCalc(config.mask_bed_min);
                d_masks[mask_idx].getPclDetection().setMaxVolumeCalc(config.mask_bed_max);
            }
            d_masks[mask_idx].setEnableDebugPoints(config.enable_debug_points);
        }
    }

    void DetectNode::configCallback(heimdall_detectnode::DetectNodeConfig &config, uint32_t level) {
        d_config = config;

        d_calibration_period = config.calibration_period;

        d_enable_depth = config.enable_depth;
        d_enable_rgb = config.enable_rgb;
        d_enable_pcl = config.enable_pcl;
        d_enable_visualization = config.enable_visualization;

        d_rgb_dev_threshold = config.rgb_dev_threshold;
        d_depth_dev_threshold = config.depth_dev_threshold;

        d_erode_size = config.erode_size;
        d_dilate_size = config.dilate_size;
        d_cc_size_threshold = config.cc_size_threshold;

        d_motion_duration = config.motion_duration;
        d_dd_rgb_motion.setMaxAge(d_motion_duration);
        d_dd_depth_motion.setMaxAge(d_motion_duration);

        ROS_INFO("Calibration period: %.5fs", config.calibration_period);

        ROS_INFO("Mani: %.5f, %.5f, %.5f, %.5f", 
                config.mask_mani_ub, config.mask_mani_lb,
                config.mask_mani_min, config.mask_mani_max);
        ROS_INFO("Chair: %.5f, %.5f, %.5f, %.5f", 
                config.mask_chair_ub, config.mask_chair_lb,
                config.mask_chair_min, config.mask_chair_max);
        ROS_INFO("Door: %.5f, %.5f, %.5f, %.5f", 
                config.mask_door_ub, config.mask_door_lb,
                config.mask_door_min, config.mask_door_max);
        ROS_INFO("Floor: %.5f, %.5f, %.5f, %.5f", 
                config.mask_floor_ub, config.mask_floor_lb,
                config.mask_floor_min, config.mask_floor_max);
        ROS_INFO("Bed: %.5f, %.5f, %.5f, %.5f", 
                config.mask_bed_ub, config.mask_bed_lb,
                config.mask_bed_min, config.mask_bed_max);
        setMasksConfig(config);

        setMaskDistanceThreshold(config.mask_distance_threshold);
    }
}
