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

namespace Heimdall
{
    Mask::Mask(ros::NodeHandle &nh, string label, Mat mask) : 
        d_nh(nh),
        d_label(label),
        d_type("unknown"),
        d_name("unknown"),
        d_pcl_detection(PclDetection(this)),
        d_dd_detection(DdDetection(this)),
        d_enable_debug_points(false)
    {
        d_mask = mask.clone();
        extractLabelInfo();

        //Create eroded mask (mainly to improve NN filtering):
        {
            int type = MORPH_ELLIPSE;
            d_mask_eroded = mask.clone();
            int size = 5;
            Mat element = getStructuringElement(type, Size(2 * size + 1, 2 * size + 1), Point(size, size));
            erode(d_mask, d_mask_eroded, element);
            //For debugging purposes: 
            //imwrite("/tmp/mask.jpg", d_mask);
            //imwrite("/tmp/mask_eroded.jpg", d_mask_eroded);
        }

        d_diff_pub = d_nh.advertise<std_msgs::Float64>("/mask/" + d_name + "/diff", 1);

        d_points_mask_pub       = d_nh.advertise<sensor_msgs::PointCloud2>("/mask/" + d_name + "/debug/mask", 1);
        d_points_background_pub = d_nh.advertise<sensor_msgs::PointCloud2>("/mask/" + d_name + "/debug/background", 1);
        d_points_detect_pub     = d_nh.advertise<sensor_msgs::PointCloud2>("/mask/" + d_name + "/debug/detect", 1);
    }

    /**
     * Copy constructor, copies content of mask and changes mask pointer in dd detection and pcl detection modules.
     */
    Mask::Mask(const Mask & mask) :
        d_nh(mask.d_nh),
        d_diff_pub(mask.d_diff_pub),
        d_points_mask_pub(mask.d_points_mask_pub),
        d_points_background_pub(mask.d_points_background_pub),
        d_points_detect_pub(mask.d_points_detect_pub),
        d_enable_debug_points(mask.d_enable_debug_points),
        d_label(mask.d_label),
        d_type("unknown"),
        d_name("unknown"),
        d_upper_bound(0.35),
        d_lower_bound(0.15),
        d_events(mask.d_events),
        d_dd_detection(mask.d_dd_detection),
        d_pcl_detection(mask.d_pcl_detection)
    {
        d_mask = mask.d_mask.clone();
        d_mask_eroded = mask.d_mask_eroded.clone();
        d_dd_detection.setMask(this);        
        d_pcl_detection.setMask(this);        
        extractLabelInfo();
    }

}
