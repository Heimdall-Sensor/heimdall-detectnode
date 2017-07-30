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
    //TODO: Remove "dd" prefix (and namespace in launch file?):
    DetectNode::DetectNode(ros::NodeHandle &nh, std::string const & rgb_topic, std::string const & depth_topic, std::string points_topic) :
        d_nh(nh),
        d_it(nh),
        d_latest_rgb_time(0),
        d_latest_depth_time(0),
        d_latest_points_time(0),
        d_last_cal_points_time(0),
        d_last_cal_rgb_time(0),
        d_last_cal_depth_time(0),
        d_rate(100),
        d_mode(DetectNodeMode::IDLE),
        d_next_cal_condition(0),
        d_rgb_encoding(""),
        d_depth_encoding(""),
        d_calibration_period(5.0),
        d_erode_size(5),
        d_dilate_size(5),
        d_cc_size_threshold(10.0),
        d_rgb_dev_threshold(100.0),
        d_depth_dev_threshold(1.0),
        d_command_service(d_nh.advertiseService("/dd/command", &DetectNode::commandService, this)),
        d_add_mask_service(d_nh.advertiseService("/dd/add_mask", &DetectNode::addMaskService, this)),
        d_remove_mask_service(d_nh.advertiseService("/dd/remove_mask_list", &DetectNode::removeMaskListService, this)),
        d_get_mask_list_service(d_nh.advertiseService("/dd/get_mask_list", &DetectNode::getMaskListService, this)),
        d_get_mask_service(d_nh.advertiseService("/dd/get_mask", &DetectNode::getMaskService, this)),
        d_rgb_count(0),
        d_depth_count(0),
        d_motion_age(1.0),
        d_activity_header_seq(0),
        d_motion_duration(1.0),
        d_mask_distance_threshold(500),
        d_enable_depth(false), 
        d_enable_rgb(false),
        d_enable_pcl(true),
        d_enable_visualization(false),
        d_config(heimdall_detectnode::DetectNodeConfig())
    {
        d_rgb_image_sub = d_it.subscribe(rgb_topic, 1, &DetectNode::rgbCallback, this);
        d_depth_image_sub = d_it.subscribe(depth_topic, 1, &DetectNode::depthCallback, this);

        d_points_sub = d_nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >(points_topic, 1, &DetectNode::pointsCallback, this);

        d_location_pub = d_nh.advertise<std_msgs::UInt64>("/dd/location", 1);

        //d_masks_points_pub = d_nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/dd/mask_points", 1);
        d_masks_points_pub = d_nh.advertise<sensor_msgs::PointCloud2>("/dd/mask_points", 1);

        d_visualization_rgb_dev_pub = d_it.advertise("/dd/visual/rgb_dev", 1);
        d_visualization_depth_dev_pub = d_it.advertise("/dd/visual/depth_dev", 1);

        d_visualization_rgb_motion_dev_pub = d_it.advertise("/dd/visual/rgb_motion_dev", 1);
        d_visualization_depth_motion_dev_pub = d_it.advertise("/dd/visual/depth_motion_dev", 1);

        d_visualization_image_cc_pub = d_it.advertise("/dd/visual/cc", 1);
        d_visualization_image_cc_motion_pub = d_it.advertise("/dd/visual/cc_motion", 1);

        d_polygons_pub = d_nh.advertise<heimdall_msgs::PolygonArray>("/dd/polygons", 1);
        d_polygons_motion_pub = d_nh.advertise<heimdall_msgs::PolygonArray>("/dd/polygons_motion", 1);

        d_activities_pub = d_nh.advertise<heimdall_msgs::ActivityArrayStamped>("/dd/activities", 1);
        d_volumes_pub = d_nh.advertise<std_msgs::Float64MultiArray>("/dd/volumes", 1);

        d_masks_pub = d_nh.advertise<heimdall_msgs::Masks>("/dd/masks", 1);

        d_notification_pub = d_nh.advertise<std_msgs::String>("/heimdall/notification", 1);

        d_rviz_mask_marker_pub = nh.advertise<visualization_msgs::Marker>("rviz_mask_marker", 0);

        d_dd_rgb_motion.setTimed(true);
        d_dd_rgb_motion.setConditionSize(1);
        d_dd_depth_motion.setTimed(true);
        d_dd_depth_motion.setConditionSize(1);

        //Setup dynamic reconfiguration:
        dynamic_reconfigure::Server<heimdall_detectnode::DetectNodeConfig>::CallbackType f;
        f = boost::bind(&DetectNode::configCallback, this, _1, _2);
        d_config_server.setCallback(f);

        d_activity_pub = d_nh.advertise<std_msgs::Float64>("/heimdall/activity", 1);
        loadMasks();
    }
}
