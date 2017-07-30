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

    void DetectNode::createColoredCloud(void) {
        int colors[][3] = {
            {255, 0, 0},
            {0, 255, 0},
            {0, 0, 255},
            {255, 255, 0},
            {0, 255, 255},
            {0, 255, 255},
            {100, 0, 0},
            {0, 100, 0},
            {0, 0, 100},
            {-1, -1, -1},
        };
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>(*d_latest_points));
        int color_idx = 0;
        int mask_marker_id = 0;
        for (auto & mask : d_masks) {
            mask_marker_id++;

            mask.getPclDetection().colorCloud(colored_cloud, colors[color_idx][0], colors[color_idx][1], colors[color_idx][2]);
            if (colors[color_idx][0] < 0) {
                color_idx = 0;
            }
            color_idx++;

            Eigen::Vector4f centroid = mask.getPclDetection().getCentroid();
            cout << centroid << endl;

            visualization_msgs::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time();
            marker.ns = "masks";
            marker.id = mask_marker_id;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = centroid[0];
            marker.pose.position.y = centroid[1];
            marker.pose.position.z = centroid[2];
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.0;
            marker.scale.y = 0.0;
            marker.scale.z = 0.2;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.text = mask.getLabel();
            //only if using a MESH_RESOURCE marker type:
            //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
            d_rviz_mask_marker_pub.publish( marker );
        }
        //Publish transformed cloud for debugging: 
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*colored_cloud, cloud_msg); 
        cloud_msg.header.frame_id = "mask_points";
        d_masks_points_pub.publish(cloud_msg);
    }
}
