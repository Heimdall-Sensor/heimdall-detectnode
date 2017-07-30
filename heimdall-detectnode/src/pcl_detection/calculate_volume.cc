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
    double PclDetection::calculateVolume(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & full_cloud) {
        bool use_convex_hull = false; 

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        filterBackground(full_cloud, cloud);

        //Make sure there is sufficient amount of points to calculate the volume for:
        if (cloud->points.size() < 10) {
            return 0.0;
        }

        if (use_convex_hull) {
            //New method:
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segment(new pcl::PointCloud<pcl::PointXYZRGB>);
            for (auto & point : cloud->points) {
                //Add original point:
                cloud_segment->points.push_back(point); 
                //Add extra one project on the XY plane (thuse greating a hull):
                pcl::PointXYZRGB new_point;
                new_point.z = point.z;
                new_point.y = point.y;
                new_point.x = 0.0;
                cloud_segment->points.push_back(new_point); 
            }

            //Publish transformed cloud for debugging: 
            if (d_mask->isEnableDebugPoints()) {
                sensor_msgs::PointCloud2 cloud_msg;
                pcl::toROSMsg(*cloud_segment, cloud_msg); 
                cloud_msg.header.frame_id = "mask_points";
                d_mask->getPointsDetectPublisher().publish(cloud_msg);
            }
            return convexHullVolume(cloud_segment);
        } else {
            //Publish transformed cloud for debugging: 
            if (d_mask->isEnableDebugPoints()) {
                sensor_msgs::PointCloud2 cloud_msg;
                pcl::toROSMsg(*cloud, cloud_msg); 
                cloud_msg.header.frame_id = "mask_points";
                d_mask->getPointsDetectPublisher().publish(cloud_msg);
            }
            //Count the number of points within the bounding box:
            int count = 0;
            double mean_volume_pp = 0.0; //Estimate of the volume
            for (auto & point : cloud->points) {
                count += 1;
                mean_volume_pp += point.x;
            }
            mean_volume_pp /= count; //Average volume per point
            //cout << "count: " << count << endl;
            double volume = d_mean_point_surface * mean_volume_pp * count;
            if (d_surface_reversed) {
                volume *= -1;
            }
            return volume;
        }
    }
}
