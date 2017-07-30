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
    void PclDetection::calcBackgroundModel(void) {
        //Downsample background cloud (to filter duplicates and speed up NN filtering):
        cout << "Number of points before VoxelGrid Background filtering: " << d_background_cloud->points.size() << endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(d_background_cloud);
        sor.setLeafSize(0.05f, 0.05f, 0.05f);
        sor.filter(*downsampled_cloud);
        cout << "Number of points after VoxelGrid Background filtering: " << downsampled_cloud->points.size() << endl;
        d_background_cloud = downsampled_cloud;

        //Publish background cloud for debugging: 
        if (d_mask->isEnableDebugPoints()) {
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*d_background_cloud, cloud_msg); 
            cloud_msg.header.frame_id = "mask_points";
            d_mask->getPointsBackgroundPublisher().publish(cloud_msg);
        }
    }
}
