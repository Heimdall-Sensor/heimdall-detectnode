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
    void PclDetection::calibrateBackground(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud) {
        //Transform cloud:
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*cloud, *transformed_cloud, d_projection_transform);
        //Filter (on bounding box):
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        filterCloud(transformed_cloud, filtered_cloud);

        calibrateOffset(filtered_cloud);
        addBackground(filtered_cloud);

        d_has_calibrated_cloud = true;
    }
}
