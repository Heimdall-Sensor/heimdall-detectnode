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
    void PclDetection::filterCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & full_cloud,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr & filtered_cloud) {
        for (auto & point : full_cloud->points) {
            if (! (isnan(point.x) || isnan(point.y) || isnan(point.z))) {
                if (   point.x > d_x_min && point.x < d_x_max
                        && point.y > d_y_min && point.y < d_y_max
                        && point.z > d_z_min && point.z < d_z_max) {
                    if (d_apply_nn_filtering) {
                        //Find closest point to calibrated cloud (projected on ZY plane):
                        double min_distance = -1;
                        for (auto & cal_point : d_calibration_cloud->points) {
                            double distance = sqrt(pow(cal_point.z - point.z, 2) + pow(cal_point.y - point.y, 2));
                            //double distance = (cal_point.z - point.z) + (cal_point.y - point.y);
                            if (min_distance < 0.0 || min_distance > distance) {
                                min_distance = distance;
                            }
                        }
                        //Close enough?
                        if (min_distance < 0.05) {
                            filtered_cloud->points.push_back(point);
                        } 
                    } else {
                        filtered_cloud->points.push_back(point);
                    }
                }
            }
        }
    }
}
