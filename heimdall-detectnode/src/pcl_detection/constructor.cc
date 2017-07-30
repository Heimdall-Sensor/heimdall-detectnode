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
    PclDetection::PclDetection(Mask * mask) :
        d_mask(mask),
        d_has_calibrated_cloud(false),
        d_calibration_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
        d_full_calibration_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
        d_projection_transform(Eigen::Matrix4f::Identity()),
        d_surface_y_min(0.0), 
        d_surface_y_max(0.0), 
        d_surface_z_min(0.0), 
        d_surface_z_max(0.0),
        d_surface_reversed(false),
        d_occupied(false),
        d_occupied_neg(false),
        d_x_min(0.0),
        d_x_max(0.0),
        d_y_min(0.0),
        d_y_max(0.0),
        d_z_min(0.0),
        d_z_max(0.0),
        d_mean_point_surface(0.0),
        d_volume_offset(0.0),
        d_volume_offset_total(0.0),
        d_volume_offset_count(0),
        d_last_moving_average_volume(0.0),
        d_x_min_config(0.0),
        d_x_max_config(0.0),
        d_width(0.0),
        d_length(0.0)
    {
    }
}
