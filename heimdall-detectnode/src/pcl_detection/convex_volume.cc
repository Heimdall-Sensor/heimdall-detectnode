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
    double PclDetection::convexHullVolume(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr const & cloud) {
    	//Creating convex hull class and the required place holders for the surfaces and vertices.
    	pcl::ConvexHull<pcl::PointXYZRGB> hull;
    	pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
    	std::vector<pcl::Vertices> polygons;

    	//Sets the cloud and calculated the Convex Hull
        hull.setComputeAreaVolume(true);
		hull.setInputCloud(cloud);
		hull.setDimension(3);
		hull.reconstruct(*surface_hull, polygons);

		//Returns the total volume
		return hull.getTotalVolume();
    }
}

