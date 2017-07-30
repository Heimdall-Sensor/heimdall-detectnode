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
    void PclDetection::colorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, int r, int g, int b) {
        const Mat & mask = d_mask->getMask();

        assert(mask.rows * mask.cols == cloud->points.size());

        for (int i = 0; i < mask.rows; i++) {
            for (int j = 0; j < mask.cols; j++) {
                //Pixel is part of mask? 
                if (((int) mask.at<uchar>(i, j)) > 0) {
                    //Add corresponding point to cloud segment if it does not contain a NaN value:
                    if (! (isnan(cloud->points[i * mask.cols + j].x) ||
                                isnan(cloud->points[i * mask.cols + j].y) ||
                                isnan(cloud->points[i * mask.cols + j].z))) {
                        cloud->points[i * mask.cols + j].r = r;
                        cloud->points[i * mask.cols + j].g = g;
                        cloud->points[i * mask.cols + j].b = b;
                    }
                }
            }
        }
    }
}
