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
    void PclDetection::extractCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segment) {
        const Mat & mask = d_mask->getMask();
        //const Mat & mask = d_mask->getMaskEroded(); //XXX: Doesn't seem to always work well (sometimes mask is empty!).

        cout << "Mask size: " << mask.rows * mask.cols << endl;
        cout << "Cloud size: " << cloud->points.size() << endl;
        //Depth can be QVGA while mask (RGB) is VGA:
        bool qvga = mask.rows * mask.cols == cloud->points.size() * 4;
        if (qvga) {
            cout << "Assuming Depth is QVGA and Color is VGA." << endl;
        }
        assert(mask.rows * mask.cols == cloud->points.size() or qvga);

        int pi = 0;
        for (int i = 0; i < mask.rows; i++) {
            if (qvga and i % 2 == 1) {
                continue; //Skip every other row in mask (depth is QVGA)
            }
            for (int j = 0; j < mask.cols; j++) {
                if (qvga and j % 2 == 1) {
                    continue; //Skip every other column in mask (depth is QVGA)
                }
                //Pixel is part of mask? 
                if (((int) mask.at<uchar>(i, j)) > 0) {
                    //Add corresponding point to cloud segment if it does not contain a NaN value:
                    if (! (isnan(cloud->points[pi].x) ||
                                isnan(cloud->points[pi].y) ||
                                isnan(cloud->points[pi].z))) {
                        cloud_segment->points.push_back(cloud->points[pi]); 
                    }
                }
                pi++;
            }
        }
        //Additional cloud information:
        cloud_segment->width = static_cast<uint32_t> (cloud_segment->points.size());
        cloud_segment->height = 1;
        cloud_segment->is_dense = true;
    }
}
