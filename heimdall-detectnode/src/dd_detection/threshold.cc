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
    void DdDetection::threshold(Mat & img_change, Mat & img_change_filtered, Mat & dev_depth_image, double threshold) {
        //Note: Can't use opencv threshold; does not work on CV_64F type images.
        for (int i = 0; i < img_change.rows; i++) {
            for (int j = 0; j < img_change.cols; j++) {
                if (dev_depth_image.at<double>(i, j) < threshold && img_change.at<uchar>(i, j) > 0) {
                    img_change_filtered.at<uchar>(i, j) = 1;
                }
            }
        }
    }
}
