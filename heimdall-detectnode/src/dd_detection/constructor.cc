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
    DdDetection::DdDetection(Mask * mask) :
        d_mask(mask),
        d_max_activity_age(30.0),
        d_pre_mask_activity(-1),
        d_pre_border_activity(-1),
        d_mask_rising_edge_threshold(0.10),
        d_mask_falling_edge_threshold(0.05),
        d_border_rising_edge_threshold(0.10),
        d_border_falling_edge_threshold(0.05),
        d_mask_distance_threshold(1.0)
    {
        //TODO: Make it work/test (currently a bit deprecated since pcl_detection is used):
/*        [>*/
        ////Create and add border around mask using dilation:
        //d_border = Mat::zeros(d_mask.rows, d_mask.cols, CV_8U);
        //DeviationDetection::dilateImage(d_mask, d_border, 30);
        //*/
        ////TODO: Change; is currently a quick hack to only regard anything below the mask as a border:
        //d_mask.copyTo(d_border);
        ////Move copy of mask down:
        //int offset_x = 0;
        //int offset_y = 20;
        //Mat trans_mat = (Mat_<double>(2,3) << 1, 0, offset_x, 0, 1, offset_y);
        //warpAffine(d_border, d_border, trans_mat, d_border.size());
        
        ////Remove inner mask from border (TODO: No need to create zeros?):
        //Mat zeros = Mat::zeros(d_mask.rows, d_mask.cols, CV_8U);
        //zeros.copyTo(d_border, d_mask);

        //d_mask_total = (int) sum(d_mask)[0];
        /*d_border_total = (int) sum(d_border)[0];*/
    }
}
