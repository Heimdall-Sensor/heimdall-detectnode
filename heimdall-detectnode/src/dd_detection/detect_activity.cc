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
    void DdDetection::detectActivity(Mat & img_change, double & mask_activity, double & border_activity, double time, Mat & dev_depth_image) {
        //Ignore pixels that are too far away from the background model in depth (TODO: Use absolute distance instead of stds?)
        //(should ignore changes in mask such as walking in front of bed rather than stepping out of bed):
        Mat img_change_filtered = Mat::zeros(img_change.rows, img_change.cols, CV_8U);
        threshold(img_change, img_change_filtered, dev_depth_image, d_mask_distance_threshold);
        //img_change.copyTo(img_change_filtered); 

        //Select only changes in mask and border:
        Mat mask_change = Mat::zeros(img_change_filtered.rows, img_change_filtered.cols, CV_8U);
        Mat border_change = Mat::zeros(img_change_filtered.rows, img_change_filtered.cols, CV_8U);
        img_change_filtered.copyTo(mask_change, d_mask->getMask());
        img_change_filtered.copyTo(border_change, d_border);

        //TODO: Compensate for distance of mask (calculate mean distance using depth information and use it to modify/normalize the resulting activity):
        mask_activity = sum(mask_change)[0] / d_mask_total;
        border_activity = sum(border_change)[0] / d_border_total;

        //Records changes:
        if (d_activity_changes.size() == 0) {
            //Initial change (the only change without at least one rising or falling edge):
            MaskActivityType mask_change = MaskActivityType::ZERO;
            if (mask_activity > d_mask_rising_edge_threshold) {
                mask_change = MaskActivityType::ONE;
            }
            MaskActivityType border_change = MaskActivityType::ZERO;
            if (border_activity > d_border_rising_edge_threshold) {
                border_change = MaskActivityType::ONE;
            }

            MaskActivityChange change(mask_change, border_change, time);
            d_activity_changes.push_back(change);
        } else {
            //After at least one activity has already been recorded:
            MaskActivityChange last_change = d_activity_changes.back();

            MaskActivityType mask_change;
            detectChange(mask_activity, d_mask_rising_edge_threshold, d_mask_falling_edge_threshold, last_change.mask, mask_change);

            MaskActivityType border_change;
            detectChange(border_activity, d_border_rising_edge_threshold, d_border_falling_edge_threshold, last_change.border, border_change);

            //Overwrite last change whenever it contains no change and history is at least of size 2:
            if ((last_change.mask == MaskActivityType::ONE || last_change.mask == MaskActivityType::ZERO)
                    && (last_change.border == MaskActivityType::ONE || last_change.border == MaskActivityType::ZERO)
                    && (d_activity_changes.size() > 1)) {
                MaskActivityChange change(mask_change, border_change, time);
                d_activity_changes[d_activity_changes.size() - 1] = change; 
            } else {
                MaskActivityChange change(mask_change, border_change, time);
                d_activity_changes.push_back(change);
            }
        }

        //Consume smitt triggered activities and try to detect a notifiable change:
        processActivities();
    }
}
