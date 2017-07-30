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
    void DeviationDetection::calcDeviation(Mat & image, Mat & dev_image) {
        //TODO: Use C++ templates instead of checks on depth?
        
        //Per condition:
        for (std::vector<Mat>::size_type cond_idx = 0; cond_idx < d_model_means.size(); cond_idx++) {
            for (int i = 0; i < image.rows; i++) {
                for (int j = 0; j < image.cols; j++) {
                    //The amount of stds from mean:
                    double result;
                    switch (d_depth) {
                        case CV_8U: 
                            result = abs(image.at<uchar>(i, j) - d_model_means[cond_idx].at<double>(i, j)) / d_model_stds[cond_idx].at<double>(i, j);
                            break;
                        case CV_16U: 
                            //Assuming a depth image is being used, in which case a value of 0 is invalid (out of range):
                            if (image.at<short>(i, j) > 0 && d_model_counts[cond_idx].at<uint>(i, j) > 0) {
                                result = abs(image.at<short>(i, j) - d_model_means[cond_idx].at<double>(i, j)) / d_model_stds[cond_idx].at<double>(i, j);
                            } else {
                                result = -1; //Indicate invalid pixel!
                            }
                            break;
                        default:
                            throw DeviationDetectionException("Unsupported depth!");
                    }

                    //Determine minimal stds away from mean:
                    if (result > -1 && (cond_idx == 0 || (cond_idx > 0 && dev_image.at<double>(i, j) > result))) {
                        dev_image.at<double>(i, j) = result;
                    }
                }
            }
        }
    }
}
