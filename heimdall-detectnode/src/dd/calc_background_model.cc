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
    bool DeviationDetection::calcBackgroundModel(void) {
        //TODO: Use C++ templates instead of checks on depth?
         
        d_model_means.clear();
        d_model_stds.clear();
        d_model_counts.clear();

        //Don't calculate if there are no background images to use:
        if (d_bg_images.size() < 1 || d_bg_images[0].size() < 1) {
            return false;
        }
        
        //Per condition:
        for (std::vector<std::vector<Mat> >::size_type cond_idx = 0; cond_idx < d_bg_images.size(); cond_idx++) {
            //Sum totals:
            Mat means = Mat::zeros(d_height, d_width, CV_64F);
            Mat counts = Mat::zeros(d_height, d_width, CV_32S); //Required to account for invalid pixels (in depth image).
            for (std::vector<Mat>::size_type img_idx = 0; img_idx < d_bg_images[cond_idx].size(); img_idx++) {
                Mat img = d_bg_images[cond_idx][img_idx];
                for (int i = 0; i < img.rows; i++) {
                    for (int j = 0; j < img.cols; j++) {
                        switch (d_depth) {
                            case CV_8U: 
                                means.at<double>(i, j) += img.at<uchar>(i, j);
                                counts.at<uint>(i, j)++;
                                break;
                            case CV_16U: 
                                if (img.at<short>(i, j) > 0) {
                                    means.at<double>(i, j) += img.at<short>(i, j);
                                    counts.at<uint>(i, j)++;
                                }
                                break;
                            default:
                                throw DeviationDetectionException("Unsupported depth!");
                        }
                    }
                }
            }
            d_model_counts.push_back(counts);
            //Calc means:
            for (int i = 0; i < means.rows; i++) {
                for (int j = 0; j < means.cols; j++) {
                    if (counts.at<uint>(i, j) > 0) {
                        means.at<double>(i, j) /= counts.at<uint>(i, j);
                    }
                }
            }
            d_model_means.push_back(means);

            //Sum stds:
            Mat stds = Mat::zeros(d_height, d_width, CV_64F);
            for (std::vector<Mat>::size_type img_idx = 0; img_idx < d_bg_images[cond_idx].size(); img_idx++) {
                Mat img = d_bg_images[cond_idx][img_idx];
                for (int i = 0; i < img.rows; i++) {
                    for (int j = 0; j < img.cols; j++) {
                        switch (d_depth) {
                            case CV_8U: 
                                stds.at<double>(i, j) += pow(img.at<uchar>(i, j) - means.at<double>(i, j), 2);
                                break;
                            case CV_16U: 
                                stds.at<double>(i, j) += pow(img.at<short>(i, j) - means.at<double>(i, j), 2);
                                break;
                            default:
                                throw DeviationDetectionException("Unsupported depth!");
                        }
                    }
                }
            }
            //Calc mean stds:
            double max_std = 0.0;
            for (int i = 0; i < stds.rows; i++) {
                for (int j = 0; j < stds.cols; j++) {
                    if (stds.at<double>(i, j) > 0.001) {
                        if (counts.at<uint>(i, j) > 0) {
                            stds.at<double>(i, j) /= counts.at<uint>(i, j);
                            stds.at<double>(i, j) = sqrt(stds.at<double>(i, j));
                        } else {
                            stds.at<double>(i, j) = -1; //Indicate invalid pixel!
                        }
                        if (stds.at<double>(i, j) > max_std) {
                            max_std = stds.at<double>(i, j);
                        }
                    }
                }
            }
            //Enforce minimal stds (TODO: Use mean std as threshold instead of max?):
            for (int i = 0; i < stds.rows; i++) {
                for (int j = 0; j < stds.cols; j++) {
                    double threshold = max_std / 10.0;
                    if (stds.at<double>(i, j) < threshold && stds.at<double>(i, j) >= 0.0) {
                        stds.at<double>(i, j) = threshold;
                    }
                }
            }

            //Blur/smooth to account for noise:
            if (d_blur_size > 0) {
                blur(stds, stds, Size(d_blur_size, d_blur_size));
            }

            d_model_stds.push_back(stds);
        }
        return true;
    }
}
