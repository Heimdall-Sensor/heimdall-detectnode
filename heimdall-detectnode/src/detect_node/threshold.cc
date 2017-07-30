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

using namespace std;

namespace Heimdall
{
    void DetectNode::threshold(Mat & image, Mat & thres_image, DeviationDetection & dd, image_transport::Publisher & pub, double threshold, Mat & dev_image) {
        //TODO: Does this really need to be in a separate method?
        dd.calcDeviation(image, dev_image);

        DeviationDetection::threshold(dev_image, thres_image, threshold);
        //TODO: Used to prevent normalization in image_view: 
        //dev_image.at<double>(0, 0) = 100.0;
        //dev_image.at<double>(1, 0) = 0.0;

        //Publish deviation visualization:
        if (d_enable_visualization) {
            publishImage(pub, dev_image, sensor_msgs::image_encodings::TYPE_64FC1);
        }
    }
}
