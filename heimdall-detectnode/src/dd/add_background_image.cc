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
    void DeviationDetection::addBackgroundImage(Mat & image, int condition) {
        addBackgroundImage(image, condition, -1);
    }

    void DeviationDetection::addBackgroundImage(Mat & image, int condition, double time) {
        if (time < 0 && isTimed()) {
            throw DeviationDetectionException("A valid time must be specified if module isTimed.");
        } 
        if (condition > getConditionSize() - 1) {
            throw DeviationDetectionException("Condition index out of range!");
        }

        //Verify/Detect change in dimensions:
        if (d_width < 0) {
            d_width = image.cols;
            d_height = image.rows;
            if (d_width < 1 || d_height < 1) {
                throw DeviationDetectionException("Image should at least have 1 pixel!");
            }
        } else if (! (d_width == image.cols && d_height == image.rows)) {
            throw DeviationDetectionException("Image has different dimensions than the ones added previously!");
        }

        //Verify/Detect change in encoding: 
        if (d_depth < 0) {
            d_depth = image.depth();
            switch (d_depth) {
                case CV_8U:
                case CV_16U:
                    //Supported...
                    break;
                default:
                    throw DeviationDetectionException("Specified depth not supported!");
            }
        } else if (d_depth != image.depth()) {
            throw DeviationDetectionException("Image has a different depth than the ones added previously!");
        }

        //Verify/Detect change in number of channels:
        if (d_channel_count < 0) {
            d_channel_count = image.channels();
            if (d_channel_count != 1) {
                throw DeviationDetectionException("Only 1 channel is supported!");
            }
        } else if (d_channel_count != image.channels()) {
            throw DeviationDetectionException("Image has a different number of channels than the ones added previously!");
        }

        //Add actual image and timestamp if needed:
        if (isTimed()) {
            d_bg_times[condition].push_back(time);
        }
        d_bg_images[condition].push_back(image);
    }
}
