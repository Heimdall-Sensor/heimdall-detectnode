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
    bool DetectNode::addMaskService(heimdall_msgs::AddMaskSrv::Request &req, heimdall_msgs::AddMaskSrv::Response &res) {
        //Verify if mask has correct format:
        if ((req.width * req.height) != req.pixels.size()) {
            throw DetectNodeException("Width and height does not correspond to size of pixel array!");
        }
        //TODO: Check if width and height correspond to existing received images? 
        //Copy mask:
        Mat mask = Mat::zeros(req.height, req.width, CV_8U);
        for (int i = 0; i < mask.rows; i++) {
            for (int j = 0; j < mask.cols; j++) {
                mask.at<uchar>(i, j) = req.pixels[req.width * i + j];
            }
        }
        //Add mask and label:
        addMask(req.label, mask);

        publishMasks();
        return true;
    }
}
