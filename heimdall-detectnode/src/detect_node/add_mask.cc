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
    void DetectNode::addMask(string & label, Mat & mask_img) {
        //TODO: Check if the dimensions of the mask are acceptable?
        
        //Check if mask already exists:
        if (hasMask(label)) {
            throw DetectNodeException("Specified mask already exists!");
        }

        Mask mask(d_nh, label, mask_img);
        mask.getDdDetection().setDistanceThreshold(d_mask_distance_threshold);
        d_masks.push_back(mask);

	    saveMasks();

        setMasksConfig(d_config);
    }
}
