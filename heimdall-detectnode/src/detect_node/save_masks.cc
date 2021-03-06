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
    void DetectNode::saveMasks(void) {
        string mask_path((boost::format("%1%/masks/") % ros::package::getPath("heimdall_detectnode")).str());
	ROS_INFO("Saving masks to %s", mask_path.c_str());
        for (vector<Mask>::size_type mask_idx = 0; mask_idx < d_masks.size(); mask_idx++) {
            imwrite((boost::format("%1%/masks/%2%.bmp") % ros::package::getPath("heimdall_detectnode") % d_masks[mask_idx].getLabel()).str(), d_masks[mask_idx].getMask());
        }
    }
}
