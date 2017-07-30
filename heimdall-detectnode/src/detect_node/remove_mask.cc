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
#include <unistd.h>

using namespace std;

namespace Heimdall
{
    void DetectNode::removeMask(string & label) {
        //Search for label:
        int mask_idx = findMask(label);
        if (mask_idx < 0) {
            throw DeviationDetectionException("Specified mask not found!");
        }
        d_masks.erase(d_masks.begin() + mask_idx);
        string file = (boost::format("%1%/masks/%2%.bmp") % ros::package::getPath("heimdall_detectnode") % label).str();
        unlink(file.c_str());
    }
}
