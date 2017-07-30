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
    void DeviationDetection::filter(double time) {
        if (! isTimed()) {
            throw DeviationDetectionException("Can't filter on time if not in timed mode!");
        }
        assert(d_bg_images.size() == d_bg_times.size());

        //Iterate over conditions:
        for (std::vector<std::vector<Mat> >::size_type cond_idx = 0; cond_idx < d_bg_images.size(); cond_idx++) {
            assert(d_bg_images[cond_idx].size() == d_bg_times[cond_idx].size());

            //Find images to remove: 
            //TODO: Kind of inefficient, should use linked list, cycle buffer or remove using iterator (with the assumption that they are ordered)?
            //Or join image and time as tuple so they can be removed in the same iteration.
            while (true) {
                int idx_to_remove = -1; 
                for (std::vector<Mat>::size_type img_idx = 0; img_idx < d_bg_images[cond_idx].size(); img_idx++) {
                    if (time - d_bg_times[cond_idx][img_idx] > d_max_age) {
                        idx_to_remove = img_idx;
                    }
                }
                if (idx_to_remove >= 0) {
                    d_bg_images[cond_idx].erase(d_bg_images[cond_idx].begin() + idx_to_remove);
                    d_bg_times[cond_idx].erase(d_bg_times[cond_idx].begin() + idx_to_remove);
                } else {
                    break;
                }
            }
        }
    }
}
