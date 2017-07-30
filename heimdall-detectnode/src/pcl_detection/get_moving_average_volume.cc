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
    double PclDetection::getMovingAverageVolume(double new_volume) {
        double current_time = ros::Time::now().toSec();
        //Add new value:
        d_volume_val_list.push_back(new_volume);
        d_volume_time_list.push_back(current_time);
        //Remove old values (TODO: Improve; can be done more efficiently using a circular or linked list):
        while (true) {
            int idx_to_remove = -1;
            int idx = 0;
            for (auto & time : d_volume_time_list) {
                if (time < (current_time - 5.0)) {
                    idx_to_remove = idx; 
                    break;
                }
                idx += 1;
            }
            if (idx_to_remove < 0) {
                break;
            } else {
                d_volume_time_list.erase(d_volume_time_list.begin() + idx_to_remove);
                d_volume_val_list.erase(d_volume_val_list.begin() + idx_to_remove);
            }
        }
        //Calculate average and return:
        double total = 0.0;
        for (auto & value : d_volume_val_list) {
            total += value;
        }
        d_last_moving_average_volume = total / d_volume_val_list.size();
        return d_last_moving_average_volume;
    }
}
