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
    void DdDetection::detectInMaskEvent(void) {
        //Detect and consume in-mask event:
        {
            double last_border_rise = -1;
            double last_mask_rise = -1;
            int consume_idx = -1; 
            bool in_mask_detected = false;
            for (vector<MaskActivityChange>::size_type change_idx = 0; change_idx < d_activity_changes.size(); change_idx++) {
                //Record last time border was active:
                if ((d_activity_changes[change_idx].border == MaskActivityType::ONE || d_activity_changes[change_idx].border == MaskActivityType::RISING_EDGE)
                        && last_border_rise < 0) {
                    last_border_rise = d_activity_changes[change_idx].time;
                }
                //Detect potential in-mask event:
                if (d_activity_changes[change_idx].mask == MaskActivityType::RISING_EDGE && d_activity_changes[change_idx].border == MaskActivityType::ONE
                        && last_border_rise >= 0
                        && (d_activity_changes[change_idx].time - last_border_rise > 0.3)) {
                    last_mask_rise = d_activity_changes[change_idx].time;
                    consume_idx = change_idx;
                }
                //Mark potential in-mask event as valid:
                if ((d_activity_changes[change_idx].mask == MaskActivityType::FALLING_EDGE || d_activity_changes[change_idx].mask == MaskActivityType::ONE)
                        && last_mask_rise >= 0
                        && (d_activity_changes[change_idx].time - last_mask_rise > 0.3)) {
                    cout << "In-Mask event detected!" << endl;
                    MaskEvent event(MaskEventType::IN_MASK, last_mask_rise);
                    d_mask->getEvents().push_back(event);
                    in_mask_detected = true;
                    break;
                }
            }
            //Consume data if detected:
            if (in_mask_detected) {
                d_activity_changes.erase(d_activity_changes.begin(), d_activity_changes.begin() + consume_idx);
            }
        }
    }
}
