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
    void DdDetection::detectOutMaskEvent(void) {
        //Detect and consume out-mask event:
        {
            double last_mask_rise = -1;
            double last_mask_fall = -1;
            int consume_idx = -1; 
            bool out_mask_detected = false;
            for (vector<MaskActivityChange>::size_type change_idx = 0; change_idx < d_activity_changes.size(); change_idx++) {
                //Record last time mask was active:
                if ((d_activity_changes[change_idx].mask == MaskActivityType::ONE || d_activity_changes[change_idx].mask == MaskActivityType::RISING_EDGE)
                        && last_mask_rise < 0) {
                    last_mask_rise = d_activity_changes[change_idx].time;
                }
                //Detect potential out-mask event:
                if (d_activity_changes[change_idx].mask == MaskActivityType::FALLING_EDGE && d_activity_changes[change_idx].border == MaskActivityType::ONE
                        && last_mask_rise >= 0
                        && (d_activity_changes[change_idx].time - last_mask_rise > 0.3)) {
                    last_mask_fall = d_activity_changes[change_idx].time;
                    consume_idx = change_idx;
                }
                //Mark potential out-mask event as valid:
                if ((d_activity_changes[change_idx].border == MaskActivityType::FALLING_EDGE || d_activity_changes[change_idx].border == MaskActivityType::ZERO)
                        && last_mask_fall >= 0
                        && (d_activity_changes[change_idx].time - last_mask_fall > 0.3)) {
                    cout << "Out-Mask event detected!" << endl;
                    MaskEvent event(MaskEventType::OUT_MASK, last_mask_fall);
                    d_mask->getEvents().push_back(event);
                    out_mask_detected = true;
                    break;
                }
            }
            //Consume data if detected:
            if (out_mask_detected) {
                d_activity_changes.erase(d_activity_changes.begin(), d_activity_changes.begin() + consume_idx);
            }
        }
    }
}
