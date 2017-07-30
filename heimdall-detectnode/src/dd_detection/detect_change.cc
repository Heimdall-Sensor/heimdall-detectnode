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
    void DdDetection::detectChange(double mask_activity, 
            double rising_edge_threshold, double falling_edge_threshold, 
            const MaskActivityType & last_change, MaskActivityType & mask_change) {
        if (mask_activity > rising_edge_threshold) {
            //Record possible rising edge:
            if (last_change == MaskActivityType::ZERO || last_change == MaskActivityType::FALLING_EDGE) {
                mask_change = MaskActivityType::RISING_EDGE;
            } else {
                mask_change = MaskActivityType::ONE;
            }
        } else if (mask_activity < falling_edge_threshold) {
            //Record possible falling edge:
            if (last_change == MaskActivityType::ONE || last_change == MaskActivityType::RISING_EDGE) {
                mask_change = MaskActivityType::FALLING_EDGE;
            } else {
                mask_change = MaskActivityType::ZERO;
            }
        } else {
            //No change:
            mask_change = last_change;
        }
    }
}
