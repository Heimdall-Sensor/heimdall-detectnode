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
#ifndef _HEIMDALL_DD_DETECTION_
#define _HEIMDALL_DD_DETECTION_

#include <exception>
#include <vector>
#include <iostream>
#include <math.h>
#include <assert.h>

#include <heimdall_detectnode/mask.h>

using namespace cv;
using namespace std;

namespace Heimdall
{

    //Forward declaration of Mask:
    class Mask;

    /** Binarized activity type (rising edge, falling edge, 1 or 0), the change in a mask or border. */
    enum class MaskActivityType {RISING_EDGE, FALLING_EDGE, ONE, ZERO};

    class MaskActivityChange
    {
        public:
            MaskActivityChange(MaskActivityType mask, MaskActivityType border, double time) :
                    mask(mask), border(border), time(time) { 
            }

            const string toString(MaskActivityType type) {
                switch (type) {
                    case MaskActivityType::RISING_EDGE:
                        return "R";
                    case MaskActivityType::FALLING_EDGE:
                        return "F";
                    case MaskActivityType::ONE:
                        return "1";
                    case MaskActivityType::ZERO:
                        return "0";
                    default:
                        return "X";
                }
            }

            void printInfo(void) {
                cout << toString(mask) << ", " << toString(border);
                printf(", %.3f\n", time);
            }

            MaskActivityType mask;
            MaskActivityType border;
            double time;
    };

    /**
     * Activity detection related to a mask using "DeviationDetection" in combination with OpenCV.
     */
    class DdDetection
    {
        public:
            DdDetection(Mask * mask);

            void setMask(Mask * mask) {
                d_mask = mask;
            }

            /**
             * Detects and records activity (change within the mask) in the mask and border.
             */
            void detectActivity(Mat & img_change, double & mask_activity, double & border_activity, double current_time, Mat & dev_depth_image);

            /**
             * Detects change (falling/rising edge) compared to last value.
             */
            void detectChange(double mask_activity, 
                    double rising_edge_threshold, double falling_edge_threshold, 
                    const MaskActivityType & last_change, MaskActivityType & mask_change);

            /** 
             * Consume smitt triggered activities and try to detect a notifiable change.
             */
            void processActivities(void);

            /**
             * Detect and consume in-mask activity event.
             */
            void detectInMaskEvent(void);

            /**
             * Detect and consume out-mask activity event.
             */
            void detectOutMaskEvent(void);

            /**
             * Detect and consume within-mask activity event.
             */
            void detectWithinMaskEvent(void);

            /** 
             * Ignore pixels that are too far away from the background model in depth (TODO: Use absolute distance instead of stds?)
             */
            void threshold(Mat & img_change, Mat & img_change_filtered, Mat & dev_depth_image, double threshold);

            /**
             * Sets the distance threshold for the masks. 
             */
            void setDistanceThreshold(double threshold) {
                d_mask_distance_threshold = threshold;
            }


        private:
            Mask * d_mask;

            Mat d_border;
            int d_mask_total;
            int d_border_total;

            //Smitt trigger thresholds:
            double d_mask_rising_edge_threshold;
            double d_mask_falling_edge_threshold;
            double d_border_rising_edge_threshold;
            double d_border_falling_edge_threshold;

            //Previously recorded mask/border activities to detect rising or falling edge:
            double d_pre_mask_activity;
            double d_pre_border_activity;

            //Recording of the *changes* in activities (after smitt triggering):
            double d_max_activity_age;
            vector<MaskActivityChange> d_activity_changes;

            double d_mask_distance_threshold;
    };
}

#endif //_HEIMDALL_DD_DETECTION_
