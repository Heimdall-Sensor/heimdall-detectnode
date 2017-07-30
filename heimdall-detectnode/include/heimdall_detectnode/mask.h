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
#ifndef _HEIMDALL_MASK_
#define _HEIMDALL_MASK_

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <exception>
#include <vector>
#include <sstream>
#include <iostream>
#include <math.h>
#include <assert.h>

#include <heimdall_msgs/GetMaskSrv.h>

#include <heimdall_detectnode/pcl_detection.h>
#include <heimdall_detectnode/dd_detection.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>


using namespace cv;
using namespace std;

namespace Heimdall
{
    /** Type of event (in/out (negative) mask, within mask activity). */
    enum class MaskEventType {NONE, IN_MASK, OUT_MASK, MASK, IN_MASK_NEG, OUT_MASK_NEG, MASK_NEG};

    class MaskEvent {
        public:
            MaskEvent(MaskEventType type, double time) :
                    type(type), time(time) {
            }

            double time;
            MaskEventType type;
    };

    /**
     * Mask representing a part of the image.
     */
    class Mask
    {
        public:

            Mask(ros::NodeHandle &nh, string label, Mat mask);

            Mask(const Mask & mask);

            const string & getLabel(void) {
                return d_label;
            }

            const string & getName(void) {
                return d_name;
            }

            const string & getType(void) {
                return d_type;
            }

            const Mat & getMask(void) {
                return d_mask;
            }

            Mat & getModifiableMask(void) {
                return d_mask;
            }

            const Mat & getMaskEroded(void) {
                return d_mask_eroded;
            }

            int countOnes(void) {
                int total = 0;
                for (int i = 0; i < d_mask.rows; i++) {
                    for (int j = 0; j < d_mask.cols; j++) {
                        //Pixel is part of mask? 
                        if (((int) d_mask.at<uchar>(i, j)) > 0) {
                            total++;
                        }
                    }
                }
                return total;
            }

            vector<MaskEvent> & getEvents(void) {
                return d_events;
            }

            void clearEvents(void) {
                d_events.clear();
            }

            /**
             * Converts the current mask to a ROS service response.
             */
            void convertToServiceResponse(heimdall_msgs::GetMaskSrv::Response & res);

            DdDetection & getDdDetection(void) {
                return d_dd_detection;
            }

            PclDetection & getPclDetection(void) {
                return d_pcl_detection;
            }

            ~Mask();

            void setUpperBound(double val) {
                d_upper_bound = val;
            }
            void setLowerBound(double val) {
                d_lower_bound = val;
            }
            double getUpperBound(void) {
                return d_upper_bound;
            }
            double getLowerBound(void) {
                return d_lower_bound;
            }

            /** Publishes the different value (of the volume).*/
            void publishDiff(double val) {
                std_msgs::Float64 diff_msg;
                diff_msg.data = val;
                d_diff_pub.publish(diff_msg);
            }

            ros::Publisher & getPointsMaskPublisher(void) {
                return d_points_mask_pub;
            }
            ros::Publisher & getPointsBackgroundPublisher(void) {
                return d_points_background_pub;
            }
            ros::Publisher & getPointsDetectPublisher(void) {
                return d_points_detect_pub;
            }
            void setEnableDebugPoints(bool val) {
                d_enable_debug_points = val;
            }
            bool isEnableDebugPoints(void) {
                return d_enable_debug_points;
            } 
                
        private:
            ros::NodeHandle d_nh;

            //http://stackoverflow.com/questions/236129/split-a-string-in-c#236803
            void split(const string &s, char delim, vector<string> &elems) {
                stringstream ss;
                ss.str(s);
                string item;
                while (getline(ss, item, delim)) {
                    elems.push_back(item);
                }
            }

            /**
             * Extract the label <type>:<name> into the type string and name string.
             */
            void extractLabelInfo(void) {
                vector<string> elems;
                split(getLabel(), ':', elems);
                if (elems.size() > 1) {
                    d_type = elems[0];
                    d_name = elems[1];
                } else {
                    d_name = elems[0];
                }
            }

            //Used to publish (volume) difference (for debugging purposes):
            ros::Publisher d_diff_pub;
            //Used to trigger IN/OUT events:
            double d_upper_bound;
            double d_lower_bound;

            string d_label;
            string d_name;
            string d_type;
            Mat d_mask;            
            Mat d_mask_eroded;

            vector<MaskEvent> d_events;

            //Different detection methods that can be used on this mask:
            DdDetection d_dd_detection;
            PclDetection d_pcl_detection;

            //Generic points publisher for debugging purposes:
            ros::Publisher d_points_mask_pub;
            ros::Publisher d_points_background_pub;
            ros::Publisher d_points_detect_pub;
            bool d_enable_debug_points;
    };
}

#endif // _HEIMDALL_MASk_

