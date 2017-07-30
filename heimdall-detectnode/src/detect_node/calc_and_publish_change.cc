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

RNG rng(12345);

namespace Heimdall
{
    void DetectNode::calcAndPublishChange(DeviationDetection & dd_rgb, DeviationDetection & dd_depth, 
            image_transport::Publisher & rgb_dev_pub, image_transport::Publisher & depth_dev_pub, image_transport::Publisher & image_cc_pub,
            ros::Publisher & polygons_pub, bool detect_activity) {
        Mat thres_image;
        //Calculate deviation of RGB and threshold it:
        if (d_enable_rgb) {
            Mat rgb_thres_image = Mat::zeros(dd_rgb.getHeight(), dd_rgb.getWidth(), CV_8U);
            Mat rgb_dev_image = Mat::zeros(dd_rgb.getHeight(), dd_rgb.getWidth(), CV_64F);
            threshold(d_latest_rgb_image, rgb_thres_image, dd_rgb, rgb_dev_pub, d_rgb_dev_threshold, rgb_dev_image);
            thres_image = rgb_thres_image;
        }
        //Calculate deviation of Depth and threshold it:
        if (d_enable_depth) {
            Mat depth_thres_image = Mat::zeros(dd_depth.getHeight(), dd_depth.getWidth(), CV_8U);
            Mat depth_dev_image = Mat::zeros(dd_depth.getHeight(), dd_depth.getWidth(), CV_64F);
            threshold(d_latest_depth_image, depth_thres_image, dd_depth, depth_dev_pub, d_depth_dev_threshold, depth_dev_image);
            if (d_enable_rgb) {
                //Copy one mask over the other (binary OR, or union of both masks):
                depth_thres_image.copyTo(thres_image, depth_thres_image);
            } else {
                thres_image = depth_thres_image;
            }
        }

        //Threshold, and dilate/erode to remove noise:
        DeviationDetection::dilateImage(thres_image, thres_image, d_dilate_size);
        DeviationDetection::erodeImage(thres_image, thres_image, d_erode_size);

        //Detect changes in masks and corresponding borders:
        if (detect_activity) {
            //TODO:XXX: Why is this only implemented for Depth?
            if (d_enable_depth) {
                Mat depth_diff_image = Mat::zeros(dd_depth.getHeight(), dd_depth.getWidth(), CV_64F);
                dd_depth.calcDifference(d_latest_depth_image, depth_diff_image);
                //cout << depth_diff_image.at<double>(dd_depth.getWidth() / 2, dd_depth.getHeight() / 2) << endl;
                double mean_activity = detectActivity(thres_image, depth_diff_image);
                //Publish mean activity (0-1):
                std_msgs::Float64 activity_msg;
                activity_msg.data = mean_activity;
                d_activity_pub.publish(activity_msg);
            }
        }

        //Find connected components:
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(thres_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

        vector<vector<Point> > contours_poly(contours.size());
        heimdall_msgs::PolygonArray polygon_array;
        polygon_array.polygons.clear();

        Mat cc_image = Mat::zeros(thres_image.size(), CV_8UC3);

        for (int i = 0; i < contours.size(); i++) {
            if (contourArea(contours[i]) > d_cc_size_threshold) {
                if (d_enable_visualization) {
                    Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
                    drawContours(cc_image, contours, i, color, CV_FILLED, 8, hierarchy, 0, Point());
                }
                approxPolyDP(Mat(contours[i]), contours_poly[i], 1, true);

                //Convert OpenCV polygon to ROS polygon:
                geometry_msgs::Polygon polygon;
                polygon.points.clear();
                for (vector<Point>::size_type polygon_idx = 0; polygon_idx < contours_poly[i].size(); polygon_idx++) {
                    geometry_msgs::Point32 point;
                    point.x = contours_poly[i][polygon_idx].x;
                    point.y = contours_poly[i][polygon_idx].y;
                    polygon.points.push_back(point);
                }
                polygon_array.polygons.push_back(polygon);
            }
        }
        polygons_pub.publish(polygon_array);

        //Publish connected components visualization:
        if (d_enable_visualization) {
            publishImage(image_cc_pub, cc_image, sensor_msgs::image_encodings::BGR8);
        }
    }
}
