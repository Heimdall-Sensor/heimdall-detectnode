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
    bool DetectNode::run() {
        double current_time = ros::Time::now().toSec();
        double timeout = 5.0;

        d_rate.sleep();

        switch (d_mode) {
            case DetectNodeMode::IDLE:
                //Nothing to do...
                break;
            case DetectNodeMode::CALIBRATE:
                //TODO: Only increase condition (and switch to IDLE mode if the minimal amount of calibration images are received?:
                if ((current_time - d_calibration_start_time.toSec()) > d_calibration_period) {

                    if (d_enable_rgb) {
                        d_dd_rgb.printModelInfo();
                    }
                    if (d_enable_depth) {
                        d_dd_depth.printModelInfo();
                    }
                    if (d_enable_pcl) {
                        for (auto & mask : d_masks) {
                            mask.getPclDetection().calcBackgroundModel();
                        }
                    }

                    //Calibration finished:
                    ROS_INFO("Calibration finished for new condition, switching to IDLE mode.");

                    d_mode = DetectNodeMode::IDLE;
                    d_next_cal_condition++;

                    //TODO: Remove in production code (should be done by remote service after calibration):
                    detect();
                } else if ((((current_time - d_latest_rgb_time.toSec()) > timeout) && d_enable_rgb)
                        || (((current_time - d_latest_depth_time.toSec()) > timeout) && d_enable_depth)) {
                    //Whenever RGB or Depth images are too old:
                    ROS_ERROR("Can't calibrate; latest RGB or Depth image is too old! Switching to IDLE mode.");
                    ROS_ERROR("timeout: %.3f", timeout);
                    if (d_enable_rgb) {
                        ROS_ERROR("latest rgb time: %.3f", d_latest_rgb_time.toSec());
                    }
                    if (d_enable_depth) {
                        ROS_ERROR("latest depth time: %.3f", d_latest_depth_time.toSec());
                    }
                    ROS_ERROR("time: %.3f", current_time);

                    d_mode = DetectNodeMode::IDLE;
                    d_next_cal_condition++;
                } else {
                    //During DD calibration:
                    if (d_enable_rgb) {
                        if (d_latest_rgb_time.toSec() > d_last_cal_rgb_time.toSec()) {
                            d_last_cal_rgb_time = ros::Time::now();
                            d_dd_rgb.addBackgroundImage(d_latest_rgb_image, d_next_cal_condition);
                        }
                    }
                    if (d_enable_depth) {
                        if (d_latest_depth_time.toSec() > d_last_cal_depth_time.toSec()) {
                            d_last_cal_depth_time = ros::Time::now();
                            d_dd_depth.addBackgroundImage(d_latest_depth_image, d_next_cal_condition);
                        }
                    }
                    //During PCL calibration:
                    if (d_enable_pcl) {
                        //For now only one pointcloud is used to calibrate (TODO: Calc average transform over multiple pointclouds?):
                        if (d_latest_points_time > d_calibration_start_time && d_last_cal_points_time < d_calibration_start_time) {
                            d_last_cal_points_time = ros::Time::now();
                            for (auto & mask : d_masks) {
                                cout << "PCL Calibrating, mask: " << mask.getLabel() << endl;
                                mask.getPclDetection().calibrateCloud(d_latest_points);

                                //Publish transformed cloud for debugging: 
                                sensor_msgs::PointCloud2 cloud_msg;
                                pcl::toROSMsg(*mask.getPclDetection().getCalibrationCloud(), cloud_msg); 
                                cloud_msg.header.frame_id = "mask_points";
                                //d_masks_points_pub.publish(cloud_points_projected);
                                d_masks_points_pub.publish(cloud_msg);
                            }
                        } else {
                            for (auto & mask : d_masks) {
                                //Calibrate for background noise:
                                mask.getPclDetection().calibrateBackground(d_latest_points);
                            }
                        }
                    }
                }
                break;
            case DetectNodeMode::DETECT:
                if ((((current_time - d_latest_rgb_time.toSec()) > timeout) && d_enable_rgb) 
                        || (((current_time - d_latest_depth_time.toSec()) > timeout) && d_enable_depth)) {
                    ROS_ERROR("Can't detect; latest RGB or Depth image is too old! Switching to IDLE mode.");
                    ROS_ERROR("timeout: %.3f", timeout);
                    if (d_enable_rgb) {
                        ROS_ERROR("latest rgb time: %.3f", d_latest_rgb_time.toSec());
                    }
                    if (d_enable_depth) {
                        ROS_ERROR("latest depth time: %.3f", d_latest_depth_time.toSec());
                    }
                    ROS_ERROR("time: %.3f", current_time);

                    d_mode = DetectNodeMode::IDLE;
                } else if (d_enable_rgb && !d_dd_rgb.hasValidBackgroundModel()) {
                    ROS_ERROR("Can't do RGB DD detect; no valid DD background model for RGB (didn't calibrate while RGB DD was enabled?).");
                    d_mode = DetectNodeMode::IDLE;
                } else if (d_enable_depth && !d_dd_depth.hasValidBackgroundModel()) {
                    ROS_ERROR("Can't do Depth DD detect; no valid DD background model for Depth (didn't calibrate while Depth DD was enabled?).");
                    d_mode = DetectNodeMode::IDLE;
                } else if ((d_enable_rgb && d_enable_depth && d_latest_rgb_time.toSec() > d_last_detect_image_time.toSec() && d_latest_depth_time.toSec() > d_last_detect_image_time.toSec())
                        || (d_enable_rgb && !d_enable_depth && d_latest_rgb_time.toSec() > d_last_detect_image_time.toSec())
                        || (!d_enable_rgb && d_enable_depth && d_latest_depth_time.toSec() > d_last_detect_image_time.toSec())
                        || (d_enable_pcl && d_latest_points_time.toSec() > d_last_detect_image_time.toSec())) {
                    //TODO: Check if RGB and Depth are of the same resolution, otherwise enlarge one of them!
                    if (d_enable_depth && d_enable_rgb) {
                        if (d_dd_rgb.getHeight() != d_dd_rgb.getHeight() || d_dd_rgb.getWidth() != d_dd_depth.getWidth()) {
                            ROS_ERROR("It is currently not possible to combine depth and rgb of different sizes, sorry.");
                        }
                    }

                    //Trigger PCL detection, run as separate threads:
                    vector<thread> pcl_threads;
                    if (d_enable_pcl) {
                        for (auto & mask : d_masks) {
                            thread pcl_thread(&PclDetection::detectSurfaceChange, &mask.getPclDetection(), d_latest_points);
                            pcl_threads.push_back(move(pcl_thread));
                            //mask.getPclDetection().detectSurfaceChange(d_latest_points); //Uncomment to run in serial
                        }
                    }

                    d_last_detect_image_time = ros::Time::now(); //TODO: Verify where when this has be set.

                    if (d_enable_rgb || d_enable_depth) {
                        //Change detection compared to calibrated background model:
                        calcAndPublishChange(d_dd_rgb, d_dd_depth, 
                                d_visualization_rgb_dev_pub, d_visualization_depth_dev_pub, d_visualization_image_cc_pub, 
                                d_polygons_pub, false);
                    }

                    //Motion detection compared to x seconds ago:
                    //First filter and recalculate:
                    if (d_enable_rgb) {
                        d_dd_rgb_motion.filter(current_time);
                        d_dd_rgb_motion.calcBackgroundModel();
                    }
                    if (d_enable_depth) {
                        d_dd_depth_motion.filter(current_time);
                        d_dd_depth_motion.calcBackgroundModel();
                    }
                    //Detect (if possible):
                    if (d_enable_rgb || d_enable_depth) {
                        if ((d_enable_rgb && d_enable_depth && d_dd_rgb_motion.hasValidBackgroundModel() && d_dd_depth_motion.hasValidBackgroundModel())
                                || (d_enable_rgb && !d_enable_depth && d_dd_rgb_motion.hasValidBackgroundModel())
                                || (!d_enable_rgb && d_enable_depth && d_dd_depth_motion.hasValidBackgroundModel())) {
                            calcAndPublishChange(d_dd_rgb_motion, d_dd_depth_motion, 
                                    d_visualization_rgb_motion_dev_pub, d_visualization_depth_motion_dev_pub, d_visualization_image_cc_motion_pub, 
                                    d_polygons_motion_pub, true);
                        } else {
                            ROS_WARN("Can't do motion detection; no depth and/or rgb received for the last %.3f seconds.", d_dd_rgb_motion.getMaxAge());
                        }
                    }
                    //Add new images to background model:
                    if (d_enable_rgb) {
                        d_dd_rgb_motion.addBackgroundImage(d_latest_rgb_image, 0, current_time);
                    }
                    if (d_enable_depth) {
                        d_dd_depth_motion.addBackgroundImage(d_latest_depth_image, 0, current_time);
                    }

                    //Wait for PCL detection threads to finish:
                    std_msgs::Float64MultiArray vol_msg;
                    if (d_enable_pcl) {
                        for (auto & pcl_thread : pcl_threads) {
                            pcl_thread.join();
                        }
                        for (auto & mask : d_masks) {
                            vol_msg.data.push_back(mask.getPclDetection().getLastMovingAverageVolume());
                        }
                    }
                    d_volumes_pub.publish(vol_msg);

                    //Look for events in masks and publish them:
                    eventNotification();
                }
                break;
            default:
                throw DeviationDetectionException("Internal bug; in unknown state!");
        }
    }
}
