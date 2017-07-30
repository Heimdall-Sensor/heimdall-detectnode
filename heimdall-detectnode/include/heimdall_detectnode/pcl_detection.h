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
#ifndef _HEIMDALL_PCL_DETECTION_
#define _HEIMDALL_PCL_DETECTION_

#include <exception>
#include <vector>
#include <iostream>
#include <math.h>
#include <assert.h>

#include <heimdall_msgs/GetMaskSrv.h>
#include <heimdall_detectnode/mask.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

using namespace cv;
using namespace std;

namespace Heimdall
{

    //Forward declaration of Mask:
    class Mask;

    /**
     * Activity detection related to a mask using PCL.
     */
    class PclDetection
    {
        public:
            PclDetection(Mask * mask);

            void setMask(Mask * mask) {
                d_mask = mask;
            }

            /**
             * Extracts those points within the pointcloud which belong to the mask.
             * WARNING: Assumes that the indices of the provided cloud equals and alignes to those of the mask.
             * @param cloud         The original cloud to extract the points from.
             * @param cloud_segment The segmented cloud extracted from the original cloud using the mask.
             */
            void extractCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segment);

            /**
             * Colors the mask inside the specified cloud.
             */
            void colorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, int r, int g, int b);

            /**
             * Returns true if the mask has been calibrated using the PCL cloud, false otherwise.
             */
            bool hasCalibratedCloud(void) {
                return d_has_calibrated_cloud;
            }

            /**
             * Calibrates the mask using the provided cloud.
             * WARNING: Assumes that the mask represents a (nearly) flat surface!
             * Basically exists out of the following steps:
             * - Extract masked cloud segment from provided cloud.
             * - Remove any nan points.
             * - Transform segmented cloud to origin.
             * - Determine orientation of resulting transform.
             * - Determine max/min x/y position (mask is reduced to a rectangle).
             */
            void calibrateCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud);

            /**
             * Detects any change (e.g., something on top/down the surface) of this mask providing the specified cloud.
             */
            void detectSurfaceChange(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr getTransformedCloud(void) {
                return d_transformed_cloud;
            }

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCalibrationCloud(void) {
                return d_calibration_cloud;
            }

            /**
             * Calculates the (estimated) volume (in m^3) of the points on top (relative to the camera) of the surface.
             */
            double calculateVolume(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud);

            /**
             * Calculates the volume of a point cloud with additional points projected on the Z axis.
             * It makes a concave Hull around it and calculates the volume
             */
            double convexHullVolume(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud);


            /**
             * Returns the centroid of the mask.
             */

            /**
             * Calculates average offset over incrementally.
             * To be called after calibrateCloud(...).
             */

            void calibrateOffset(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud);

            double getMovingAverageVolume(double new_volume);
            double getLastMovingAverageVolume() {
                return d_last_moving_average_volume;
            }

            Eigen::Vector4f getCentroid() {
                return d_centroid;
            }

            /**
             * Sets min. threshold for volume calculation.
             */
            void setMinVolumeCalc(double val) {
                d_x_min_config = val;
                //TODO: Transform calibration cloud, rather than doing this all the time:
                if (d_surface_reversed) {
                    d_x_max = val * -1;
                } else {
                    d_x_min = val;
                }
            }
            /**
             * Sets max. threshold for volume calculation.
             */
            void setMaxVolumeCalc(double val) {
                d_x_max_config = val;
                //TODO: Transform calibration cloud, rather than doing this all the time:
                if (d_surface_reversed) {
                    d_x_min = val * -1;
                } else {
                    d_x_max = val;
                }
            }


            /** 
             * Filter points not within limits and not above the calibrated surface:
             */
            void filterCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & full_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & filtered_cloud);


            /** 
             * Used to create a model for background noise to filter out during detection.
             */
            void addBackground(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud);

            /** 
             * Used to create a model for background noise to filter out during detection.
             */
            void calcBackgroundModel(void);

            void calibrateBackground(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud);

            /** 
             * Remove points near background model
             */
            void filterBackground(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & full_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & filtered_cloud);

            /**
             * Tries to find the current masked surface in specified cloud.
             */
            void findSurface(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud);

        private:
            Mask * d_mask;

            bool d_has_calibrated_cloud;
            //The original transformed cloud used for calibration:
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr d_full_calibration_cloud;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr d_calibration_cloud;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr d_background_cloud;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr d_transformed_cloud; //For debugging purposes
            Eigen::Matrix4f d_projection_transform;
            double d_surface_y_min, d_surface_y_max; //Mask surface min/max in Y
            double d_surface_z_min, d_surface_z_max; //Mask min/max in Y

            bool d_apply_nn_filtering = true;

            bool d_surface_reversed;
            double d_volume_offset;
            //Whether or not there is somebody or something occupying the surface:
            bool d_occupied;
            bool d_occupied_neg;

            double d_x_min_config;
            double d_x_max_config;
            //Bounding box as found after PCA (min/max x are reversed if d_surface_reversed == true):
            double d_x_min;
            double d_x_max;
            double d_y_min;
            double d_y_max;
            double d_z_min;
            double d_z_max;

            double d_width;
            double d_length;

            double d_mean_point_surface;
            double d_volume_offset_total;
            int d_volume_offset_count;

            double d_last_moving_average_volume;
            vector<double> d_volume_val_list;
            vector<double> d_volume_time_list;

            Eigen::Vector4f d_centroid;
    };
}

#endif // _HEIMDALL_PCL_DETECTION_

