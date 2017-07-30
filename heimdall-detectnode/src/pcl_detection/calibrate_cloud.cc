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
    void PclDetection::calibrateCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segment(new pcl::PointCloud<pcl::PointXYZRGB>);
        extractCloud(cloud, cloud_segment);
        //Outlier removal:
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radius_outlier_removal;
        radius_outlier_removal.setInputCloud(cloud_segment);
        radius_outlier_removal.setRadiusSearch(0.1);
        radius_outlier_removal.setMinNeighborsInRadius(10);
        radius_outlier_removal.filter(*cloud_segment);

        //Transform mask to origin using PCA:
        //This implementation has been inspired by:
        //http://codextechnicanum.blogspot.nl/2015/04/find-minimum-oriented-bounding-box-of.html
        //http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html
        vector<double> dimensions;
        //Eigen::Vector4f centroid;

        //Centroid:
        pcl::compute3DCentroid(*cloud_segment, d_centroid);

        //Convariance matrix:
        Eigen::Matrix3f covariance_matrix;
        computeCovarianceMatrixNormalized(*cloud_segment, d_centroid, covariance_matrix);

        //Calculate eigen vectors:
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
        eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));  

        //Transform the original cloud to the origin where the principal components correspond to the axes:
        Eigen::Matrix4f projection_transform(Eigen::Matrix4f::Identity());
        projection_transform.block<3, 3>(0, 0) = eigen_vectors.transpose();
        projection_transform.block<3, 1>(0, 3) = -1.f * (projection_transform.block<3, 3>(0, 0) * d_centroid.head<3>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_points_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*cloud_segment, *cloud_points_projected, projection_transform);
        //Get the minimum and maximum points of the transformed cloud:
        pcl::PointXYZRGB min_point, max_point;
        pcl::getMinMax3D(*cloud_points_projected, min_point, max_point);
        const Eigen::Vector3f mean_diagonal = 0.5f * (max_point.getVector3fMap() + min_point.getVector3fMap());

        cout << "Min/max x: " << max_point.x << ", " << min_point.x << endl;
        cout << "Min/max y: " << max_point.y << ", " << min_point.y << endl;
        cout << "Min/max z: " << max_point.z << ", " << min_point.z << endl;

        double y_diff = abs(max_point.y - min_point.y);
        double z_diff = abs(max_point.z - min_point.z);
        if (y_diff > z_diff) {
            d_length = y_diff;
            d_width = z_diff;
        } else {
            d_length = z_diff;
            d_width = y_diff;
        }

        d_surface_y_min = min_point.y;
        d_surface_z_min = min_point.z;
        d_surface_y_max = max_point.y;
        d_surface_z_max = max_point.z;

        d_projection_transform = projection_transform;
        cout << "Transform:" << projection_transform << endl;

        d_full_calibration_cloud = cloud_points_projected;
        //Downsample calibration cloud (to speed up NN filtering):
        cout << "Number of points before VoxelGrid filtering: " << cloud_points_projected->points.size() << endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_calibration_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud_points_projected);
        sor.setLeafSize(0.05f, 0.05f, 0.05f);
        sor.filter(*downsampled_calibration_cloud);
        cout << "Number of points after VoxelGrid filtering: " << downsampled_calibration_cloud->points.size() << endl;
        d_calibration_cloud = downsampled_calibration_cloud;
        //Publish calibration cloud for debugging: 
        if (d_mask->isEnableDebugPoints()) {
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*d_calibration_cloud, cloud_msg); 
            cloud_msg.header.frame_id = "mask_points";
            d_mask->getPointsMaskPublisher().publish(cloud_msg);
        }

        //Estimate average square meters per point using NN radius approach:
        //TODO

        //Determine position of original origin after transformation (this to determine the orientation of the surface towards the camera):
        pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_origin_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::PointType origin_point;
        origin_point.x = 0.0; origin_point.y = 0.0; origin_point.z = 0.0;
        origin_cloud->points.push_back(origin_point);
        pcl::transformPointCloud(*origin_cloud, *transformed_origin_cloud, projection_transform);
        cout << "Origion after transformation:" << endl;
        for (auto & point : transformed_origin_cloud->points) {
            cout << point << endl;
        }
        //If X is negative after transformation of the origion, the surface top side is reversed.
        d_surface_reversed = transformed_origin_cloud->points[0].x < 0;

        //Determine bounding box for detection:
        //Define bounding box to count points in:
        double margin = 1.0;
        d_y_min = d_surface_y_min * margin;
        d_y_max = d_surface_y_max * margin;
        d_z_min = d_surface_z_min * margin;
        d_z_max = d_surface_z_max * margin;
        //Reverse bounding box if surfaces orientation is reversed:
        //TODO: Transform calibration cloud, rather than doing this all the time:
        if (d_surface_reversed) {
            d_x_max = d_x_min_config * -1;
            d_x_min = d_x_max_config * -1;
        } else {
            d_x_min = d_x_min_config;
            d_x_max = d_x_max_config;
        }

        //Estimate square meters per point:
        int onesCount = d_mask->countOnes();
        //TODO: Implement better surface estimation!
        //TODO: The stuff below only reliably works if mask is rectangular (due to w*h calculation):
        cout << "onesCount: " << onesCount << endl;
        d_mean_point_surface = ((d_y_max - d_y_min) * (d_z_max - d_z_min)) / onesCount; //Use original mask ones count due to filtered NaNs.
        cout << "d_mean_point_surface: " << d_mean_point_surface << endl;

        //Reset background calibration:
        {
            //Used to create a model for background noise to filter out during detection:
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr background_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            d_background_cloud = background_cloud;

            d_volume_offset = 0.0;
            d_volume_offset_total = 0.0;
            d_volume_offset_count = 0;

            d_has_calibrated_cloud = false; //Is only calibrated if background is calibrated as well.
        }
    }
}
