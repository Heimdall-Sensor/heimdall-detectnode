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
    void PclDetection::findSurface(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud) {
        cout << "Finding surface for mask: " << d_mask->getLabel() << endl;

        //Transform cloud:
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*cloud, *transformed_cloud, d_projection_transform);

        //Filter out points on a different height:
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        vector<int> original_indices; int point_idx = 0;
        for (auto & point : transformed_cloud->points) {
            if (! (isnan(point.x) || isnan(point.y) || isnan(point.z))) {
                if (point.x < 0.05 && point.x > -0.05) {
                    filtered_cloud->points.push_back(point);
                    original_indices.push_back(point_idx);
                }
            }
            point_idx++;
        }

        //Apply clustering on resulting plane:
        //For more info: http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(filtered_cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(0.07); //cm
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(55000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(filtered_cloud);
        ec.extract(cluster_indices);

        int pre_size = d_full_calibration_cloud->points.size();
        cout << "Calibrated size: " << pre_size << endl;
        //Apply PCA for each cluster and determine width and height:
        int j = 0;
        double best_match_surface_diff = -1;
        int best_match_cluster_idx = -1;
        for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
            //Construct cloud cluster:
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
            for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                cloud_cluster->points.push_back(filtered_cloud->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            cout << "Cluster(" << j << "): " << cloud_cluster->points.size() << " has points." << endl;

            //Apply PCA:
            //TODO: Mostly duplicate code as in calibrateCloud(...)
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud_cluster, centroid);
            //Convariance matrix:
            Eigen::Matrix3f covariance_matrix;
            computeCovarianceMatrixNormalized(*cloud_cluster, centroid, covariance_matrix);
            //Calculate eigen vectors:
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix, Eigen::ComputeEigenvectors);
            Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
            eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));  
            //Transform the original cloud to the origin where the principal components correspond to the axes:
            Eigen::Matrix4f projection_transform(Eigen::Matrix4f::Identity());
            projection_transform.block<3, 3>(0, 0) = eigen_vectors.transpose();
            projection_transform.block<3, 1>(0, 3) = -1.f * (projection_transform.block<3, 3>(0, 0) * centroid.head<3>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud(*cloud_cluster, *cloud_cluster_projected, projection_transform);
            //Get the minimum and maximum points of the transformed cloud:
            pcl::PointXYZRGB min_point, max_point;
            pcl::getMinMax3D(*cloud_cluster_projected, min_point, max_point);

            double cluster_surface = abs(max_point.y - min_point.y) * abs(max_point.z - min_point.z);

            double cluster_surface_diff = abs(cluster_surface - d_width * d_length);
            cout << "Cluster surface diff: " << cluster_surface_diff << endl;
            if (best_match_cluster_idx < 0 or cluster_surface_diff < best_match_surface_diff) {
                int size = cloud_cluster->points.size();
                if (size > 0.4 * pre_size and size < 1.6 * pre_size) {
                    best_match_cluster_idx = j;
                    best_match_surface_diff = cluster_surface_diff;
                }
            } 

            j++;
        }
        if (best_match_cluster_idx >= 0) {
            cout << "Cluster " << best_match_cluster_idx << " is the best match!" << endl;
        } else {
            cout << "No suitable surface found, sorry!" << endl;
            return;
        }

        //Select best matching cluster and modify mask using the point indices:
        //TODO: Mostly duplicate code as in extractCloud(...)
        Mat & mask = d_mask->getModifiableMask();
        //Depth can be QVGA while mask (RGB) is VGA:
        bool qvga = mask.rows * mask.cols == cloud->points.size() * 4;
        if (qvga) {
            cout << "Assuming Depth is QVGA and Color is VGA." << endl;
        }
        assert(mask.rows * mask.cols == cloud->points.size() or qvga);
        //Reset mask and fill in ones at the indices of the matching cluster:
        int pi = 0;
        for (int i = 0; i < mask.rows; i++) {
            if (qvga and i % 2 == 1) {
                //Copy every other row in mask (depth is QVGA):
                for (int j = 0; j < mask.cols; j++) {
                    mask.at<uchar>(i, j) = mask.at<uchar>(i-1, j);
                }
            } else {
                for (int j = 0; j < mask.cols; j++) {
                    if (qvga and j % 2 == 1) {
                        //Copy every other column in mask (depth is QVGA)
                        mask.at<uchar>(i, j) = mask.at<uchar>(i, j-1);
                    } else {
                        mask.at<uchar>(i, j) = 0; //Reset
                        //Pixel is not filtered?
                        //TODO Could be done more efficient by keeping track of indices or changing to loop structure:
                        int orig_idx_idx = 0; 
                        bool found = false;
                        for (auto & orig_idx : original_indices) {
                            if (pi == orig_idx) {
                                //Pixel part of found cluster?
                                for (vector<int>::const_iterator pit = cluster_indices[best_match_cluster_idx].indices.begin(); 
                                        pit != cluster_indices[best_match_cluster_idx].indices.end(); ++pit) {
                                    if (orig_idx_idx == *pit) {
                                        mask.at<uchar>(i, j) = 1;
                                        found = true;
                                        break;
                                    }
                                }
                            }
                            if (found) {
                                break;
                            }
                            orig_idx_idx++;
                        }
                        pi++;
                    }
                }
            }
        }
    }
}
