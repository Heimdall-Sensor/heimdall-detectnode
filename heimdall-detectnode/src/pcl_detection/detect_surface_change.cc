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
    void PclDetection::detectSurfaceChange(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud) {
        //Transform cloud:
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*cloud, *transformed_cloud, d_projection_transform);
        d_transformed_cloud = transformed_cloud;
        //Filter (on bounding box):
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        filterCloud(transformed_cloud, filtered_cloud);

        //Count the number of points within the bounding box:
        double volume = calculateVolume(filtered_cloud) - d_volume_offset;
        volume = getMovingAverageVolume(volume);
        d_mask->publishDiff(volume);

        //Detect change (and add event):
        double upper_bound = d_mask->getUpperBound();
        double lower_bound = d_mask->getLowerBound();

        bool occupied = d_occupied;
        if (volume > upper_bound) {
            occupied = true;
        } else if (volume < lower_bound) {
            occupied = false;
        }
        if (occupied != d_occupied) {
            //Report event:
            if (occupied) {
                cout << d_mask->getLabel() << " IN event surface detected!" << endl;
                MaskEvent event(MaskEventType::IN_MASK, ros::Time::now().toSec());
                d_mask->getEvents().push_back(event); //Currently no lock needed since there is only one thread per mask.
            } else {
                cout << d_mask->getLabel() << " OUT event surface detected!" << endl;
                MaskEvent event(MaskEventType::OUT_MASK, ros::Time::now().toSec());
                d_mask->getEvents().push_back(event);
            }
        }
        d_occupied = occupied;

        bool occupied_neg = d_occupied_neg;
        if (volume < -upper_bound) {
            occupied_neg = true;
        } else if (volume > -lower_bound) {
            occupied_neg = false;
        }
        if (occupied_neg != d_occupied_neg) {
            //Report event:
            if (occupied_neg) {
                cout << d_mask->getLabel() << " IN NEG event surface detected!" << endl;
                MaskEvent event(MaskEventType::IN_MASK_NEG, ros::Time::now().toSec());
                d_mask->getEvents().push_back(event); //Currently no lock needed since there is only one thread per mask.
            } else {
                cout << d_mask->getLabel() << " OUT NEG event surface detected!" << endl;
                MaskEvent event(MaskEventType::OUT_MASK_NEG, ros::Time::now().toSec());
                d_mask->getEvents().push_back(event);
            }
        }
        d_occupied_neg = occupied_neg;
    }
}
