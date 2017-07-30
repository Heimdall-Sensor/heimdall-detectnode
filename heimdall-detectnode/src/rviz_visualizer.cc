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
#include <ros/ros.h>
#include <sstream>
#include <ros/console.h>

#include <visualization_msgs/Marker.h>

//using namespace Heimdall;
using namespace std;


int main(int argc, char **argv) {
    //TODO: Remove/move this test code.
    //Simple test to add markers in rviz:

    ros::init(argc, argv, "heimdall_dd_node");
    ros::NodeHandle nh; 

    //TODO: Remove in production code:
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    bool published = false;

    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        //if (! published) {
        //if (true) {
        for (int i = 0; i < 10; i++) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time();
            //marker.ns = "my_namespace";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = i;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 1;
            marker.scale.y = 0.1;
            marker.scale.z = 1.0;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.text = "Hello World!";
            //only if using a MESH_RESOURCE marker type:
            //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
            vis_pub.publish( marker );

            published = true;
        }

        rate.sleep();
    }

    return 0;
}
