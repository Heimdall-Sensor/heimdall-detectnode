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
#include <heimdall_detectnode/detect_node.h>
#include <sstream>
#include <ros/console.h>

using namespace Heimdall;
using namespace std;


void print_usage(void) {
    cout << "Usage:" << endl;
    cout << "   rosrun detect_node rgb:=<rgb_image_topic> depth:=<depth_image_topic> points:=<cloud_points_topic>" << endl;
    cout << "Subscribes to:" << endl;
    cout << "    \"/dd/calibrate\": To initiate calibration for a new condition." << endl;
    cout << "    \"/dd/detect\":    To enable/disable detection (will first calculate background model)." << endl;
    cout << "Publishes to:" << endl;
    cout << "    \"/dd/location\":  Indicates the location (mask index) of the detected deviation." << endl;
    cout << "To test using command line:" << endl;
    cout << "    rosservice call /dd/command calibrate" << endl;
    cout << "    rosservice call /dd/command detect:on" << endl;
}


int check_arg(string argument) {
    if (ros::names::remap(argument) == argument) {
        ROS_ERROR("No \"%s\" argument specified!", argument.c_str());
        return 1;
    }
    return 0;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "detect_node");
    ros::NodeHandle nh; 

    //TODO: Remove in production code:
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    if (check_arg("points")) {
        print_usage(); 
        return 1;
    }

    DetectNode node(nh, ros::names::remap("rgb"), ros::names::remap("depth"), ros::names::remap("points"));

    while (ros::ok()) {
        //double start_time = ros::Time::now().toSec();
        ros::spinOnce();
        node.run();
        //cout << "Loop Freq.: " << 1 / (ros::Time::now().toSec() - start_time) << endl;
    }

    return 0;
}
