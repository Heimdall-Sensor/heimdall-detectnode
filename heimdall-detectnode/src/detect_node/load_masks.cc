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
    void DetectNode::loadMasks(void) {
        string mask_path((boost::format("%1%/masks/") % ros::package::getPath("heimdall_detectnode")).str());
        fs::path masks_dir(mask_path);
	ROS_INFO("Loading masks from %s", mask_path.c_str());
        fs::directory_iterator it(masks_dir), eod;
        //Iterate over all files under ./masks/ dir: 
        BOOST_FOREACH(fs::path const & p, std::make_pair(it, eod)) { 
            if (is_regular_file(p)
                    && p.string().find('.') != string::npos
                    && p.string().substr(p.string().find('.')) == ".bmp") {
                //Read mask image:
                Mat mask_img = imread(p.string(), CV_8U);
                //Extract base name to be used as label:
                string label = string(
                            p.string().substr(
                                p.string().find_last_of('/') + 1, 
                                p.string().find('.') - (p.string().find_last_of('/') + 1)
                            )
                        );
                addMask(label, mask_img);
            } else {
                //Ignore .holder file:
                if (p.string().substr(p.string().find('.')) != ".holder") {
                    //TODO: Not working due to BOOST_FOREACH marco; replace BOOST_FOREACH?
                    //ROS_WARN("Incorrect mask file: %s, ignoring...", p.string());
                    cerr << "Incorrect mask file: " << p.string() << ", ignoring..." << endl;
                }
            }
        }
        publishMasks();
    }
}
