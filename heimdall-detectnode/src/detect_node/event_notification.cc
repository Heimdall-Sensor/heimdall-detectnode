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
    void DetectNode::eventNotification(void) {
        //Look for events in masks and publish them:
        //TODO: Include throttling, mask info, etc.
        for (vector<Mask>::size_type mask_idx = 0; mask_idx < d_masks.size(); mask_idx++) {
            vector<MaskEvent> events = d_masks[mask_idx].getEvents();

            //TODO: Move this stuff to Mask code? 
            string mask_type = d_masks[mask_idx].getType();
            
            //TODO: A bit of a hacky implementation; improve:
            if (mask_type == "door") {
                MaskEventType pre_event_type = MaskEventType::NONE;
                double pre_event_time = 0.0;
                for (vector<MaskEvent>::size_type event_idx = 0; event_idx < events.size(); event_idx++) {

                    //Going through the door away from the camera? 
                    if (events[event_idx].type == MaskEventType::IN_MASK_NEG && 
                            pre_event_type == MaskEventType::OUT_MASK) {
                        cout << "Person left through door!" << endl;
                        if ((events[event_idx].time - pre_event_time) < 2.5) {
                            cout << "Within time limit!" << endl;
                            std_msgs::String msg;
                            msg.data = (boost::format("{\"message\":\"A person has left the room through the %1%!\", \"level\":\"HIGH\"}") % d_masks[mask_idx].getName()).str();
                            d_notification_pub.publish(msg);
                        }
                    }
                    //Going through the door towards the camera? 
                    if (events[event_idx].type == MaskEventType::IN_MASK && 
                            pre_event_type == MaskEventType::OUT_MASK_NEG) {
                        cout << "Person entered through door!" << endl;
                        if ((events[event_idx].time - pre_event_time) < 2.5) {
                            cout << "Within time limit!" << endl;
                            std_msgs::String msg;
                            msg.data = (boost::format("{\"message\":\"A person has entered the room through the %1%!\", \"level\":\"HIGH\"}") % d_masks[mask_idx].getName()).str();
                            d_notification_pub.publish(msg);
                        }
                    }
                    
                    pre_event_time = events[event_idx].time;
                    pre_event_type = events[event_idx].type;
                }
                //TODO: Take into account of multiple ins/outs:
                if (events.size() > 0 
                        && events[events.size() - 1].type != MaskEventType::OUT_MASK
                        && events[events.size() - 1].type != MaskEventType::OUT_MASK_NEG) {
                    d_masks[mask_idx].clearEvents();
                }
            } else {
                for (vector<MaskEvent>::size_type event_idx = 0; event_idx < events.size(); event_idx++) {
                    switch (events[event_idx].type) {
                        case MaskEventType::IN_MASK:
                            if (mask_type == "bed" || mask_type == "chair") {
                                std_msgs::String msg;
                                msg.data = (boost::format("{\"message\":\"A person has entered the %1%!\", \"level\":\"NORMAL\"}") % d_masks[mask_idx].getName()).str();
                                d_notification_pub.publish(msg);
                            } else if (mask_type == "floor") {
                                std_msgs::String msg;
                                msg.data = (boost::format("{\"message\":\"A person is lying down on the %1%!\", \"level\":\"HIGH\"}") % d_masks[mask_idx].getName()).str();
                                d_notification_pub.publish(msg);
                            } else {
                                std_msgs::String msg;
                                msg.data = (boost::format("{\"message\":\"Activity towards the %1% detected!\", \"level\":\"HIGH\"}") % d_masks[mask_idx].getName()).str();
                                d_notification_pub.publish(msg);
                            }
                            break;
                        case MaskEventType::OUT_MASK:
                            if (mask_type == "bed" || mask_type == "chair") {
                                std_msgs::String msg;
                                msg.data = (boost::format("{\"message\":\"A person has left the %1%!\", \"level\":\"HIGH\"}") % d_masks[mask_idx].getName()).str();
                                d_notification_pub.publish(msg);
                            } else if (mask_type == "floor") {
                                //Ignoring...
                            } else {
                                std_msgs::String msg;
                                msg.data = (boost::format("{\"message\":\"Activity towards the %1% detected!\", \"level\":\"HIGH\"}") % d_masks[mask_idx].getName()).str();
                                d_notification_pub.publish(msg);
                            }
                            break;
                        case MaskEventType::MASK:
                            if (mask_type == "bed") {
                                std_msgs::String msg;
                                msg.data = (boost::format("{\"message\":\"Disturbance inside the %1% detected!\", \"level\":\"HIGH\"}") % d_masks[mask_idx].getName()).str();
                                d_notification_pub.publish(msg);
                            }
                            break;
                        default:
                            break;
                    }
                }
                d_masks[mask_idx].clearEvents();
            }
        }
    }
}
