#ifndef _HEIMDALL_RVIZ_VISUALIZER_NODE_
#define _HEIMDALL_RVIZ_VISUALIZER_NODE_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64MultiArray.h>
#include <heimdall_detectnode/mask.h>

#include <heimdall_msgs/CommandSrv.h>
#include <heimdall_msgs/AddMaskSrv.h>
#include <heimdall_msgs/GetMaskSrv.h>
#include <heimdall_msgs/GetMaskListSrv.h>
#include <heimdall_msgs/RemoveMaskListSrv.h>

#include <heimdall_msgs/Masks.h>

#include <std_msgs/String.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

using namespace std;

using namespace cv;

namespace Heimdall
{

    enum class RvizVisualizerNodeMode {};

    class RvizVisualizerNodeException: public std::exception
    {
        public:
            RvizVisualizerNodeException(char const *msg) : d_msg(msg) {}
            RvizVisualizerNodeException(std::string const &msg) : d_msg(msg) {}
            ~RvizVisualizerNodeException() noexcept {};

            char const *what() const noexcept { return d_msg.c_str(); };
        private:
            std::string const d_msg;
    };

    class RvizVisualizerNode
    {
        public:

            /**
             * @param nh            The node handler to use.
             * @param input_image   The topic of the images to use for both calibration and detection.
             */
            DetectNode(ros::NodeHandle &nh, std::string const & rgb_topic, std::string const & depth_topic, std::string points_topic);

            ~DetectNode();

            /**
             * Callback function, whenever an image from the input topic is received.
             * Enables detection if set using /dd/detect topic.
             */
            void rgbCallback(const sensor_msgs::ImageConstPtr& msg);

            /**
             * Callback function, whenever an image from the depth topic is received.
             * Enables detection if set using /dd/detect topic.
             */
            void depthCallback(const sensor_msgs::ImageConstPtr& msg);

            /**
             * Callback funtion on points topic.
             */
            void pointsCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud);

            /**
             * Initiates calibration.
             * @param   true    If calibration could be started succcesfully, false otherwise.
             */
            bool calibrate(void);
            
            bool run();

            /**
             * Sets the calibration period out in seconds.
             */
            void setCalibrationPeriod(float period) {
                d_calibration_period = period;
            }

            /**
             * @return      A string specifying the RGB encoding being used.
             * @return      ""  If no encoding has been set.
             */
            const std::string getRgbEncoding(void) {
                return d_rgb_encoding;
            }

            /**
             * @return      A string specifying the Depth encoding being used.
             * @return      ""  If no encoding has been set.
             */
            const std::string getDepthEncoding(void) {
                return d_depth_encoding;
            }

            void publishMasks(void);

            /**
             * Dynamic reconfigure callback function.
             * Called whenever a change in the configuration is requested using dynamic reconfigure.
             */
            void configCallback(heimdall_detectnode::DetectNodeConfig &config, uint32_t level);

            bool removeMaskListService(heimdall_msgs::RemoveMaskListSrv::Request &req, heimdall_msgs::RemoveMaskListSrv::Response &res);

            bool commandService(heimdall_msgs::CommandSrv::Request &req, heimdall_msgs::CommandSrv::Response &res);

            bool addMaskService(heimdall_msgs::AddMaskSrv::Request &req, heimdall_msgs::AddMaskSrv::Response &res);

            bool getMaskService(heimdall_msgs::GetMaskSrv::Request &req, heimdall_msgs::GetMaskSrv::Response &res);

            bool getMaskListService(heimdall_msgs::GetMaskListSrv::Request &req, heimdall_msgs::GetMaskListSrv::Response &res);

            /**
             * Initiates detection.
             */
            void detect(void);

            /**
             * Stops detection.
             */
            void idle(void);

            /**
             * Adds the specified mask to the list of possible detection masks.
             */
            void addMask(string & label, Mat & mask_img);

            /**
             * Removes the specified mask.
             * @return  true    If the specified mask was found and removed, false otherwise.
             */
            void removeMask(string & label);

            void publishImage(image_transport::Publisher & pub, Mat & image, const std::string & encoding);

            void threshold(Mat & image, Mat & thres_image, DeviationDetection & dd, image_transport::Publisher & pub, double threshold, Mat & dev_image);

            void calcAndPublishChange(DeviationDetection & dd_rgb, DeviationDetection & dd_depth, 
                    image_transport::Publisher & rgb_dev_pub, image_transport::Publisher & depth_dev_pub, image_transport::Publisher & image_cc_pub,
                    ros::Publisher & polygons_pub, bool detect_activity);

            /**
             * Detect changes in masks and corresponding borders.
             */
            double detectActivity(Mat & img_change, Mat & dev_depth_image);

            /**
             * Maximum age (in seconds) to determine any motions.
             */
            double getMotionAge(void) {
                return d_motion_age;
            }

            /**
             * Finds the mask with the specified label.
             * @return  The index of the mask in the mask list.
             * @return  -1 If the mask could not be found.
             */
            int findMask(const string & label);

            bool hasMask(const string & label) {
                return findMask(label) >= 0;
            }

            /**
             * Look for events in masks and publish them.
             */
            void eventNotification(void);

            /**
             * Saves currently loaded masks to <package_root>/masks/.
             */
            void saveMasks(void);

            /**
             * Removes current loaded masks and loads new masks from <package_root>/masks/.
             */
            void loadMasks(void);

            /**            * Removes all masks.
             */
            void removeMasks(void) {
                d_masks.clear();
            }

            /**
             * Sets the distance threshold for all masks if dd_detection is used. 
             */
            void setMaskDistanceThreshold(double threshold) {
                d_mask_distance_threshold = threshold;
                for (vector<Mask>::size_type mask_idx = 0; mask_idx < d_masks.size(); mask_idx++) {
                    d_masks[mask_idx].getDdDetection().setDistanceThreshold(threshold);
                }
            }

        private:
            ros::NodeHandle d_nh;

            image_transport::ImageTransport d_it;

            //DD Detection:
            bool d_enable_depth;
            bool d_enable_rgb;
            //PLC Detection:
            bool d_enable_pcl;

            bool d_enable_visualization;

            //Used to publish location (index of the mask) of deviation:
            ros::Publisher d_location_pub;

            ros::Time d_last_detect_image_time;

            //General RGB:
            Mat d_latest_rgb_image;
            image_transport::Subscriber d_rgb_image_sub;
            std::string d_rgb_encoding;
            ros::Time d_latest_rgb_time;

            //General Depth:
            Mat d_latest_depth_image;
            image_transport::Subscriber d_depth_image_sub;
            std::string d_depth_encoding;
            ros::Time d_latest_depth_time;

            //General Pointcloud:
            pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr d_latest_points; 
            ros::Subscriber d_points_sub;
            ros::Time d_latest_points_time; 
            ros::Time d_last_cal_points_time;

            //DD RGB:
            ros::Time d_last_cal_rgb_time;
            DeviationDetection d_dd_rgb;
            double d_rgb_dev_threshold;
            image_transport::Publisher d_visualization_rgb_dev_pub;

            //DD Depth:
            ros::Time d_last_cal_depth_time;
            DeviationDetection d_dd_depth;
            double d_depth_dev_threshold;
            image_transport::Publisher d_visualization_depth_dev_pub;

            //DD RGB Motion:
            DeviationDetection d_dd_rgb_motion;
            image_transport::Publisher d_visualization_rgb_motion_dev_pub;

            //DD Depth Motion:
            DeviationDetection d_dd_depth_motion;
            image_transport::Publisher d_visualization_depth_motion_dev_pub;

            double d_motion_age;

            //Misc:
            ros::Rate d_rate;

            DetectNodeMode d_mode;

            ros::Time d_calibration_start_time;

            ros::Publisher d_polygons_pub;
            ros::Publisher d_polygons_motion_pub;

            int d_next_cal_condition;

            float d_calibration_period;

            image_transport::Publisher d_visualization_image_cc_pub;
            image_transport::Publisher d_visualization_image_cc_motion_pub;

            dynamic_reconfigure::Server<heimdall_detectnode::DetectNodeConfig> d_config_server;

            int d_dilate_size;
            int d_erode_size;

            int d_rgb_count;
            int d_depth_count;

            double d_cc_size_threshold;

            ros::ServiceServer d_command_service;
            ros::ServiceServer d_add_mask_service;
            ros::ServiceServer d_get_mask_service;
            ros::ServiceServer d_get_mask_list_service;
            ros::ServiceServer d_remove_mask_service;

            //Detection masks (and borders) for activity detection within specific regions:
            std::vector<Mask> d_masks;

            ros::Publisher d_activities_pub;
            int d_activity_header_seq;

            double d_motion_duration;

            ros::Publisher d_masks_pub;

            ros::Publisher d_notification_pub;

            ros::Publisher d_activity_pub;

            ros::Publisher d_masks_points_pub;

            double d_mask_distance_threshold;
    };
}

#endif // _HEIMDALL_DD_NODE_
