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
#ifndef _HEIMDALL_DD_
#define _HEIMDALL_DD_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <exception>
#include <vector>
#include <iostream>
#include <math.h>
#include <assert.h>

using namespace cv;
using namespace std;

namespace Heimdall
{
    class DeviationDetectionException : public std::exception
    {
        public:
            DeviationDetectionException(char const *msg) : d_msg(msg) {}
            DeviationDetectionException(std::string const &msg) : d_msg(msg) {}
            ~DeviationDetectionException() noexcept {};

            char const *what() const noexcept { return d_msg.c_str(); };
        private:
            std::string const d_msg;
    };

    /**
     * TODO: Update documentation:
     * Implementation of #1152:
     * - Whenever one pressed the calibrate function it stores a few background images for the given condition. 
     *   So image pressing the button a couple of time while turning on/off the lights, pulling down the blindings, etc.
     * - For each condition and for each pixel (possibly after down-sampling), calculate the mean value (of all or a single value) 
     *   and its standard deviation.
     * - For the current image and for each pixel (possibly after down-sampling), calculate the minimal difference to the corresponding condition. 
     *   I would suggest using HSV color space, in which case one should realize that H is circular (0 is around red, while 255 is also around red).
     * - Have separate binary masks to annotate regions of interest (we can create a nice annotation tool in the app later on).
     * - For a given amount of frames, and for each mask, calculate the average standard deviations difference to the background model. 
     *   The threshold can then we defined in terms of standard deviations rather than some meaningless threshold value.
     * - One could detect short term differences by doing the same as above, but creating a simple background model for a given amount of preceding frames.
     * TODO: Use linked list to improve performance?
     * TODO: Use temporary variables for sums to improve performance?
     */
    class DeviationDetection
    {
        public:

            DeviationDetection();

            //TODO: Move inline functions in separate cc files?

            /**
             * Stores the provided background image for the given condition.
             * Can only be used if isTimed() returns false.
             * @param   image       The background image to store.
             * @param   condition   The condition index to use (should not be higher than getConditionSize() - 1).
             */
            void addBackgroundImage(Mat & image, int condition);

            /**
             * Stores the provided background image for the given condition and time.
             * @param   image       The background image to store.
             * @param   condition   The condition index to use (should not be higher than getConditionSize() - 1).
             * @param   time        The number of seconds since epoch.
             */
            void addBackgroundImage(Mat & image, int condition, double time);

            /**
             * Prints information about the background model.
             */
            void printModelInfo(void) {
                for (std::vector<std::vector<Mat> >::size_type i = 0; i < d_bg_images.size(); i++) {
                    std::cout << "Condition " << i << ": " << d_bg_images[i].size() << std::endl;
                }
            }

            void setConditionSize(int size) {
                //TODO: Use size of vector instead of d_condition_size?
                d_condition_size = size;
                d_bg_images.resize(size); 
                d_bg_times.resize(size); 
            }

            int getConditionSize(void) {
                return d_condition_size;
            }

            /**
             * Calculates the background model with the previous provided background images and conditions.
             * I.e., for each condition and for each pixel (possibly after down-sampling), 
             * calculate the mean value (of all or a single value) and its standard deviation.
             * @return  true    If the background model has been calculated successfully, false otherwise.
             */
            bool calcBackgroundModel(void);

            /**
             * Calculates the minimal distance (in terms of standard deviations) compared to the background model.
             * @param   image       The image to compare.
             * @param   dev_image   The resulting deviations per pixel (use downsampling beforehand if needed).
             */
            void calcDeviation(Mat & image, Mat & dev_image);

            /**
             * Calculates the minimal difference (absolute terms) compared to the background model.
             * @param   image       The image to compare.
             * @param   diff_image   The resulting absolute differences per pixel (use downsampling beforehand if needed).
             */
            void calcDifference(Mat & image, Mat & diff_image);

            int getWidth(void) {
                return d_width;
            }

            int getHeight(void) {
                return d_height;
            }

            /**
             * @param   size    The kernel size of the normalized box filter for blurring.
             *                  Set to 0 to disable blurring.
             */
            void setBlur(int size) {
                if (size < 0) {
                    throw DeviationDetectionException("Blur size cannot be negative!");
                }
                d_blur_size = size;
            }

            /**
             * Utility function for dilating an image.
             * @param   size    The size of the kernel.
             */
            static void dilateImage(Mat & source, Mat & target, int size) {
                int type = MORPH_ELLIPSE;
                Mat element = getStructuringElement(type, Size(2 * size + 1, 2 * size + 1), Point(size, size));
                dilate(source, target, element);
            }

            /**
             * Utility function for eroding an image.
             * @param   size    The size of the kernel.
             */
            static void erodeImage(Mat & source, Mat & target, int size) {
                int type = MORPH_ELLIPSE;
                Mat element = getStructuringElement(type, Size(2 * size + 1, 2 * size + 1), Point(size, size));
                erode(source, target, element);
            }

            /**
             * Utility function to threshold the matrix with doubles to the thresholded mask dev_threshold.
             */
            static void threshold(Mat & source, Mat & target, double dev_threshold) {
                //TODO: Use opencv threshold function?
                for (int i = 0; i < source.rows; i++) {
                    for (int j = 0; j < source.cols; j++) {
                        if (source.at<double>(i, j) > dev_threshold) {
                            target.at<uchar>(i, j) = 1;
                        } else {
                            target.at<uchar>(i, j) = 0;
                        }
                    }
                }
            }

            /**
             * Clears all calibration data and background model, sets condition size 0.
             */
            void reset(void) {
                d_bg_images.clear();
                d_model_means.clear();
                d_model_stds.clear();
                setConditionSize(0);
            }

            /**
             * @return  true    If the background model is timed.
             */
            bool isTimed(void) {
                return d_is_timed;
            }

            /**
             * Enables or disables timed mode.
             */
            void setTimed(bool state) {
                d_is_timed = state;
            }

            /**
             * Sets maximum age for the background model.
             * Only relevant if setIsTimed is set to true.
             */
            void setMaxAge(double seconds) {
                d_max_age = seconds;
            }

            double getMaxAge(void) {
                return d_max_age;
            }

            /**
             * Removes all background images older than what is returned getMaxAge().
             * @param   time    The current time since epoch to use.
             */
            void filter(double time); 

            /**
             * @return          True if it has a valid background model.
             */
            bool hasValidBackgroundModel(void) {
                //TODO: Check if amount of images are the same per condition?
                return d_model_means.size() > 0 && (! isTimed() || (isTimed() && d_bg_images.size() == d_bg_times.size()));
            }

            ~DeviationDetection();

        private:
            //Background images for each condition:
            std::vector<std::vector<Mat> > d_bg_images;
            //Timestamp for each background image:
            std::vector<std::vector<double> > d_bg_times;

            double d_max_age;

            bool d_is_timed;

            //Background model means for each condition:
            std::vector<Mat> d_model_means;

            //Holds amount of valid pixels per condition (needed to account for noisy depth image):
            std::vector<Mat> d_model_counts;

            //Background model standard deviation for each condition:
            std::vector<Mat> d_model_stds;

            //The amount of conditions to use:
            int d_condition_size;

            //Information about previous received image:
            int d_height;
            int d_width;
            int d_depth;
            int d_channel_count;

            int d_blur_size;
    };
}

#endif // _HEIMDALL_DD_

