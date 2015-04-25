#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <highgui.h>
#include <cv.h>
#include <opencv2/video/tracking.hpp>
#include <math.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr&)

int main(int argc,char** argv)
{
	ros::init(argc,argv,"opticflow_LK");
	ros::Nodehandle n;
	ros::Subscriber image_sub=n.subscribe("/usb_cam_node/image_raw",1,imageCallback);
	ros::spin();
	return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& im_msg)
{
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(im_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    

}