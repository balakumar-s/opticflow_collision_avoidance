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

void imageCallback(const sensor_msgs::ImageConstPtr&);
using namespace cv;
int main(int argc,char** argv)
{
	ros::init(argc,argv,"opticflow_LK");
	ros::NodeHandle n;
  cv::namedWindow("grayscale_input");
  cv::namedWindow("left");
  cv::namedWindow("right");
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
    Mat input_image=cv_ptr->image;
    Mat gray_image;
    cvtColor(input_image,gray_image,CV_BGR2GRAY);
    cv::imshow("grayscale_input",gray_image);
    cv::waitKey(0);
    Mat left_image,right_image;
  for (int j=0;j<gray_image.rows;j++)
  {
    for(int i=0;i<gray_image.cols;i++)
    {
      if(i<gray_image.cols/2)
      {
        left_image.at<double>(i,j)=gray_image.at<double>(i,j);
      }
      else
      {
        right_image.at<double>(i+1-gray_image.cols/2,j)=gray_image.at<double>(i,j);
      }
    }
   } 
   imshow("left",left_image);
   imshow("right",right_image);

}