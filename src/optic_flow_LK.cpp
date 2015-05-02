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
#include <std_msgs/Float32.h>

void imageCallback(const sensor_msgs::ImageConstPtr&);
ros::Publisher left_pub,right_pub;
using namespace cv;
int main(int argc,char** argv)
{
	ros::init(argc,argv,"opticflow_LK");
	ros::NodeHandle n;
  cv::namedWindow("grayscale_input");
  cv::namedWindow("left");
  cv::namedWindow("right");
	ros::Subscriber image_sub=n.subscribe("/usb_cam/image_raw",1,imageCallback);
  left_pub=n.advertise<std_msgs::Float32>("optic/left",1);
  right_pub=n.advertise<std_msgs::Float32>("optic/right",1);
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
    //Image is converted to grayscale
    //TODO remove distortion
    Mat input_image=cv_ptr->image;
    Mat gray_image_1;
    cvtColor(input_image,gray_image_1,CV_BGR2GRAY);
    Mat gray_image=Mat(gray_image_1.rows,gray_image_1.cols,CV_8U);
    gray_image=gray_image_1;
    // Grayscale image is displayed
    cv::imshow("grayscale_input",gray_image);
    cv::waitKey(1);
    // Image is split into left and right
    Mat left_image=Mat(720,1280/2,CV_8U);
    Mat right_image=Mat(720,1280/2,CV_8U);
  for (int i=0;i<gray_image.rows;i++)
  {
    for(int j=0;j<gray_image.cols;j++)
    {
      if(j<gray_image.cols/2)
      {
        left_image.at<uchar>(i,j)=gray_image.at<uchar>(i,j);
      }
      else
      {
        right_image.at<uchar>(i,j-gray_image.cols/2)=gray_image.at<uchar>(i,j);
      }
    }
   } 
   // Left and right images are streamed
   imshow("left",left_image);
       cv::waitKey(1);
   imshow("right",right_image);
       cv::waitKey(1);
  // TODO: Implement optic flow method
  //optic_lucas(left_image);
  // Find maximum limit of vectors

  // Write float values
    std_msgs::Float32 left_,right_;
    left_.data=1;
    right_.data=1;
    left_pub.publish(left_);
    right_pub.publish(right_);

}