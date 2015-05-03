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
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#define PI 3.14
void imageCallback(const sensor_msgs::ImageConstPtr&);
ros::Publisher left_pub,right_pub;
using namespace cv;
using namespace Eigen;
int n=5;
//std::vector<Point2f> optic_lucas(Mat,Mat);
//Point2f(float,float) optic_lucas(Mat,Mat);
MatrixXf optic_lucas(Mat,Mat);
float square(float);

int count=1;
Mat left_image_prev;
int main(int argc,char** argv)
{
	ros::init(argc,argv,"opticflow_LK");
	ros::NodeHandle n;
  cv::namedWindow("grayscale_input");
  cv::namedWindow("left");
  cv::namedWindow("right");
  cv::namedWindow("x_derivative");
  cv::namedWindow("y_derivative");
  cv::namedWindow("time_derivative");
  cv::namedWindow("left_optic_flow");
  
  
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
  /* imshow("left",left_image);
       cv::waitKey(1);
   imshow("right",right_image);
       cv::waitKey(1);
       */
  // TODO: Implement optic flow method
  if(count>1)
  {
    int line_thickness=1;
    cv::Scalar line_color=(94.0, 206.0, 165.0, 0.0);
  Mat optic_image=left_image;
  Point2f left_vec[left_image.rows][left_image.cols];
  MatrixXf left_flow;
  left_flow=optic_lucas(left_image_prev,left_image);
  for(int i=0;i<left_image.rows;i++)
  {
    for(int j=0;j<left_image.cols;j++)
    {
      CvPoint p,q;
      p.x=i;
      p.y=j;
      q.x=left_flow(i,j);
      q.y=left_flow(i+left_image.rows-1,j+left_image.cols-1);
      double angle;   
      angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
      double hypotenuse;  hypotenuse = sqrt( square(p.y - q.y) + square(p.x - q.x));
      q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
      q.y = (int) (p.y - 3 * hypotenuse * sin(angle));
      line( optic_image, p, q, line_color, line_thickness, CV_AA, 0 );
      /* Now draw the tips of the arrow.  I do some scaling so that the
        * tips look proportional to the main line of the arrow.
        */   
      p.x = (int) (q.x + 9 * cos(angle + PI / 4));
      p.y = (int) (q.y + 9 * sin(angle + PI / 4));    
      line( optic_image, p, q, line_color, line_thickness, CV_AA, 0 );
      p.x = (int) (q.x + 9 * cos(angle - PI / 4));
      p.y = (int) (q.y + 9 * sin(angle - PI / 4));    
      line( optic_image, p, q, line_color, line_thickness, CV_AA, 0 );
    }
  }
  imshow("left_optic_flow",optic_image);
  }
  left_image_prev=left_image;
  count++;
 // Point2f right_vec=optic_lucas(right_image_prev,right_image);
  //Plot vectors:

  // Find maximum limit of vectors

  // Write float values
    std_msgs::Float32 left_,right_;
    left_.data=1;
    right_.data=1;
    left_pub.publish(left_);
    right_pub.publish(right_);

}

//std::vector<Point2f> optic_lucas(Mat first_image,Mat second_image)
//const Point2f&(float,float) optic_lucas(Mat first_image,Mat second_image)
MatrixXf optic_lucas(Mat first_image,Mat second_image)
{
  Mat x_derivative=Mat(first_image.rows,first_image.cols,CV_8U);
  Mat y_derivative=Mat(first_image.rows,first_image.cols,CV_8U);
  Mat time_derivative=Mat(first_image.rows,first_image.cols,CV_8U);
  for(int i=0;i<first_image.rows;i++)
  {
    for(int j=0;j<first_image.cols;j++)
    {
      if(i>0&&i<first_image.rows-1)
      {
        x_derivative.at<uchar>(i,j)=(first_image.at<uchar>(i-1,j)-first_image.at<uchar>(i+1,j))/2;
      }
      if(j>0&&j<first_image.cols-1)
      {
        y_derivative.at<uchar>(i,j)=(first_image.at<uchar>(i,j-1)-first_image.at<uchar>(i,j+1))/2;
      }
    }
  }
  time_derivative=second_image-first_image;
  imshow("x_derivative",x_derivative);
  cv::waitKey(1);
  imshow("y_derivative",y_derivative);
  cv::waitKey(1);
  imshow("time_derivative",time_derivative);
  cv::waitKey(1);
  Point2f flow_vectors[first_image.rows][first_image.cols];
  MatrixXf optic_flow_matrix(first_image.rows*2,first_image.cols*2);
  for(int i=n;i<first_image.rows-n;i++)
  {
    for(int j=n;j<first_image.cols-n;j++)
    {
      MatrixXf A(n,n);
      MatrixXf B(1,n);
      for(int k=0;k<n;k++)
      {
        int row_marker=k-1;
        for(int l=0;l<n;l++)
        {
          int col_marker=l-1;
          A(0,l+k)=x_derivative.at<uchar>(i+row_marker,j+col_marker);
          A(1,l+k)=y_derivative.at<uchar>(i+row_marker,j+col_marker);
          B(0,l+k)=time_derivative.at<uchar>(i+row_marker,j+col_marker);
        }
      }
      MatrixXf temp_vector= (A.transpose()*A)*A.transpose()*B.transpose();
      MatrixXf flow_matrix=-temp_vector.inverse();
      optic_flow_matrix(i,j)=flow_matrix(0,0);
      optic_flow_matrix(i+first_image.rows-1,j+first_image.cols-1)=flow_matrix(0,1);
      flow_vectors[i][j].x=flow_matrix(0,0);
      flow_vectors[i][j].y=flow_matrix(0,1);

    }
    
  }
  //std::vector<Point2f> temp;//=flow_vectors;
  //Point2f t;
 // return t;
  return optic_flow_matrix;
}
float square(float x)
{
  float ans=pow(x,2);
}