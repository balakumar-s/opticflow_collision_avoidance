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
#include <eigen3/Eigen/SVD>

#define PI 3.14
using namespace cv;
using namespace Eigen;

void imageCallback(const sensor_msgs::ImageConstPtr&);
//MatrixXd pinv(MatrixXd&, double);
void control_function();
ros::Publisher left_pub,right_pub,left_image_pub,right_image_pub;
float vec_threshold=50;
int n=11;
int scale=2;
int optic_lucas(Mat,Mat,ros::Publisher);
float square(float);
Mat input_image;
int count=1;
Mat left_image_prev;
Mat right_image_prev;
sensor_msgs::Image image_temp;

MatrixXd pinv(MatrixXd& m, double epsilon = 1E-9) 
{
  typedef JacobiSVD<MatrixXd> SVD;
  SVD svd(m, ComputeFullU | ComputeFullV);
  typedef SVD::SingularValuesType SingularValuesType;
  const SingularValuesType singVals = svd.singularValues();
  SingularValuesType invSingVals = singVals;
  for(int i=0; i<singVals.rows(); i++) {
    if(singVals(i) <= epsilon) {
      invSingVals(i) = 0.0; // FIXED can not be safely inverted
    }
    else {
      invSingVals(i) = 1.0 / invSingVals(i);
    }
  }
  return MatrixXd(svd.matrixV() *
      invSingVals.asDiagonal() *
      svd.matrixU().transpose());
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"opticflow_LK");
	ros::NodeHandle n;
        ros::Rate loop_rate(15); 
//  cv::namedWindow("left_optic_flow");
//  cv::namedWindow("right_optic_flow");
  ros::Subscriber image_sub=n.subscribe("/usb_cam/image_raw",1,imageCallback);
  left_image_pub=n.advertise<sensor_msgs::Image>("flow/left",1);
  right_image_pub=n.advertise<sensor_msgs::Image>("flow/right",1);
  left_pub=n.advertise<std_msgs::Float32>("optic/left",1);
  right_pub=n.advertise<std_msgs::Float32>("optic/right",1);
  while (ros::ok())
  {
  if(input_image.empty()==0)
  {
  control_function();
  }
  ros::spinOnce();
  loop_rate.sleep();
  }
	return 0;
}
void control_function()
{
    Mat gray_image_1;
    cvtColor(input_image,gray_image_1,CV_BGR2GRAY);
    Mat gray_image=Mat(gray_image_1.rows/scale,gray_image_1.cols/scale,CV_8U);
    resize(gray_image_1,gray_image,gray_image.size());
  
    // Grayscale image is displayed
   // cv::imshow("grayscale_input",gray_image);
  //  cv::waitKey(1);
  
    // Image is split into left and right
    cv::Rect left_ROI(0, 0, gray_image.cols/2, gray_image.rows);
    cv::Rect right_ROI(gray_image.cols/2,0, gray_image.cols/2, gray_image.rows);
    Mat left_image=gray_image(left_ROI);//Mat(gray_image.rows,gray_image.cols/2,CV_8U);
    Mat right_image=gray_image(right_ROI);//Mat(gray_image.rows,gray_image.cols/2,CV_8U);
   // Left and right images are streamed
 //  imshow("left",left_image);
 //  cv::waitKey(1);
  
 //  imshow("right",right_image);
//   cv::waitKey(1);
       
  // Implement optic flow method
  int left_obs=0;
  int right_obs=0;
  if(count>1)
  {
    left_obs=optic_lucas(left_image_prev,left_image,left_image_pub);
    right_obs=optic_lucas(right_image_prev,right_image,right_image_pub);
  }
  left_image_prev=left_image;
  right_image_prev=right_image;
  count++;
  
 

  // Find maximum limit of vectors

  // Write float values
    std_msgs::Float32 left_,right_;
    left_.data=left_obs;
    right_.data=right_obs;
    left_pub.publish(left_);
    right_pub.publish(right_);

}
void imageCallback(const sensor_msgs::ImageConstPtr& im_msg)
{
         image_temp.header=im_msg->header;
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
    input_image=cv_ptr->image;

}

int optic_lucas(Mat first_image_in,Mat second_image_in,ros::Publisher name_pub)
{
  Mat first_image;
  Mat second_image;
  GaussianBlur(first_image_in,first_image,Size(n,n),0,0);
  GaussianBlur(second_image_in,second_image,Size(n,n),0,0);
  int obstacles=0;
  Mat optic_image;
  cvtColor(first_image,optic_image, CV_GRAY2RGB);
  int line_thickness=1;
  cv::Scalar line_color=CV_RGB(64, 64, 255);
  int maxCorners=50;
  std::vector<cv::Point2f> corners; 
  corners.reserve(maxCorners);
  goodFeaturesToTrack(first_image,corners,maxCorners,0.01,5);
 for(int feature_num=0;feature_num<maxCorners;feature_num++)
 {
    int i=corners[feature_num].x;
    int j=corners[feature_num].y;
    MatrixXd A(2,n*n);
    MatrixXd B(n*n,1);
    int temp_counter=0;
    for(int k=0;k<n;k++)
    {
        int row_marker=k-1;
        for(int l=0;l<n;l++)
        {
          int col_marker=l-1;
          float x_d=(first_image.at<uchar>(i-1+row_marker,j+col_marker)-first_image.at<uchar>(i+1+row_marker,j+col_marker))/2;
          float y_d=(first_image.at<uchar>(i+row_marker,j-1+col_marker)-first_image.at<uchar>(i+row_marker,j+1+col_marker))/2;

          A(0,temp_counter)=x_d;
          A(1,temp_counter)=y_d;
          B(temp_counter,0)=second_image.at<uchar>(i+row_marker,j+col_marker)-first_image.at<uchar>(i+row_marker,j+col_marker);//time_derivative.at<uchar>(i+row_marker,j+col_marker);
          temp_counter++;
        }
      }
     
      MatrixXd flow_matrix=A*A.transpose();
      flow_matrix=pinv(flow_matrix)*A*B;
      int force=abs(flow_matrix(0,0))+abs(flow_matrix(1,0));
      if(force>vec_threshold/scale)
      {
      CvPoint p,q;
      p.y=i;
      p.x=j;
      q.y=flow_matrix(0,0)+i;
      q.x=flow_matrix(1,0)+j;
      double angle;   
      angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
      double hypotenuse;  hypotenuse = sqrt( square(p.y - q.y) + square(p.x - q.x));
      q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
      q.y = (int) (p.y - 3 * hypotenuse * sin(angle));
      line( optic_image, p, q, line_color, line_thickness, CV_AA, 0 );    
      p.x = (int) (q.x + 9 * cos(angle + PI / 4));
      p.y = (int) (q.y + 9 * sin(angle + PI / 4));    
      line( optic_image, p, q, line_color, line_thickness, CV_AA, 0 );
      p.x = (int) (q.x + 9 * cos(angle - PI / 4));
      p.y = (int) (q.y + 9 * sin(angle - PI / 4));    
      line( optic_image, p, q, line_color, line_thickness, CV_AA, 0 );
      obstacles++;
      }
 
   
    
  }
  if(optic_image.empty()==0)
  {
   cv_bridge::CvImage cv_ptr;
   cv_ptr.header=image_temp.header;
   cv_ptr.encoding=sensor_msgs::image_encodings::BGR8;
   cv_ptr.image=optic_image;
  name_pub.publish(cv_ptr.toImageMsg());
 // imshow(window_name,optic_image);
  //cv::waitKey(1);
  }
  return obstacles;

}
float square(float x)
{
  float ans=pow(x,2);
}
