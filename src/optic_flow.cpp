
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <highgui.h>
#include <cv.h>

#include <math.h>

using namespace cv;


 void Temp_Space( IplImage **blank_image, CvSize size, int depth, int channels )
{           
	
	*blank_image = cvCreateImage( size, depth, channels );
	
}

int main()
{    
    CvCapture* odroid_camera = cvCaptureFromCAM(CV_CAP_ANY); 
     //CvCapture *odroid_camera = cvCaptureFromFile("/home/rjt/output_afm1.avi");

  cvQueryFrame( odroid_camera );
     CvSize dimensions;

	dimensions.height =
		(int) cvGetCaptureProperty( odroid_camera , CV_CAP_PROP_FRAME_HEIGHT );
	dimensions.width =
		(int) cvGetCaptureProperty( odroid_camera , CV_CAP_PROP_FRAME_WIDTH );	



           std::cout<<"height = "<<dimensions.height<<"\t"<< " width =  "<<dimensions.width;

		int number_of_frames;

 		cvSetCaptureProperty(odroid_camera, CV_CAP_PROP_POS_AVI_RATIO, 1 );

		 number_of_frames = (int) cvGetCaptureProperty( odroid_camera, CV_CAP_PROP_POS_FRAMES );

 		
     

       cvSetCaptureProperty( odroid_camera, CV_CAP_PROP_POS_FRAMES, 0);

       
       cvNamedWindow("Camera_Output", 0);   

	int frame = 0;

	while(1)
	{
       
       

       IplImage *Image_Frame = NULL;

	 IplImage  *Image_Frame_Final = NULL;
        IplImage  *frame1 = NULL;
       IplImage  *frame2 = NULL;
      IplImage  *eig_image = NULL;
          IplImage  *temp_image = NULL;
       IplImage  *temp_space_1 = NULL;
      IplImage  *temp_space_2 = NULL;

	 cvSetCaptureProperty( odroid_camera, CV_CAP_PROP_POS_FRAMES, frame);



	//IplImage *Image_Frame_Temp = cvQueryFrame( odroid_camera );
        
 	 //Image_Frame = cvCreateImage(cvSize(32, 32), Image_Frame_Temp->depth, Image_Frame_Temp->nChannels);

       // cvResize(Image_Frame_Temp, Image_Frame, CV_INTER_LINEAR);

        

       Image_Frame = cvQueryFrame( odroid_camera );

     
         //cvSaveImage("/home/rjt/test_1.jpg", Image_Frame);
           
        //Image_Frame_Temp = cvQueryFrame( odroid_camera );
      
                   
         Temp_Space( &Image_Frame_Final, dimensions, IPL_DEPTH_8U, 3 );


  	 cvConvertImage(Image_Frame, Image_Frame_Final, 0);
		


	Temp_Space( &frame1,dimensions, IPL_DEPTH_8U, 1 );
	

	cvConvertImage(Image_Frame, frame1, 0);

		

		
		 Image_Frame = cvQueryFrame( odroid_camera );                
             //  Image_Frame_Temp = cvQueryFrame( odroid_camera);

	      // Image_Frame = cvCreateImage(cvSize(32, 32), Image_Frame_Temp->depth, Image_Frame_Temp->nChannels);

	      // cvResize(Image_Frame_Temp, Image_Frame, CV_INTER_LINEAR);

              
               
		Temp_Space( &frame2, dimensions, IPL_DEPTH_8U, 1 );

		cvConvertImage(Image_Frame, frame2, 0);

	
		Temp_Space( &eig_image, dimensions, IPL_DEPTH_32F, 1 );
		Temp_Space( &temp_image, dimensions, IPL_DEPTH_32F, 1 );

		
		CvPoint2D32f features_1[100];

		

		int features_count;
		
		

		features_count = 100;
	

              // int harris_factor=3;
             //  double k=0.04;

		cvGoodFeaturesToTrack(frame1, eig_image, temp_image, features_1, &features_count, .01, .01, NULL);//, 5, 0.4);

		
		CvPoint2D32f features_2[100];


		char optical_flow_found_feature[100];

		
		float error[100];

		
		
		CvSize optical_flow_window = cvSize(dimensions.width,dimensions.height);
		
		//CvSize optical_flow_window = cvSize(480,480);

		CvTermCriteria optical_flow_termination_criteria
			= cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 );

		

		Temp_Space( &temp_space_1, dimensions, IPL_DEPTH_8U, 1 );
		Temp_Space( &temp_space_2, dimensions, IPL_DEPTH_8U, 1 );

		
		cvCalcOpticalFlowPyrLK(frame1, frame2, temp_space_1, temp_space_2, features_1, features_2, features_count, optical_flow_window, 5, optical_flow_found_feature, error, optical_flow_termination_criteria, 0 );
		
		
                double x_avg=0,y_avg=0,angle_avg=0;

		for(int i = 0; i < features_count; i++)
		{
			
			
			CvPoint p,q;

			p.x = (double) features_1[i].x;

			p.y = (double) features_1[i].y;

			q.x = (double) features_2[i].x;

			q.y = (double) features_2[i].y;

			x_avg=x_avg+(p.x-q.x);
                        y_avg=y_avg+(p.y-q.y);
			

		

			double angle;		
			angle = atan2( abs((double) p.y - q.y), abs((double) p.x - q.x ))*180/3.14;
			
			angle_avg=angle_avg+angle;

			 std::cout<<"   "<<p.x-q.x<<"\t"<< "   "<<p.y-q.y<<"\n";// <<" "<<angle<<"\n";
   			//double hypotenuse;	hypotenuse = sqrt( square(p.y - q.y) + square(p.x - q.x) );
			
			
			//q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
			//q.y = (int) (p.y - 3 * hypotenuse * sin(angle));

		
			cvLine( Image_Frame_Final, p, q, CV_RGB(0,255,0), 1, CV_AA, 0 );

						
			//p.x = (int) (q.x + 9 * cos(angle + 3.14 / 4));
			//p.y = (int) (q.y + 9 * sin(angle + 3.14 / 4));
			//cvLine( Image_Frame_Final, p, q, CV_RGB(0,255,0), 1, CV_AA, 0 );
			//p.x = (int) (q.x + 9 * cos(angle - 3.14 / 4));
			//p.y = (int) (q.y + 9 * sin(angle - 3.14 / 4));
			//cvLine( Image_Frame_Final, p, q, CV_RGB(0,255,0), 1, CV_AA, 0 );
			
		}
		x_avg=x_avg/100;
                y_avg=y_avg/100;
		//angle_avg=angle_avg/10;


	        std::cout<<x_avg<<"\t" <<y_avg<<"\t"; //<<angle_avg<<"\n";
		
		
		cvShowImage("Camera_Output", Image_Frame_Final);

              
	
              
		int key;
		key = cvWaitKey(0);

if (key == 'a')frame--;
 else frame++;

if (frame < 0) frame = 0;
if (frame >= number_of_frames - 1) frame = number_of_frames - 2;

		
	}

 //  cvReleaseCapture(&odroid_camera); 
 //  cvDestroyWindow("Camera_Output"); 
  // return 0;
}

