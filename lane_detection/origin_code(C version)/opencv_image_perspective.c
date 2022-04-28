#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>  

#define IMG_Width     640
#define IMG_Height    480
#define PERSPECTIVE_IMG_W 640
#define PERSPECTIVE_IMG_H 480

using namespace cv;
using namespace std;

Point2f Source[]={Point2f(0,0),Point2f(-290,330),Point2f(640,0),Point(640+290,330)};
Point2f Destination[]={Point2f(0,0),Point2f(0,PERSPECTIVE_IMG_H),Point2f(PERSPECTIVE_IMG_W,0),Point(PERSPECTIVE_IMG_W,PERSPECTIVE_IMG_H)};

Mat Perspective(Mat img)
{ 
  Mat Matrix, result_img; 	
	 
  Matrix = getPerspectiveTransform(Source,Destination);
  warpPerspective(img,result_img,Matrix,Size(PERSPECTIVE_IMG_W,PERSPECTIVE_IMG_H));
  
  return result_img;
}

Mat Canny_Edge_Detection(Mat img)
{
   Mat mat_blur_img, mat_canny_img;
   blur(img, mat_blur_img, Size(3,3));	
   Canny(mat_blur_img,mat_canny_img, 70,170,3);
	
   return mat_canny_img;	
}

int main(void)
{
	int img_width, img_height;
	img_width = 640;
	img_height = 480;
		
	//////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////  OpenCV  변수 선언 ////////////////////////////////////
	Mat mat_image_org_color;
	Mat mat_image_org_gray;
	Mat mat_image_gray_result;
	Mat mat_image_canny_edge;		
	Mat image;
	
	Scalar GREEN(0,255,0);
	Scalar RED(0,0,255);
	Scalar BLUE(255,0,0);
	Scalar YELLOW(0,255,255);
	//////////////////////////////////////////////////////////////////////////////////////
	
	mat_image_org_color = imread("/home/pi/HancomMDS/AutoCar/C++/OpenCV/5/images/line_1_0.jpg"); 
   
	img_width = mat_image_org_color.size().width ;
        img_height = mat_image_org_color.size().height;
	
        printf("Image size[%3d,%3d]\n", img_width,img_height);
    
  
        namedWindow("Display window", CV_WINDOW_NORMAL);   
	resizeWindow("Display window", img_width,img_height);   
        moveWindow("Display window", 10, 20);
	
	namedWindow("Gray Image window", CV_WINDOW_NORMAL);   
	resizeWindow("Gray Image window", img_width,img_height);   
        moveWindow("Gray Image window", 700, 20);
	
	namedWindow("Canny Edge Image window", CV_WINDOW_NORMAL);   
	resizeWindow("Canny Edge Image window", img_width,img_height);   
        moveWindow("Canny Edge Image window", 10, 520);
		
	
	while(1)
	{
	   mat_image_org_color = imread("/home/pi/HancomMDS/AutoCar/C++/OpenCV/6/images/line_2_0.jpg"); 	   
	   cvtColor(mat_image_org_color,mat_image_org_gray, CV_RGB2GRAY );       // color to gray conversion        
	   //threshold(mat_image_org_gray,mat_image_canny_edge, 200,255,THRESH_BINARY);
	   mat_image_canny_edge = Canny_Edge_Detection(mat_image_org_gray);	
	   image = Perspective(mat_image_canny_edge);
	   vector<Vec4i> linesP;
	   //HoughLinesP(mat_image_canny_edge, linesP, 1, CV_PI/180,70,30,40);
	   HoughLinesP(image, linesP, 1, CV_PI/180,60,60,20);
	   printf("Line Number : %3d\n", linesP.size());
	   for(int i=0; i<linesP.size();i++)
	   {
		Vec4i L= linesP[i];
		//int cx1 = linesP[i][0];
		//int cy1 = linesP[i][1];
		//int cx2 = linesP[i][2];
		//int cy2 = linesP[i][3];
		
		line(mat_image_org_color,Point(L[0],L[1]),Point(L[2],L[3]), Scalar(0,0,255), 1, LINE_AA);		   
		printf("L :[%3d,%3d] , [%3d,%3d] \n", L[0],L[1], L[2],L[3]); 
		//printf("H :[%3d,%3d] , [%3d,%3d] \n", cx1,cy1,cx2,cy2);
	   } 
	        printf("\n\n\n");
	   
	  if(mat_image_org_color.empty())
	  {
	      cerr << "빈 영상입니다.\n";
	      break;	  
	  }
          imshow("Display window", mat_image_org_color);
          imshow("Gray Image window",image/* mat_image_org_gray*/);
	  imshow("Canny Edge Image window", mat_image_canny_edge);      
	   if(waitKey(10) > 0)
               break;     
	}
	
   
       destroyAllWindows();
   
       return 0;
	
}
