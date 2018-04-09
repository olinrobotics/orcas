#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <typeinfo> 
#include <math.h>

using namespace cv;

/*template <typename T, size_t N> const T* mybegin(const T (&a)[N]) { return a; }    
template <typename T, size_t N> const T* myend  (const T (&a)[N]) { return a+N; }*/

int main(int argc, char** argv){

	int c, key;
	//Creates matrices available for filling later
	Mat src, gray, sobel, gaussian, dest, dest_eroded;
	Mat imgHSV, imgThreshedHSV, hsvandgray;

	IplImage* color_img;
	CvCapture* cv_cap = cvCreateFileCapture("underwater1.webm"); //previous video
	//CvCapture* cv_cap = cvCaptureFromCAM(1); //USB Cam

	std::vector<int> distance_vector;

	for(;;){
		color_img = cvQueryFrame(cv_cap);

		if (color_img != 0){

			src = color_img;
			//Changing color image to HSV to filter color
			cvtColor(src, imgHSV, CV_BGR2HSV);
			cvtColor(src, gray, CV_BGR2GRAY);

			//inRange(imgHSV, Scalar(60, 70, 70), Scalar(120, 255, 255), imgThreshedHSV);  //blue
			//inRange(imgHSV, Scalar(30, 40, 30), Scalar(200, 200, 255), imgThreshedHSV); //green general
			inRange(imgHSV, Scalar(30, 40, 30), Scalar(200, 200, 175), imgThreshedHSV); //green
		
			Sobel(gray, sobel, -1, 0, 1, 3);

			GaussianBlur(sobel, gaussian, Size(5,5), 2, 2);

			Canny(gaussian, dest, 40, 350, 3);
			Mat element = getStructuringElement(MORPH_CROSS, Size(5, 5), Point(2, 2));

			dilate(dest, dest_eroded, element);

			bitwise_and(dest_eroded, imgThreshedHSV, hsvandgray);

			for (int i = 0; i < hsvandgray.rows; i++){
				for (int j = hsvandgray.cols; j >= 1; --j){
					int intensity = (int)hsvandgray.at<uchar>(i, j);
					if (intensity > 240){
						int distance = j; //modify this later
						distance_vector.push_back(distance);
					}
				}
			}

/*
			int max_distance = *std::max_element(mybegin(distance_vector), myend(distance_vector));
			std::cout << "max distance: " << max_distance << std::endl;*/
			int k = 0;
			int key;

			if(key == 97){
			for( std::vector<int>::iterator i = distance_vector.begin(); i != distance_vector.end(); ++i){
				k += 1;
    			if(k == 100){
    				std::cout << *i << ',';
    				k = 0;
    				}
    			}
    		}
		
			imshow("Grayscale", gray);
			imshow("Sobel", sobel);
			//imshow("HSV Threshed", imgThreshedHSV);
			imshow("Canny Edge", dest_eroded);
			//imshow("Bitwise And", hsvandgray);
			imshow("Gaussian", gaussian);

			c = cvWaitKey(10);
			if (c == 27){
				break;
			}
		}
	}
	
	cvReleaseCapture( &cv_cap);
	cvDestroyWindow("Video");
	return 0;
}