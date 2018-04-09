#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <functional>
#include <numeric>
#include <stdio.h>
#include <stdlib.h>
#include <typeinfo> 
#include <math.h>

using namespace cv;


//First we create a Single_Line class to hold all the individual lines we find
class Single_Line {
	float rho_values, theta_values;
	int count;

public:
	void add_value(float, float);
	void initialize();
	float average_theta();
	float average_rho();
};

void Single_Line::initialize(){
	count = 0.001;
	rho_values = theta_values = 0;
}

void Single_Line::add_value(float rho, float theta){
	std::cout << "adding rho: " << rho << " adding theta: " << theta << std::endl;
	count += 1;
	rho_values += rho;
	theta_values += theta;
}

float Single_Line::average_theta(){
	float mean = theta_values / count;

	std::cout << "theta mean is: " << mean << std::endl;
	return mean;

}

float Single_Line::average_rho(){
	float mean = rho_values / count;

	std::cout << "rho mean is: " << mean << std::endl;
	return mean;

}

int main(int argc, char** argv)
{

	int c, key;

	//Creates matrices available for filling later
	Mat src, src2, gray, gaussian_result, img, dest;
	Mat sobel, imgHSV, imgThreshed, imgThreshedHSV;

	IplImage* color_img;
	CvCapture* cv_cap = cvCreateFileCapture("lasertest.avi"); //previous video
	//CvCapture* cv_cap = cvCaptureFromCAM(0); //USB Cam
	cvNamedWindow("Final", 0);
	cvNamedWindow("Gaussian", 0);

	//Necessary if taking info from USB Camera
/*	printf("Press enter to begin detection\n");
	getchar();*/

	for(;;){

		color_img = cvQueryFrame(cv_cap);

		if (color_img != 0){
			src = color_img;

			//Changing color image to HSV to filter color
			cvtColor(src, imgHSV, CV_BGR2HSV);

			//inRange(imgHSV, Scalar(60, 70, 70), Scalar(120, 255, 255), imgThreshedHSV);  //blue
			inRange(imgHSV, Scalar(30, 40, 30), Scalar(100, 200, 255), imgThreshedHSV); //green

			//Use for Sobel filter, emphasizes edges, uncessary currently
/*			cvtColor(src, gray, CV_BGR2GRAY);
			Sobel(gray, sobel, -1, 1, 1, 5);
			inRange(sobel, Scalar(200), Scalar(255), imgThreshed);
			bitwise_and(imgThreshed, imgThreshedHSV, dest);
*/
			//Reduces noise
			GaussianBlur(dest, gaussian_result, Size(3,3), 2, 2);

			//Creates vector too hold the lines from Hough transform
			std::vector<Vec2f> lines;

			//Hough Transform
			//TO-DO: Optimize for lasers
			HoughLines(imgThreshedHSV, lines, 1, CV_PI/180, 50);

			std::cout << "number of lines: " << lines.size() << std::endl;

			//Creates a baseline and a placeholder object for the "Single Lines" we'll find
			Single_Line baseline, new_value;

			//Creates a vector of Single Line objects to fill
			std::vector<Single_Line> single_line_objects(lines.size());
			
			//Fills first space of vector with a baseline, never used
			baseline.initialize();
			baseline.add_value(180, 1.5);
			single_line_objects[0] = baseline;

			//Initializes counter to traverse single_line_objects
			int k = 0;

			for(size_t i = 0; i < lines.size(); i++){

				//Hough Lines outputs rho and theta, we break up the two
				float rho = lines[i][0];
				float theta = lines[i][1];
	
				//Finding x, y intercepts and initial points
				double a = cos(theta), b = sin(theta);
				double x0 = a*rho, y0 = b*rho;

				//Creating points to draw lines, 1000 is arbitrarily large number
				//TO-DO: Change the scaling to represent size of line segment
				Point pt1(cvRound(x0 + 1000*(-b)),
						cvRound(y0 + 1000*(a)));
				Point pt2(cvRound(x0 - 1000*(-b)),
						cvRound(y0 - 1000*(a)));

				if (theta < 0.8 ){
					//Skips lines that are vertical, since we're mostly looking for horizontal lines
					goto stop1;
				}

				else{
					//std::cout << "angle values: " << theta << std::endl;
					//line(src, pt1, pt2, Scalar(0,0,255), 1, 8);
				}

				for(size_t j = 1; j < k + 1; j++){
					std::cout << "+-+-" << std::endl;
					//std::cout << "iteration  " << j << "  for line  " << i << std::endl;
					//Checks if every line is within 0.1 theta of already created Single Lines
					if (fabs(theta - single_line_objects[j].average_theta()) < 0.1){
						//printf("Within existing line\n");
						single_line_objects[j].add_value(rho, theta);
						goto stop3;
					}
					else{
						continue;
					}
				}

				printf("Adding new line\n");
				k += 1;
				
				//Zeros previous placeholder object, initializes new line with current rho and theta
				new_value.initialize();
				new_value.add_value(rho, theta);
				single_line_objects[k] = new_value;
				
				stop3:
				std::cout << "ending line loop" << std::endl;

				stop1:
				{};
			}

			//Counts total number of Single Line Objects
			std::cout << "k is: " << k << std::endl;

			for (int p = 1 ; p < k + 1; p++){

				//Draws a line for each line in single objects, follows same rule as above
				float rho = single_line_objects[p].average_rho();
				float theta = single_line_objects[p].average_theta();

				double a = cos(theta), b = sin(theta);
				double x0 = a*rho, y0 = b*rho;
				
				Point pt1(cvRound(x0 + 1000*(-b)),
						cvRound(y0 + 1000*(a)));
				Point pt2(cvRound(x0 - 1000*(-b)),
						cvRound(y0 - 1000*(a)));

				line(src, pt1, pt2, Scalar(255,255,255), 1, 8);
			}

			imshow("Final", src);
			imshow("Gaussian", imgThreshedHSV);

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