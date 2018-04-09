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

class Single_Line {
	//float avg_rho, avg_theta;
	float rho_values, theta_values;
	int count;

public:
	void add_value(float, float);
	void initialize();
	double average_theta();
	double average_rho();
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

double Single_Line::average_theta(){
	double mean = theta_values / count;

	std::cout << "theta mean is: " << mean << std::endl;
	return mean;

}

double Single_Line::average_rho(){
	double mean = rho_values / count;

	std::cout << "rho mean is: " << mean << std::endl;
	return mean;

}

int main(){

	//Creates matrices available for filling later
	Mat src, src2, gray, gaussian_result, img, dest;
	Mat sobel, imgHSV, imgThreshed, imgThreshedHSV;

	src = imread("test2.jpg", 1);
	printf("Image loaded\n");

	//Changing color image to HSV to filter color
	cvtColor(src, imgHSV, CV_BGR2HSV);
	cvtColor(src, gray, CV_BGR2GRAY);

	Sobel(gray, sobel, -1, 1, 1, 5);

	inRange(sobel, Scalar(200), Scalar(255), imgThreshed);
	imshow("Threshed", imgThreshed);
	
	//inRange(imgHSV, Scalar(60, 70, 70), Scalar(120, 255, 255), imgThreshedHSV);  //blue
	inRange(imgHSV, Scalar(30, 40, 150), Scalar(100, 100, 255), imgThreshedHSV); //green

	imshow("HSV Threshed", imgThreshedHSV);

	bitwise_and(imgThreshed, imgThreshedHSV, dest);

	//FOLLOWING USES HOUGH LINES
	//Reduces noise
	GaussianBlur(dest, gaussian_result, Size(3,3), 2, 2);

	imshow("Final", gaussian_result);

	std::vector<Vec2f> lines2;
	HoughLines(imgThreshedHSV, lines2, 1, CV_PI/180, 50);

	std::cout << "number of lines: " << lines2.size() << std::endl;

	Single_Line new_value;
	std::vector<Single_Line> single_line_objects(lines2.size());
	int k = 0;

	for(size_t i = 0; i < lines2.size(); i++)
	{
		float rho = lines2[i][0];
		float theta = lines2[i][1];

		
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		Point pt1(cvRound(x0 + 1000*(-b)),
				cvRound(y0 + 1000*(a)));
		Point pt2(cvRound(x0 - 1000*(-b)),
				cvRound(y0 - 1000*(a)));

		if (theta < 0.8 ){
			//printf("passing\n");
			goto stop1;
		}
		else{
			std::cout << "angle values: " << theta << std::endl;
		//line(src, pt1, pt2, Scalar(0,0,255), 1, 8);
		}

		for(size_t j = 0; j < k + 1; j++){
			std::cout << "+-+-" << std::endl;
			//std::cout << "iteration  " << j << "  for line  " << i << std::endl;
			if (fabs(theta - single_line_objects[j].average_theta()) < 0.1){
				//std::cout << theta << " : " << single_line_objects[k].average(2) << " : " << fabs(theta - single_line_objects[k].average(2)) << std::endl;
				//printf("Within existing line\n");
				single_line_objects[j].add_value(rho, theta);
				goto stop3;
			}
			else{
				continue;
			}

			}
		//printf("Adding new line\n");
		k += 1;
		new_value.initialize();
		new_value.add_value(rho, theta);
		single_line_objects[k] = new_value;
		

		stop3:
		//std::cout << "ending line loop" << std::endl;

		stop1:
		{};
	}

	std::cout << "k is: " << k << std::endl;
	for (int p = 0 ; p < k + 1; p++){

		float rho = single_line_objects[p].average_rho();
		float theta = single_line_objects[p].average_theta();
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		Point pt1(cvRound(x0 + 1000*(-b)),
				cvRound(y0 + 1000*(a)));
		Point pt2(cvRound(x0 - 1000*(-b)),
				cvRound(y0 - 1000*(a)));

		line(src, pt1, pt2, Scalar(255,0,255), 1, 8);
	}


	imshow("Normal", src);

	waitKey(0);
	return 0;
}