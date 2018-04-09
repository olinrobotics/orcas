#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <typeinfo> 
#include <math.h>
#include <cmath>
#include <fstream>

using namespace cv;

int main(int argc, char** argv){

	std::ofstream outputFile("x_data.txt");
	std::ofstream outputFile2("y_data.txt");

	int c, key, total_max;
	//Creates matrices available for filling later
	Mat src, gray, sobel_vert, sobel_horz, gaussian, canny_output, canny_eroded;

	int original_height = 5; //in cm
	double laser_theta = CV_PI/6; //degrees
	int focal_length = 0.184; //in cm

	src = imread("media/100c.jpg", 1);
	printf("Image loaded\n");

	cvtColor(src, gray, CV_BGR2GRAY);

	Canny(gray, canny_output, 40, 350, 3);
	Mat element = getStructuringElement(MORPH_CROSS, Size(5, 5), Point(2, 2));
	dilate(canny_output, canny_eroded, element);

	Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );

	Sobel(canny_eroded, sobel_horz, -1, 0, 1, 3, 1); //filters out vertical lines, leaves horizontal

	std::vector<Vec4i> lines;
	HoughLinesP(canny_eroded, lines, 1, CV_PI/180, 80, 150, 2);
	int size = lines.size();

	printf("num horz lines %d\n", size);

	for(size_t i=0; i < lines.size(); i++){
		//Drawing each line that we've found
		line(drawing, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0,255,0), 2, 8);
	}

	std::vector<Vec4i> lines2;
	HoughLinesP(canny_eroded, lines2, 1, CV_PI/180, 80, 150, 15);
	Sobel(canny_eroded, sobel_vert, -1, 1, 0, 3, 1); //opposite, leaves vertical lines

	for(size_t i=0; i < lines2.size(); i++){
		//Drawing each vertical line that we've found
		line(drawing, Point(lines2[i][0], lines2[i][1]), Point(lines2[i][2], lines2[i][3]), Scalar(0,0,255), 2, 8);

			Point vert_point0(lines2[i][0], lines2[i][1]);
			Point vert_point1(lines2[i][2], lines2[i][3]);
				    outputFile << vert_point0.x << ",";
				    outputFile2 << vert_point0.y << ",";

				    outputFile << vert_point1.x << ",";
				    outputFile2 << vert_point1.y << ",";


		for(size_t j=0; j < lines.size(); j+=5){
			//finding intersections
			Point horz_point0(lines[j][0], lines[j][1]);
			Point horz_point1(lines[j][2], lines[j][3]);



			Point x = horz_point0 - vert_point0;
		    Point d1 = horz_point1 - horz_point0;
		    Point d2 = vert_point1 - vert_point0;

		    float cross = d1.x*d2.y - d1.y*d2.x;
		    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
		    Point r = horz_point0 + d1 * t1;

		    circle(src, horz_point0, 3, Scalar(255, 0, 0), 1, 8, 0);
		    	outputFile << horz_point0.x << ",";
				outputFile2 << horz_point0.y << ",";

				outputFile << horz_point1.x << ",";
				outputFile2 << horz_point1.y << ",";

		    circle(src, vert_point0, 3, Scalar(255, 0, 0), 1, 8, 0);
		    //outputFile << " " << std::endl;
		}

	}
	
	outputFile.close();
	outputFile2.close();



	imshow("src", src);
	imshow("vert sobel", sobel_vert);
	imshow("canny", drawing);
	waitKey(0);

	return 0;
}