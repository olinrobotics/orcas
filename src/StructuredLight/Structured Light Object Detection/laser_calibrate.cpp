#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <typeinfo>
#include <math.h>
#include <cmath>
#include <algorithm>

#include <thread>
#include <tuple>
#include <fstream>

using namespace cv;

//Global variables are bad, but these make the code easier
//VideoCapture cv_cap("media/scaled_t1.webm");
VideoCapture cv_cap(1);
bool ready_cap = false;
bool done = false;
std::vector<int> x_values;


int getdistance(Mat src){

	//Creates matrices available for filling later
	Mat gray, sobel, gaussian, canny_output, eroded, overlay_color, overlay;
	Mat contours_output, eroded_raw;
	Mat element = getStructuringElement(MORPH_CROSS, Size(5, 5), Point(2, 2));

	//Changing color image to gray
	cvtColor(src, gray, CV_BGR2GRAY);

	//Sobel filter for horizontal lines, then canny to detect edges
	Sobel(gray, sobel, -1, 0, 1, 3, 1);
	GaussianBlur(sobel, gaussian, Size(5,5), 2, 2);
	Canny(gaussian, canny_output, 40, 350, 3);

	//Creating Mat to fill in lines later
	Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );

	//Allows us to scale code to image width
	Size image_size = canny_output.size();

	//drawing stuff in occupancy grid
	Mat occupancy_grid = Mat::zeros( canny_output.size(), CV_8UC3 );
	std::vector<Point> occupancy_points;

	for(int g = 0; g < 485; g += 35){
		line(occupancy_grid, Point(g, 0), Point(g, 450), Scalar(0, 0, 255), 1, 8, 0);
		line(occupancy_grid, Point(0, g), Point(450, g), Scalar(0, 0, 255), 1, 8, 0);
	}

	//Vectors defined to hold found contours
	std::vector<std::vector<Point> > contours;
	std::vector<Vec4i> hierarchy;

	//Defined here to find countour endpoints
	int proj_x, proj_y = 0;

	//Values used to hold calculated end points of contours
	std::vector<int> values;
	std::vector<int> distance_values;

	// Find contours
	findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

		//If we find too many contours, skips all calculations and waits
	if (contours.size() > 3){
		//imshow("eroded", previous);
		printf("Please make sure there is only 1 laser line\n");
		return 0;
	}

	else if(contours.size() == 0){
		printf("Make sure laser line is sufficiently bright\n");
		return 0;
	}

	//std::cout << "contours size " << contours.size() << std::endl;

	// Draw contours
	for( int l = 0; l < contours.size(); l++ ){
		if(contours[l].size() > 5){
			drawContours( drawing, contours, l, Scalar(255, 255, 255), 2, 8, hierarchy, 0, Point() );
		}
	}

	//Thickens found countour lines
	dilate(drawing, eroded_raw, element);

	cvtColor(eroded_raw, eroded, CV_BGR2GRAY);

	//Finds intensity per column
	for (int i = 0; i < eroded.cols; i++){
		for (int j = 0; j < eroded.rows; j++){
			int intensity = (int)eroded.at<uchar>(j, i);
			if (intensity > 240){
				eroded.at<uchar>(j, i) = 150;
				if (abs(j - proj_y) > 5){  //&& abs(i - proj_x) > 10

					values.push_back(proj_y); //These let us draw the end points
					values.push_back(proj_x);

					values.push_back(j);
					values.push_back(i);
				}
				proj_y = j;
				proj_x = i;
				break;
			}
			else if (i == eroded.cols - 1 && j == eroded.rows - 1){
				values.push_back(proj_y);
				values.push_back(proj_x);
			}

			else{
				eroded.at<uchar>(j, i) = 50;
			}
		}
	}

	//Brings back color to the overlay
	cvtColor(eroded, overlay_color, CV_GRAY2BGR);

	std::vector<int> pixel_distances;

	if ( values.size() > 2){
		//Draws countour enpoint lines
		for (int s = 2; s < values.size(); s +=4 ){
			line(overlay_color, Point(values[s + 1], values[s]), Point(values[s + 1], image_size.height), Scalar(0,244,0), 2, 8, 0);
			line(overlay_color, Point(values[s + 3], values[s + 2]), Point(values[s + 3], image_size.height), Scalar(244,244,0), 2, 8, 0);
			line(overlay_color, Point(values[s + 1], values[s]), Point(values[s + 3], values[s + 2]), Scalar(0,244,244), 2, 8, 0);


			int mid_line = image_size.height / 2;
			pixel_distances.push_back(mid_line - values[s]);
		}
	}

	int d1;

	d1 = *std::min_element(pixel_distances.begin(), pixel_distances.end());

	return d1;
}



void showcamera(){
	int c;
	Mat color_img, src;

	for(;;){

		color_img = cv_cap.grab();

		if (cv_cap.read(color_img) && !done){
			src = color_img;

			if(ready_cap){
				//Only at key press do we measure for pixel distance
				int d1;
				d1 = getdistance(src);
				printf("Pixel distance is %d\n", d1);
				x_values.push_back(d1);
				ready_cap = false;
			}
			imshow("original", src);

			c = cvWaitKey(50);
			if (c == 27){
				break;
			}
		}

		else {
			return;
		}
	}

}

std::tuple<double, double> calculate_expreg(){
	//Basically calculates exponential regression by finding the linear regression of logY (distance)

    double init = 0.0;

    std::vector<double> y_values;
    std::vector<double> xy_values;

    y_values.push_back(log10(20));
    y_values.push_back(log10(30));
    y_values.push_back(log10(40));
    y_values.push_back(log10(50));
    y_values.push_back(log10(60));
    y_values.push_back(log10(70));

    for(int i = 0; i < x_values.size(); i++){
        xy_values.push_back(x_values[i]*y_values[i]);
    }

    int n = x_values.size();

    int sumx;
    sumx = std::accumulate(x_values.begin(), x_values.end(), init);

    double sumy;
    sumy = std::accumulate(y_values.begin(), y_values.end(), init);

    int sumx2;
    sumx2 = std::inner_product(x_values.begin(), x_values.end(), x_values.begin(), init);

    double sumxy;
    sumxy = std::accumulate(xy_values.begin(), xy_values.end(), init);

    double b;
    b = (n*sumxy - sumx*sumy) / (n*sumx2 - sumx*sumx);

    double a;
    a = (sumy - b*sumx) / n;

    double A;
    A = pow(10, a);

    double r;
    r = pow(10, b);

    return std::make_tuple(A, r);

	}



int main(int argc, char** argv){

	double A, r;

	//We'll output the results of exp_reg in a text file
	std::ofstream outputFile("exp_reg.txt");

	//Opens the video that we defined as global variable
	cv_cap.open(1);

	//Scaled tests start at 20cm, then go up by increments of 10cm
	//cv_cap.open("media/scaled_t1.webm");

	cv_cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cv_cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	std::thread camera(showcamera);

	printf("Ready to calibrate. Press enter to continue each step\n");
	getchar();

	printf("Set camera/laser array 20cm away from flat uniform surface.\n");
	printf("Press enter when ready to measure\n");
	getchar();
	ready_cap = true;
	printf("Press enter to move to next measurement, we are measuring to 70cm\n");
	getchar();

	printf("Move camera 30cm away\n");
	getchar();
	ready_cap = true;
	getchar();

	printf("Move camera 40cm away\n");
	getchar();
	ready_cap = true;
	getchar();

	printf("Move camera 50cm away\n");
	getchar();
	ready_cap = true;
	getchar();

	printf("Move camera 60cm away\n");
	getchar();
	ready_cap = true;
	getchar();

	printf("Move camera 70cm away\n");
	getchar();
	ready_cap = true;
	getchar();

	printf("Calibration complete, thank you!\n");
	done = true;
	camera.join();

	printf("Calculating exponential fit line...\n");
	std::tie(A, r) = calculate_expreg();

	std::cout << "Curve equation is: distance = " << A << "*" << r << "^(pixel distance)" << std::endl;
	outputFile << A << " " << r;
	outputFile.close();

	printf("Thank you for your time, press enter to exit calibration.\n");
	getchar();

	cv_cap.release();
	return 0;
}
