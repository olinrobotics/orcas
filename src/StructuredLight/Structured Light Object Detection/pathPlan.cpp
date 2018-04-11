// It is working correctly now!! yay!

//#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <math.h>

#include <cmath>
#include <algorithm>

#include <iostream>
using namespace std;

// output the longest sequence of Trues
double *path_location (int range_array[], int length) {

  int count = 0;
  int longest_pos = 0;
  int threshold = 40;
  int location_range[] = {0, 0};

  // finds the longest sequence
  for (int i = 0 ; i < length; i++) {

  	// resets if false or less then threshold
    if (range_array[i] == false || range_array[i] <= threshold) {
      count = 0;
    }

    // otherwise, updates count and longest_pos
    else {
      count++;
      if (longest_pos < count ){
        longest_pos = count;

        // modifies location_range
        location_range[0] = i-longest_pos+1;
        location_range[1] = i;
      }
    }
  }

  // finds center point based on location_range
  double center_point = (location_range[0] + location_range[1])/2.0;

  // returns center_point and length
  double * returnArray = (double *) malloc(2*sizeof(double));
  returnArray[0] = center_point;
  returnArray[1] = length;

  return returnArray;
}


// performs calculations to find the rudder positions
int rudder_pos(double loc_array[]) {
	double location = loc_array[0];
	int length = loc_array[1];
	double slope = 30/(length/2.0);
	double intercept = -30.0;
	double position = ((slope*location) + intercept) + 90.0;

	return int(position);
}

int main () {
  // uses array_to_bool and path_location
  // returns an output speed and theta for pwm to tugboat
  // start with minimum speed at all times for simplicity
  // look up table for theta values and trajectory

	int test[] = {70, 60, 50, 10, 70, 60, 50, 50, 50, 70};
  int length = sizeof(test)/sizeof(test[0]);
	double *ptr = path_location(test, length);
	//cout<<ptr[0]<<" "<<ptr[1];
	//int arr[] = {ptr[1], ptr[2]};
	int x = rudder_pos(ptr);
	//cout << x << " " << endl;
  printf("x= %d\n", x);
  return 0;
}
