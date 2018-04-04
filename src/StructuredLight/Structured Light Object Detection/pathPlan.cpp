#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cmath>
#include <algorithm>

int array_to_bool ( int occupancy[5], int length ){
  // occupancy is an array of distance values
  bool bool_array[length];
  int threshold = 30;
  for ( int c = 0 ; c < length ; c++ ) {
    if ( occupancy[c] < threshold ){
      bool_array[c] = false;
    }
    else {
      bool_array[c] = true;
    }
  }
  return 0;
}


int path_location ( int occupancy[5], int length) {
  // output the longest sequcne of Trues
  int count = 0, result = 0, location[2];
  for ( int i = 0 ; i < length ; i++ ) {
    if (occupancy[i] == false){
      count = 0;
    }
    else {
      count++;
      if ( result < count ){
        result = count;
        location[0] = i-count;
        location[1] = i;
      }
    }
  }
  return 0;
}

int main () {
  // uses array_to_bool and path_location
  // returns an output speed and theta for pwm to tugboat
  // start with minimum speed at all times for simplicity
  // look up table for theta values and traject

  return 0;
}
