#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <functional>
#include <numeric>
#include <stdio.h>
#include <stdlib.h>
#include <typeinfo> 
#include <math.h>


int main(int argc, char** argv) 
{
    cvNamedWindow("xample2", CV_WINDOW_AUTOSIZE);
    CvCapture* capture = cvCreateFileCapture( "lasertest.avi" );
    if (!capture)
    {
      std::cout << "!!! cvCreateFileCapture didn't found the file !!!\n";
      return -1; 
    }

    IplImage* frame;
    while (1) 
    {
        frame = cvQueryFrame(capture);
        if(!frame) 
            break;

        cvShowImage("xample2", frame);

        char c = cvWaitKey(33);
        if (c == 27) 
            break;
    }

    cvReleaseCapture(&capture);
    cvDestroyWindow("xample2");
}