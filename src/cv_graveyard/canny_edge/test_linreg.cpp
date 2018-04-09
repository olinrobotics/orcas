#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <functional>
#include <numeric>

int main(int argc, char** argv){

    double init = 0.0;

    std::vector<int> x_values;
    std::vector<double> y_values;
    std::vector<double> xy_values;

    x_values.push_back(108);
    x_values.push_back(140);
    x_values.push_back(167);
    x_values.push_back(184);
    x_values.push_back(195);
    x_values.push_back(201);

    y_values.push_back(log10(20));
    y_values.push_back(log10(30));
    y_values.push_back(log10(40));
    y_values.push_back(log10(50));
    y_values.push_back(log10(60));
    y_values.push_back(log10(70));

    //std::cout << log(20) << std::endl;

    for(int i = 0; i < x_values.size(); i++){
        xy_values.push_back(x_values[i]*y_values[i]);
        printf("%f, %f\n", y_values[i], xy_values[i]);
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

    std::cout << "sumy " << sumy << " sumx " << sumx << " b " << b << " sumxy " << sumxy << " sumx2 " << sumx2 << std::endl;

    double A;
    A = pow(10, a);

    double r;
    r = pow(10, b);

    std::cout << "A = " << A << " r = " << r << std::endl;

    //printf("sumx %d\n", sumx2);
return 0;
}
