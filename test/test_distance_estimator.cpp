#include "distance_estimator.h"
#include "ros/ros.h"

#include <gtest/gtest.h>
#include <iostream>

TEST(DistanceEstimator, works){
    DistanceEstimator de(1.0, 2.0, 3.0, 0.0);
    sensor_msgs::LaserScan scan;
    scan.intensities.resize(100);
    scan.ranges.resize(100);
    for (float &intensity : scan.intensities) {
        intensity = 250.0;
    }

    de.CalculateDistances(scan);
    EXPECT_FLOAT_EQ(12.0187767, scan.ranges[0]);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    //ros::init(argc, argv, "tester");
    //ros::NodeHaaåndle nh;

    return RUN_ALL_TESTS();
}
