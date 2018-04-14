#include <ros/ros.h>
#include <gtest/gtest.h>
#include <iostream>

#include "pathfinder/distance_estimator.h"

TEST(DistanceEstimator, works){
    DistanceEstimator de(1.0, 2.0, 3.0, 0.0);
    sensor_msgs::LaserScan scan;
    scan.intensities.resize(100);
    scan.ranges.resize(100);
    for (float &intensity : scan.intensities) {
        intensity = 250.0;
    }

    de.CalculateDistances(scan);
    // check equal to python implementation output
    EXPECT_FLOAT_EQ(12.0187767, scan.ranges[0]);
    EXPECT_FLOAT_EQ(12.0187767, scan.ranges[99]);
}

TEST(DistanceEstimator, works_with_theta){
    DistanceEstimator de(1.0, 2.0, 3.0, 0.08);
    sensor_msgs::LaserScan scan;
    scan.intensities.resize(100);
    scan.ranges.resize(100);
    for (float &intensity : scan.intensities) {
        intensity = 250.0;
    }

    de.CalculateDistances(scan);
    // check equal to python implementation output
    EXPECT_FLOAT_EQ(12.05097405, scan.ranges[0]);
    EXPECT_FLOAT_EQ(11.98671223, scan.ranges[99]);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    //ros::init(argc, argv, "tester");
    //ros::NodeHandle nh;

    return RUN_ALL_TESTS();
}
