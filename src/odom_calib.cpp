/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Description: Script to run a trajectory from a velocity table
 *  Author: Matthieu Magnon
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "odom_calib.h"

#define PUB_FREQUENCY   10      // [Hz]

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_calibration");

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    geometry_msgs::Twist msg;

    ros::Rate loop_rate(PUB_FREQUENCY);

    ROS_INFO("Odometry calibration sequence launched");

    double begin_time = ros::Time::now().toSec();
    double prev_time = ros::Time::now().toSec();
    
    int i = 0;
    double dt[SEQ_NB];
    while (ros::ok() && i < SEQ_NB) {

        msg.linear.x = cmd_vel_table[i][1];
        msg.angular.z = cmd_vel_table[i][2];
        
        dt[i] = ros::Time::now().toSec() - prev_time;
        prev_time = ros::Time::now().toSec();

        ROS_INFO("dt = %fs", dt[i]);

        i++;
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Sequence terminated");

    // compute static about time jittering
    double mean = 0.0;
    for (i = 1; i < SEQ_NB; i++) {
        mean += dt[i];
    }
    mean /= SEQ_NB;

    double std = 0.0;
    for (i = 1; i < SEQ_NB; i++) {
        std += (dt[i] - mean) * (dt[i] - mean);
    }
    std = sqrt(std / SEQ_NB);

    ROS_INFO("Time jittering : avg = %fs, std = %fs", (float)mean, (float)std);

    return 0;
}
