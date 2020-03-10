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
 *  Description: Script to control a robot on its docking plateform
 *  Author: Matthieu Magnon
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <Eigen/Eigen>

#include <iostream>

#define DEBUG
#define DEBUG_CONTROL
//#define DEBUG_BEZIER

#define TRAJ_POINT_NB   3
#define TRAJ_DURATION   30      //s
#define TRAJ_DT         0.5    //s
#define PUB_FREQUENCY   2     //Hz
#define TIME_POINT_MAX  10000
#define MAX_LIN_SPEED   0.25     //m/s
#define MAX_ANG_SPEED   0.35  //rad/s [20deg/s]

struct traj
{
    int nb = 0;
    float x[TIME_POINT_MAX] = {0};
    float y[TIME_POINT_MAX] = {0};
    float t[TIME_POINT_MAX] = {0};
};

/* State of the robot: x, y, theta, v */
Eigen::VectorXd X(4);
Eigen::VectorXd U(2);

int n_choose_k(int n, int k);
int factorial(int n);
struct traj bezier(float *x_coord, float *y_coord, int pt_nb, float dt, float duration);
void odomMessageCallback(const nav_msgs::Odometry& odom_msg);
Eigen::Vector2d control(Eigen::Vector4d X, Eigen::Vector2d YRef, Eigen::Vector2d dYRef);

float x_coord[TRAJ_POINT_NB] = {-0.5,  1.0, 3.0};
float y_coord[TRAJ_POINT_NB] = {0.5, 6.0, 3.0};

bool at_least_one_odom_received = false;

static float saturate(float val, float lower, float upper)
{
    return std::max(lower, std::min(val, upper));
}

template <typename type>
type sign(type value) {
    return type((value>0)-(value<0));
}

int main(int argc, char** argv)
{

#ifdef DEBUG
    std::cout << "Eigen version: " << EIGEN_WORLD_VERSION << ".";
    std::cout << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;
#endif

    ros::init(argc, argv, "docking");
    ros::NodeHandle nh;

    ROS_INFO("Docking node launched");

    ros::Subscriber odom_msg_sub = nh.subscribe("/odom", 100, odomMessageCallback);

    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    geometry_msgs::Twist msg;

    ros::Rate loop_rate(PUB_FREQUENCY);

    /* Compute trajectory */
    struct traj coord = bezier(x_coord, y_coord, TRAJ_POINT_NB, TRAJ_DT, TRAJ_DURATION);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1000);
    nav_msgs::Path path_msg;
    path_msg.header.seq = 0;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "odom";
    for (int i = 0; i < coord.nb; i++) {
        geometry_msgs::PoseStamped pose;
        pose.header.seq = i;
        pose.header.stamp = ros::Time::now() + ros::Duration((float)(i) * TRAJ_DT);
        pose.header.frame_id = "odom";
        pose.pose.position.x = (float)coord.x[i];
        pose.pose.position.y = (float)coord.y[i];
        pose.pose.position.z = 0.f;
        pose.pose.orientation.w = 1.f;
        pose.pose.orientation.x = 0.f;
        pose.pose.orientation.y = 0.f;
        pose.pose.orientation.z = 0.f;
        path_msg.poses.push_back(pose);
#ifdef DEBUG
        ROS_INFO("pose(%f) = [%f %f]", (float)(i) * TRAJ_DT, pose.pose.position.x, pose.pose.position.y);
#endif
    }
    path_pub.publish(path_msg);
    ros::spinOnce();
    ros::Duration(1.0).sleep();

#ifdef DEBUG
    std::cout << "XX = [";
    for (int k = 0; k < coord.nb; k++)
        std::cout << coord.x[k] << ", ";
    std::cout << "]" << std::endl;

    std::cout << "YY = [";;
    for (int k = 0; k < coord.nb; k++)
        std::cout << coord.y[k] << ", ";
    std::cout << "]" << std::endl;

    std::cout << "T = [";;
    for (int k = 0; k < coord.nb; k++)
        std::cout << coord.t[k] << ", ";
    std::cout << "]" << std::endl;
#endif

    /* TODO: controller should work with different frequency for guidance and control loop */
    int p = 1;
    while (ros::ok() && (p < coord.nb - 1))
    {
        geometry_msgs::Twist twist_msg;
        if (at_least_one_odom_received) {
            // estimate
            /* TODO: use visual pose estimator fusionned with odometry */

            // control
            Eigen::Vector2d Cmd;
            Eigen::Vector2d YRef;
            Eigen::Vector2d dYRef;
            YRef(0) = (double)coord.x[p];
            YRef(1) = (double)coord.y[p];
            dYRef(0) = (double)((coord.x[p] - coord.x[p - 1]) / TRAJ_DT);
            dYRef(1) = (double)((coord.y[p] - coord.y[p - 1]) / TRAJ_DT);

/*
            if (abs(X(3)) < 0.001) {
                ROS_WARN("Robot velocity is too slow to be controllable, injecting speed");
                X(3) = sign(X(3)) * 0.001;
            }
*/
            Cmd = control(X, YRef, dYRef);

            // publish twist command
            twist_msg.linear.x = saturate((float)Cmd(0), -MAX_LIN_SPEED, MAX_LIN_SPEED);
            twist_msg.angular.z = saturate((float)Cmd(1), -MAX_ANG_SPEED, MAX_ANG_SPEED);

            twist_pub.publish(twist_msg);

            ROS_ERROR("X =   lin_feedback_ctrl([%f, %f, %f, %f]', [%f, %f]', [%f, %f]')      U = [%f m/s, %f rad/s]   twist_msg lin/ang = [%f %f]  ",
                       X(0), X(1), X(2), X(3), YRef(0), YRef(1), dYRef(0), dYRef(1), Cmd(0), Cmd(1), 
                       twist_msg.linear.x, twist_msg.angular.z);

        } else {
            ROS_INFO("No odom message received");
        }

        path_pub.publish(path_msg);
        ros::spinOnce();
        loop_rate.sleep();
        p++;
    }

}

/*
 * Matlab implementation:
 * A = [-x(4) * sin(x(3)), cos(x(3)); ...
 *     x(4) * cos(x(3)), sin(x(3))];
 * y = [x(1); x(2)]; % sortie : y = (x, y, theta)
 * dy = [x(4)*cos(x(3)); x(4)*sin(x(3))];
 * v = (yr - y) + 2 * (dyr - dy);
 * u = inv(A) * v;
 */
Eigen::Vector2d control(Eigen::Vector4d X, Eigen::Vector2d YRef, Eigen::Vector2d dYRef)
{
    Eigen::MatrixXd A(2, 2);
    A(0, 0) = -X(3) * sin(X(2));
    A(0, 1) = cos(X(2));
    A(1, 0) = X(3) * cos(X(2));
    A(1, 1) = sin(X(2));

    Eigen::Vector2d Y;
    Y(0) = X(0);
    Y(1) = X(1);

    Eigen::Vector2d dY;
    dY(0) = X(3) * cos(X(2));
    dY(1) = X(3) * sin(X(2));

    Eigen::Vector2d V;
    V(0) = 0.0; V(1) = 0.0;
    V = (YRef - Y) + 2.0 * (dYRef - dY);

    Eigen::Vector2d U;
    U(0) = 0.0; U(1) = 0.0;
    U = A.inverse() * V;

#ifdef DEBUG_CONTROL
    std::cout << " **** CONTROL ****" << std::endl;
    std::cout << "A = " << A << std::endl;
    std::cout << "dY = " << dY << std::endl;
    std::cout << "V = " << V << std::endl;
    std::cout << "U = " << U << std::endl;
#endif

    return U;
}


/*
 * Matlab implementation:
 * t = [0:dt:1];
 * x = 0; y = 0;
 * n = length(X) - 1;
 * for i = 0:n
 *      x = x + nchoosek(n, i) * X(i + 1) * (1 - t).^(n - i) .* t.^(i);
 *      y = y + nchoosek(n, i) * Y(i + 1) * (1 - t).^(n - i) .* t.^(i);
 * end
 */
struct traj bezier(float *x_coord, float *y_coord, int pt_nb, float dt, float duration)
{
    struct traj xyt;

    if (!x_coord || !y_coord) {
        ROS_WARN("bezier: Invalid coordinates");
        return xyt;
    }

    xyt.nb = (int)(duration / dt) + 1;

    float time[xyt.nb];
    float one_minus_time[xyt.nb];

    ROS_INFO("bezier: compute %i discretization time intervals, with simulation time interval = %fs,"
             " corresponding to real time from 0s to %fs, ", xyt.nb, dt / duration, xyt.nb * dt);

    /* Precompute vectors */
    for (int k = 0; k < xyt.nb; k++) {
        time[k] = k * dt / duration;
        one_minus_time[k] = 1.f - time[k];
        xyt.t[k] = duration * time[k];
    }

    int n = pt_nb - 1;
    for (int i = 0; i <= n; i++) {
        float n_c_k = (float)n_choose_k(n, i);
        for (int k = 0; k < xyt.nb; k++) {
            xyt.x[k] = xyt.x[k] + n_c_k * x_coord[i] * pow(one_minus_time[k], n - i) * pow(time[k], i);
            xyt.y[k] = xyt.y[k] + n_c_k * y_coord[i] * pow(one_minus_time[k], n - i) * pow(time[k], i);
        }
    }


#ifdef DEBUG_BEZIER
    for (int k = 0; k < xyt.nb; k++)
        ROS_INFO("bezier: k = %i | t =  %f | x = %f | y = %f", k, xyt.t[k], xyt.x[k], xyt.y[k]);
#endif

    return xyt;
}

int n_choose_k(int n, int k)
{
    /* WARNING: unstable for high n and k */
    return (factorial(n) / (factorial(n - k) * factorial(k)));
}

int factorial(int n)
{
    int result;

    result = 1.0;

    for (int i = 1; i <= n; i++)
        result *= i;

    return result;
}

void odomMessageCallback(const nav_msgs::Odometry& odom_msg)
{
    at_least_one_odom_received = true;
    X(0) = (double)odom_msg.pose.pose.position.x;
    X(1) = (double)odom_msg.pose.pose.position.y;
    X(2) = 2.0 * asin((double)odom_msg.pose.pose.orientation.z);
    X(3) = (double)odom_msg.twist.twist.linear.x;

#ifdef DEBUG
    ROS_INFO("X = [x, y, theta, v] = [%f, %f, %f, %f]", X(0), X(1), X(2) * 57.f, X(3));
#endif

    return;
}
