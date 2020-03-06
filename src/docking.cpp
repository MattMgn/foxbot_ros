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

#include <Eigen/Eigen>

#include <iostream>

#define DEBUG

#define TRAJ_POINT_NB   3
#define TRAJ_DURATION   30      //s
#define TRAJ_DT         0.02    //ms
#define PUB_FREQUENCY   10     //Hz
#define TIME_POINT_MAX  1000

struct traj
{
    int nb;
    float x[TIME_POINT_MAX];
    float y[TIME_POINT_MAX];
    float t[TIME_POINT_MAX];
};

struct state
{
    float x;
    float y;
    float theta;
    float dx;
    float dy;
    float dtheta;
};

/* State of the robot: x, y, theta, v */
Eigen::VectorXf X(4);
Eigen::VectorXf U(2);

int n_choose_k(int n, int k);
int factorial(int n);
struct traj bezier(float *x_coord, float *y_coord, int pt_nb, float dt, float duration);
void odomMessageCallback(const nav_msgs::Odometry& odom_msg);

struct state est_pose;

float x_coord[TRAJ_POINT_NB] = {-1.0,  0.0, 0.0};
float y_coord[TRAJ_POINT_NB] = {1.0, 0.75, 0.0};

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

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    geometry_msgs::Twist msg;

    ros::Rate loop_rate(PUB_FREQUENCY);

    /* Compute trajectory */
    struct traj coord = bezier(x_coord, y_coord, TRAJ_POINT_NB, TRAJ_DT, TRAJ_DURATION);


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

    while (ros::ok())
    {
        // estimate

        // control

        // dist to goal

        // pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
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
Eigen::Vector2f control(Eigen::VectorXf X, Eigen::Vector2f YRef, Eigen::Vector2f dYRef)
{
    Eigen::Matrix4f A;
    A(0, 0) = -X(3) * sinf(X(2));
    A(0, 1) = cosf(X(2));
    A(1, 0) = X(4) * cosf(X(2));
    A(1, 1) = sinf(X(2));

    Eigen::Vector2f Y;
    Y(0) = X(0);
    Y(1) = Y(1);

    Eigen::Vector2f dY;
    dY(0) = X(3) * cosf(X(2));
    dY(1) = X(3) * sinf(X(2));

    Eigen::Vector2f V;
    V = (YRef - Y) + 2.f * (dYRef - dY);

    Eigen::Vector2f U;
    //U = A.inverse()*V;
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

    xyt.nb = (int)(1.0 / dt) + 1;

    float time[xyt.nb];
    float one_minus_time[xyt.nb];

    /* Precompute vectors */
    for (int k = 0; k < xyt.nb; k++) {
        time[k] = k * dt;
        one_minus_time[k] = 1.f - time[k];
        xyt.t[k] = duration * time[k];
    }

    int pt_n = pt_nb - 1;
    for (int i = 0; i < pt_n; i++) {
        float n_c_k = (float)n_choose_k(pt_n, i);
        for (int k = 0; k < xyt.nb; k++) {
            xyt.x[k] = xyt.x[k] + n_c_k * x_coord[i] * pow(one_minus_time[k], pt_n - i) * pow(time[k], i);
            xyt.y[k] = xyt.y[k] + n_c_k * y_coord[i] * pow(one_minus_time[k], pt_n - i) * pow(time[k], i);
        }
    }

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
    est_pose.x = odom_msg.pose.pose.position.x;
    est_pose.y = odom_msg.pose.pose.position.y;
    est_pose.theta = 2.0f * asinf(odom_msg.pose.pose.orientation.z);

    X(0) = odom_msg.pose.pose.position.x;
    X(1) = odom_msg.pose.pose.position.y;
    X(2) = 2.0f * asinf(odom_msg.pose.pose.orientation.z);
    X(3) = odom_msg.twist.twist.linear.x;

#ifdef DEBUG
    //ROS_INFO("est_pose = [x, y, theta] = [%f, %f, %f]", est_pose.x, est_pose.y, est_pose.theta * 57.f);
    ROS_INFO("X = [x, y, theta, v] = [%f, %f, %f, %f]", X(0), X(1), X(2) * 57.f, X(3));
#endif

    return;
}
