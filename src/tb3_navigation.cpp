//
// Created by malintha on 1/23/18.
//

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include "pidImpl.h"
#include "visualization_msgs/Marker.h"

using namespace Eigen;

std::string m_worldFrame;
std::string m_bodyFrame;
double worldTime = 0;
static float radius_x = 1.5;
static float radius_y = 1.5;
static Vector2d* wparray[10];
static double anglearr[10];

void showOnRviz(tf::Transform& transform);
float get(const ros::NodeHandle &n, const std::string &key);
Vector2d getGoalPosition(double t);
void drawTarget(Vector2d targetX, ros::Publisher publisher);
void drawTrack(ros::Publisher pub);

float transformtoyaw(tf::StampedTransform transform) {
    tfScalar roll, pitch, yaw;
    Vector3d rpy;
    tf::Matrix3x3(tf::Quaternion(
            transform.getRotation().x(),
            transform.getRotation().y(),
            transform.getRotation().z(),
            transform.getRotation().w()
    )).getRPY(roll, pitch, yaw);

    return yaw;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "turtlebot3_nav");
    static ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher m_pubNav = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    wparray[0] = new Vector2d(1.04, 1.08);
    wparray[1] = new Vector2d(0, 1.5);
    wparray[2] = new Vector2d(-1.06, 1.05);
    wparray[3] = new Vector2d(-1.5, 0);
    wparray[4] = new Vector2d(-1.05, -1.06);
    wparray[5] = new Vector2d(0, -1.5);
    wparray[6] = new Vector2d(1.06, -1.06);
    wparray[7] = new Vector2d(1.5, 0);

    anglearr[0] = 0.785;
    anglearr[1] = 1.57;
    anglearr[2] = 2.35;
    anglearr[3] = 3.14;
    anglearr[4] = 2.35;
    anglearr[5] = 1.57;
    anglearr[6] = 0.785;
    anglearr[7] = 0;

    int counter = 0;
    Vector2d targetX = *wparray[counter];
    double targettheta = anglearr[counter];

    tf::TransformListener m_listener;
    tf::StampedTransform transform;

    geometry_msgs::Twist msg;
    m_worldFrame = "world";
    m_bodyFrame = "vicon/leonardo/leonardo";

    m_listener.waitForTransform(m_worldFrame, m_bodyFrame, ros::Time(0), ros::Duration(10.0));

    static float kp = get(n, "kp");
    static float ki = get(n, "ki");
    static float kd = get(n, "kd");
    static float speed = get(n, "speed");
    static float dt = get(n, "dt");
    pidImpl pid(kp, kd, ki, dt, worldTime);

    Vector2d X;
    ros::Time start = ros::Time::now();
    while (ros::ok()) {
        m_listener.lookupTransform(m_worldFrame, m_bodyFrame, ros::Time(0), transform);
        X << transform.getOrigin().x(), transform.getOrigin().y();
        worldTime = ros::Duration(ros::Time::now() - start).toSec();
        if (sqrt(pow(X[0] - targetX[0], 2) + pow(X[1] - targetX[1], 2)) < 0.3) {
            counter++;
            targetX = *wparray[counter];
            targettheta = anglearr[counter];
            std::cout<<counter<<"  "<<targetX[0]<<" "<<targetX[1]<<std::endl;
            if(counter == 7)
                counter = -1;
        }

        float theta = transformtoyaw(transform);
        if(theta < 0)
            theta = -theta;
        float theta_dot = pid.update(targettheta,targetX, X, theta, worldTime);

        msg.linear.x = speed;
        msg.angular.z = theta_dot;

        m_pubNav.publish(msg);
        drawTrack(marker_pub);
        drawTarget(targetX, marker_pub);
        showOnRviz(transform);
    }
    ros::spin();
    return 0;
}

void showOnRviz(tf::Transform& transform) {
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
}
Vector2d targetx;

Vector2d getGoalPosition(double t) {
        targetx << radius_x * cos(M_PI*t/52), radius_y * sin(M_PI*t/52);
    return targetx;
}

void drawTarget(Vector2d targetX, ros::Publisher publisher) {
    visualization_msgs::Marker targetPath;
    geometry_msgs::Point p, p1;

    targetPath.ns = "targetpath";
    targetPath.header.stamp = ros::Time::now();
    targetPath.type = visualization_msgs::Marker::LINE_STRIP;
    targetPath.header.frame_id="world";
    targetPath.action = visualization_msgs::Marker::ADD;
    targetPath.id = 10;
    targetPath.color.r = 1.0;
    targetPath.color.a = 1.0;
    targetPath.scale.x = 0.1;
    targetPath.pose.orientation.w = 1.0;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    p1.x = targetX[0];
    p1.y = targetX[1];
    p1.z = 0;
    targetPath.points.push_back(p1);
    targetPath.points.push_back(p);
    publisher.publish(targetPath);
}

void drawTrack(ros::Publisher pub) {
    visualization_msgs::Marker path;
    path.header.stamp = ros::Time::now();
    path.type = visualization_msgs::Marker::LINE_STRIP;
    path.header.frame_id="world";
    path.action = visualization_msgs::Marker::ADD;
    path.id = 0;
    path.color.r = 0.752;
    path.color.g = 0.752;
    path.color.b = 0.752;
    path.color.a = 0.9;
    path.scale.x = 0.4;
    path.scale.z = 0;
    path.scale.y = 0;
    path.pose.orientation.w = 1.0;

    for(float f=-1;f<1;f+=0.01) {
        geometry_msgs::Point p;
        p.x = (radius_x + 0.1) * cos(M_PI*f);
        p.y = (radius_y + 0.1) * sin(M_PI*f);
        p.z = -0;
        path.points.push_back(p);
    }
    pub.publish(path);
}

float get(const ros::NodeHandle &n, const std::string &key) {
    double value;
    n.getParam("/tb3_navigation/"+key, value);
    return value;
}