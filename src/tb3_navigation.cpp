//
// Created by malintha on 1/23/18.
//

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>

std::string m_worldFrame;
std::string m_bodyFrame;
static double frequency = 20;
static float linearV = 0.001;
static float angularV = 0.001;
static float dt = 0.001;

void showOnRviz(float v, float theta_dot);

float xPrev, yPrev, thetaPrev;

int main(int argc, char **argv) {

    ros::init(argc, argv, "turtlebot3_nav");
    ros::NodeHandle n("~");
    tf::TransformListener m_listener;
    tf::StampedTransform transform;
/*
 * we use geometry_msgs::twist to specify the linear and angular velocities to the turtlebot. Here we
 * only change the linear velocity only if the time elapsed is less than 10 seconds. Finally, we publish the twist
 * message using m_pubNav publisher which publishes to cmd_vel topic.
 */
    geometry_msgs::Twist msg;
    n.param<std::string>("worldFrame", m_worldFrame, "world");
    n.getParam("frame", m_bodyFrame);

    /*
     * creates a ros publisher that publishes to cmd_vel topic. Turtlebot core is listening to this topic and
     * changes the linear and angular velocities accordingly.
     * geometry_msgs are widely used to communicate linear and angular positions/velocities between ros nodes.
     *
     */
    ros::Publisher m_pubNav = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    m_listener.waitForTransform(m_worldFrame, m_bodyFrame, ros::Time(0), ros::Duration(10.0));
    ros::Rate loop_rate(frequency);

    while (ros::ok()) {
        /*
         * reading the vicon feedback for error calculation. You can calculate your error and set the angular velocity
         * accordingly. Here I have set a fixed angular and linear velocity for demonstration on Rviz.
         */
        m_listener.lookupTransform(m_worldFrame, m_bodyFrame, ros::Time(0), transform);
        float x = transform.getOrigin().x();
        float y = transform.getOrigin().y();
        /**
         * PID calculations should come here
         * After the PID correction set the corrected angular velocity to the tf msg to be sent to the
         * turtlebot. Then publish the values via cmd_vel topic.
         */
        msg.linear.x = linearV;
        msg.angular.z = angularV;
        m_pubNav.publish(msg);

        /*
         * set the rviz visualization
         */
        showOnRviz(linearV, angularV);
    }


    ros::spin();
    return 0;
}

void showOnRviz(float v, float theta_dot) {
    float x, y;
    float theta = theta_dot * dt + thetaPrev;
    x = (v * cos(theta)) * dt + xPrev;
    y = (v * sin(theta)) * dt + yPrev;

    static tf::TransformBroadcaster br;
    tf::Transform transform_rviz;
    tf::Quaternion q;
    q.setRPY(0, 0, theta);
    transform_rviz.setRotation(q);
    transform_rviz.setOrigin(tf::Vector3(x, y, 0));
    // Here I only set the base_link transformations. You can ignore the warnings on Rviz regarding other transformations.
    br.sendTransform(tf::StampedTransform(transform_rviz, ros::Time::now(), "map", "base_link"));

    xPrev = x;
    yPrev = y;
    thetaPrev = theta;
}

