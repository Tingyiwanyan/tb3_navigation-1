//
// Created by malintha on 1/23/18.
//

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>

std::string m_worldFrame;
std::string m_bodyFrame;
static double frequency = 50;
static int translation = 1;

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

    float init_x;
    float x;
    float y;

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
    m_listener.lookupTransform(m_worldFrame, m_bodyFrame, ros::Time(0), transform);
    x = init_x = transform.getOrigin().x();

    while(abs(x - init_x) < 1) {
        // Get the transform between the worldframe and bodyframe by bodyframe ID and save it in transform variable.
        m_listener.lookupTransform(m_worldFrame, m_bodyFrame, ros::Time(0), transform);

        // get the x, y value using the previously received transform.
        x = transform.getOrigin().x();
        y = transform.getOrigin().y();

        ROS_INFO("Current position x:%f y:%f ", x, y);

        msg.linear.x = 0.15;
        m_pubNav.publish(msg);
    }

    msg.linear.x = 0;
    m_pubNav.publish(msg);

    return 0;
}


