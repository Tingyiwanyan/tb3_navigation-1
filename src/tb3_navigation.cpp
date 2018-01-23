//
// Created by malintha on 1/23/18.
//

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>

class tb3_navigation {

public:
    tb3_navigation(const std::string &worldFrame, const std::string &frame,
                  const ros::NodeHandle &n):m_bodyFrame(frame), m_worldFrame(worldFrame) {

        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_listener.waitForTransform(m_worldFrame, m_bodyFrame, ros::Time(0), ros::Duration(10.0));

    }

    void run(double frequency) {
        this->frequency = frequency;
        ros::Timer timer = nh.createTimer(ros::Duration(1.0 / frequency), &tb3_navigation::iteration, this);
        ros::spin();
    }

    void iteration(const ros::TimerEvent &e) {
        tf::StampedTransform transform;
        m_listener.lookupTransform(m_worldFrame, m_bodyFrame, ros::Time(0), transform);
        float x = transform.getOrigin().x();
        float y = transform.getOrigin().y();
        float w = transform.getOrigin().w();
        ROS_INFO("Current position: %f %f", x, y);

        if(x < 1.5 && y < 1.5) {
            geometry_msgs::Twist msg;
            msg.linear.x = 0.15;
            m_pubNav.publish(msg);
        }
    }

private:
    std::string m_worldFrame;
    std::string m_bodyFrame;
    ros::Publisher m_pubNav;
    float frequency;
    tf::TransformListener m_listener;
    ros::NodeHandle nh;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtlebot3_nav");
    static ros::NodeHandle n("~");
    std::string worldFrame;
    n.param<std::string>("/turtle/worldFrame", worldFrame, "/world");
    std::string frame;
    n.param<std::string>("/turtle/frame", frame,"vicon/raphael/raphael");
    double frequency;
    n.param("frequency", frequency, 50.0);
    tb3_navigation navigator(worldFrame, frame, n);
    navigator.run(frequency);
    return 0;
}
