//
// Created by malintha on 1/23/18.
//

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>

class tb3_navigation {

public:
    /**
     * The constructoro for tb3_navigation node.
     * @param worldFrame The world frame specified in the launch file. (world)
     * @param frame Body frame of the turtlebot as specified in the launch file
     * @param n Ros node handle
     */
    tb3_navigation(const std::string &worldFrame, const std::string &frame,
                   const ros::NodeHandle &n) : m_bodyFrame(frame), m_worldFrame(worldFrame), elapsed_time(0) {

        // creates a ros publisher that publishes to cmd_vel topic. Turtlebot core is listening to this topic and
        // changes the linear and angular velocities accordingly.
        // geometry_msgs are widely used to communicate linear and angular positions/velocities between ros nodes.
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        // m_listener is listening to incoming transforms from the vicon_server. Here, we wait for 10 seconds
        // till we receive the first transform from the vicon server. m_bodyFrame is the corresponding vicon object for
        // the turtlebot that we specified in the launch file.
        m_listener.waitForTransform(m_worldFrame, m_bodyFrame, ros::Time(0), ros::Duration(10.0));
    }

    void run(double frequency) {
        this->frequency = frequency;
        ros::Timer timer = nh.createTimer(ros::Duration(1.0 / frequency), &tb3_navigation::iteration, this);
        ros::spin();
    }
/**
 * This method is being fired at every ros iteration (at 50Hz as defined in the main function). You can change the
 * frequency as you like and integrate your control code within this method.
 * @param e
 */
    void iteration(const ros::TimerEvent &e) {
        tf::StampedTransform transform;
        // Get the transform between the worldframe and bodyframe by bodyframe ID and save it in transform variable.
        m_listener.lookupTransform(m_worldFrame, m_bodyFrame, ros::Time(0), transform);

        // get the x, y, z, and orientation value using the previously received transform.
        float x = transform.getOrigin().x();
        float y = transform.getOrigin().y();
        float w = transform.getOrigin().w();
        elapsed_time += 1 / frequency;
        ROS_INFO("Current position x:%f y:%f w:%f time: %f", x, y, w, elapsed_time);

        // we use geometry_msgs::twist to specify the linear and angular velocities to the turtlebot. Here we
        // only change the linear velocity only if the time elapsed is less than 10 seconds. Finally, we publish the twist
        // message using m_pubNav publisher which publishes to cmd_vel topic.
        geometry_msgs::Twist msg;
        msg.linear.x = 0;

        if (elapsed_time < 10) {
            msg.linear.x = 0.15;
        }

        m_pubNav.publish(msg);

    }

private:
    std::string m_worldFrame;
    std::string m_bodyFrame;
    ros::Publisher m_pubNav;
    float frequency;
    tf::TransformListener m_listener;
    ros::NodeHandle nh;
    double elapsed_time;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtlebot3_nav");
    static ros::NodeHandle n("~");
    std::string worldFrame;
    n.param<std::string>("worldFrame", worldFrame, "/world");
    std::string frame;
    n.getParam("frame", frame);
    double frequency;
    n.param("frequency", frequency, 50.0);
    tb3_navigation navigator(worldFrame, frame, n);
    navigator.run(frequency);
    return 0;
}
