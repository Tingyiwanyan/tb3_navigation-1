//
// Created by malintha on 2/10/18.
//

#ifndef PROJECT_PIDIMPL_H
#define PROJECT_PIDIMPL_H

#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include <std_msgs/Float32.h>

using namespace Eigen;

class pidImpl {
public:
    pidImpl(
            float kp,
            float kd,
            float ki,
            float dt,
            double t)
            : m_kp(kp)
            , m_kd(kd)
            , m_ki(ki)
            , m_minOutput(-0.1)
            , m_maxOutput(0.1)
            , m_integratorMin(-0.0001)
            , m_integratorMax(0.0001)
            , m_integral(0)
            , m_previousError(0)
            , m_previousTime(t),

              dt(dt)
    {
        error_pub = nh.advertise<std_msgs::Float32>("pid_error", 10);
    }
    float d;
    float update(double targettheta, Vector2d targetX, Vector2d X, float theta, double t)
    {
//        float error = sqrt(pow(X[0] - targetX[0], 2) + pow(X[1] - targetX[1], 2));
//        float targettheta = atan2(targetX[1] - X[1], targetX[0] - X[0]);
//        if(targettheta < 0)
//            targettheta = -targettheta;
        float error = targettheta - theta;

        if(error < 0)
            error = -error;
        double time = t;
            float p = m_kp * error;
            m_integral += error * dt;
            float i = std::max(std::min(m_ki * m_integral, m_integratorMax), m_integratorMin);
            d = 0;
            if (dt > 0) {
                d = m_kd * (error - m_previousError) / dt;
            }
            float output = std::max(std::min(p + d + i, m_maxOutput), m_minOutput);
            std::cout<<"error: "<<error<<"  "<<"  "<<theta<<"  "<<"  "<<targettheta<<"  "<<output<<"     pid: "<<p<<"     "<<i<<"     "<<d<<std::endl;
            m_previousError = error;
            m_previousTime = time;
        if(output == -2)
            output = output_prev;
        output_prev = output;

        return output;
    }

    void pidreset() {
        m_integral = 0;
    }

    void publishError(float er) {
        std_msgs::Float32 msg_error;
        msg_error.data = er;
        error_pub.publish(msg_error);
    }

private:
    float m_kp;
    float m_kd;
    float m_ki;
    float m_minOutput;
    float m_maxOutput;
    float m_integratorMin;
    float m_integratorMax;
    float m_integral;
    float m_previousError;
    double m_previousTime;
    float dt;
    ros::Publisher error_pub;
    ros::NodeHandle nh;
    float output_prev;
};


#endif
