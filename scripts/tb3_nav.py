#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import tf
import geometry_msgs.msg

def controller():
    rospy.init_node("tb3_nav_py")
    frequency = 10
    rate = rospy.Rate(frequency)
    m_worldFrame = "world"
    m_bodyFrame = "vicon/splinter/splinter"
    listener = tf.TransformListener()
    listener.waitForTransform(m_worldFrame, m_bodyFrame, rospy.Time(), rospy.Duration(5.0))
    pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    goalsX = [0.1,0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]
    goalsY = [0.1,0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]
    goalsTheta = [math.pi/4, math.pi/4, math.pi/4, math.pi/4, math.pi/4, math.pi/4, math.pi/4, math.pi/4, math.pi/4, math.pi/4]
    counter = 0
    num_goals = len(goalsX)
    dt = 0.1

    # add the variables here
    m_kp = 0.1
    m_kd = 0.1
    m_ki = 0.1
    m_minoutput = -0.1
    m_maxoutput = 0.1
    m_integratormin = -0.01
    m_integratormax = 0.01
    m_integral = 0
    m_previouserror = 0
    time_elapsed = 0

    while (((counter < num_goals) and time_elapsed<30) and not rospy.is_shutdown()):
        time_elapsed = time_elapsed + dt
        (trans, rot) = listener.lookupTransform(m_worldFrame,m_bodyFrame, rospy.Time())
        x1 = goalsX[counter]
        y1 = goalsY[counter]
        theta1 = goalsTheta[counter]

        x = trans[0]
        y = trans[1]
        (r, p, theta) = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
        log_state = "goal state: %s %s %s \t current state: %s %s %s\n" % (x1, y1, theta1, x, y, theta)
        # print(log_state)

        # do the PID calculation here

        msg = geometry_msgs.msg.Twist()
        # msg.linear.x = set the linear velocity output to this
        msg.linear.x = 0.1
        # msg.angular.z = set the angular velocity output to this
        msg.angular.z = 0
        error = theta1 - theta
        p = m_kp * error
        m_integral = m_integral + error
        i  = max(min(m_ki * m_integral, m_integratormax), m_integratormin)
        d = 0
        d = m_kd * (error - m_previouserror) / 0.1
        output = p + d + i
        msg.angular.z = max(min(output, m_maxoutput), m_minoutput)
        print("error: %s, theta: %s, targettheta: %s, output: %s, p: %s, i: %s\n" %(error,theta,theta1,output,p,i))

        m_previouserror = error

        if(counter == num_goals-1):
            msg.linear.x = 0
            msg.angular.z = 0
            print "the last msg"
        pub.publish(msg)

        # uncomment below when running with tb3
        if(math.sqrt((x-x1)*(x-x1) + (y-y1)*(y-y1)) < 0.1):
            counter += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
