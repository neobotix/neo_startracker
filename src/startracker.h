#ifndef STARTRACKER_H
#define STARTRACKER_H




#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>


#define MSG_SIZE 25

class StarTracker
{
public:
    StarTracker();
    ~StarTracker();

    int init();
    void run(const ros::TimerEvent& e);
    int shutdown();

    ros::Time absTimeStamp_, prevAbsTimeStamp_;

    ros::NodeHandle rosNode_;

    struct Data
    {
        int32_t x;
        int32_t y;
        int32_t z;
        int32_t roll;
        int32_t pitch;
        int32_t yaw;
    };

    struct Pose
    {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
    };

    Data receivedPosition_;
    Pose currentPosition_;
    Pose oldPosition_;

    //UDP-IP Settings
    std::string IP_;
    int port_;

private:

    int sock_; /* our socket */

    double getDifference(ros::Time now, ros::Time before);
    int32_t getInt32(unsigned char cByte1, unsigned char cByte2, unsigned char cByte3, unsigned char cByte4);

    //ROS NodeHandle
    ros::NodeHandle nh_;

    //ROS Publisher
    ros::Publisher m_pubPoseStamped;
    ros::Publisher m_pubPose2d;
    ros::Publisher m_pubTwist;

    //ROS TF
    tf2_ros::TransformBroadcaster m_tfBCMapToWorld;
    geometry_msgs::TransformStamped m_msgTrafoWorldToOdom; // My frames are named "base_link" and "leap_motion"

    //ROS Timer
    ros::Timer m_timerRun;

    //ROS Poses
    geometry_msgs::PoseStamped startracker_world_zero;

    //ROS msgs
    nav_msgs::Odometry m_msgOdom;

    //frames
    std::string absOdomFrame_, relOdomFrame_, absSensorFrame_, relSensorFrame_, driftFrame_;
    //topic names
    std::string poseTopicName_;

    
    

    bool m_bInitialized;

    int relOdomOn_, absOdomOn_, driftOn_, driftVisOn_, diagnosticsOn_, infosOn_;
    int recvBytes_, sendBytes_;
    int driftCount_;
    int infoCount_, qrAmount_;
    int prevID_;

    bool m_bWorldTransformAvailable;

};

#endif // STARTRACKER_H
