#ifndef STARTRACKER_H
#define STARTRACKER_H




#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

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
    int run();
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
    ros::Publisher pubPoseStamped;

    //ROS TF
    tf::TransformBroadcaster absOdomBroad_, relOdomBroad_;

    //ROS msgs
    nav_msgs::Odometry relOdomMsg_, absOdomMsg_;
    geometry_msgs::PoseWithCovarianceStamped driftMsg_;
    geometry_msgs::PoseStamped driftMsgVis_;
    geometry_msgs::TransformStamped absOdomTrans_, relOdomTrans_;
    geometry_msgs::Quaternion absOdomQuat_, relOdomQuat_;

    //frames
    std::string absOdomFrame_, relOdomFrame_, absSensorFrame_, relSensorFrame_, driftFrame_;
    //topic names
    std::string poseTopicName_;

    int relOdomOn_, absOdomOn_, driftOn_, driftVisOn_, diagnosticsOn_, infosOn_;
    int recvBytes_, sendBytes_;
    int driftCount_;
    int infoCount_, qrAmount_;
    int prevID_;

};

#endif // STARTRACKER_H
