#include "startracker.h"


StarTracker::StarTracker()
{
    std::string nameSpace = ros::this_node::getName();

    //ros related values
    poseTopicName_   = nh_.param <std::string>(nameSpace + "/ros_topics/abs_topic", "startracker_pose");

    pubPoseStamped    = nh_.advertise<geometry_msgs::PoseStamped>(poseTopicName_, 1);

    port_ = nh_.param(nameSpace + "/device_configuration/port", 18000);
    ROS_INFO("Neo_StartTracker: using UDP Port = %d", port_);

}

StarTracker::~StarTracker()
{

}

int StarTracker::init()
{

    //prepare ports for using
    struct sockaddr_in myaddr; /* address */

    /* create a UDP socket */
    if ((sock_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        ROS_ERROR("Neo_StartTracker: failed to create socket!");
        return 1;
    }

    /* bind the socket to any valid IP address and a specific port */
    memset((char *)&myaddr, 0, sizeof(myaddr));    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    myaddr.sin_port = htons(port_);

    if (bind(sock_, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0)
    {
        ROS_ERROR("Neo_StartTracker: failed to bind socket!");
        return 1;
    }

    ROS_INFO("Neo_StartTracker: initialization successful");
    return 0;
}

int StarTracker::run()
{

    unsigned char rcvBuf[MSG_SIZE];

    //receive data
    struct sockaddr_in remaddr; /* remote address */
    socklen_t addrlen = sizeof(remaddr); /* length of addresses */

    //read data from socket
    int n = recvfrom(sock_, &rcvBuf[0], MSG_SIZE, 0, (struct sockaddr *)&remaddr, &addrlen);

    //evaluate data
    if(n > 0)
    {   
        //ROS_DEBUG("Bytes received: %d", n);

        // if message is received the sensor is ready
        if(rcvBuf[0] == 'p')
        {

            /*//print buffer
            for(int a = 0; a < sizeof(rcvBuf); a++)
            {
                ROS_DEBUG("Buffer Test: %02x", (unsigned char)rcvBuf[a]);
            }*/

            /*Our simplest 6DOF protocol (VR v1) is 25 bytes long, consisting of the character 'p' followed by x,y,z,p (pan/yaw), t (tilt/pitch),
             * r (roll), 4 bytes each:

            pXxxxYyyyZzzzPpppTtttRrrr

            The values are in 4-byte little endian format, and are:
            X,Y,Z - when cast to 4-byte integers, represent linear position in hundredths of a mm (i.e. metres * 10^5)
            P,T,R - when cast to 4-byte integers, represent angles in units of a millionth of a revolution
            (i.e. degrees/360 * 10^6) - we use the range [-10^5, 10^5] = [-180, 180] deg*/

            //read data
            receivedPosition_.x = getInt32(rcvBuf[1],rcvBuf[2],rcvBuf[3],rcvBuf[4]);
            receivedPosition_.y = getInt32(rcvBuf[5],rcvBuf[6],rcvBuf[7],rcvBuf[8]);
            receivedPosition_.z = getInt32(rcvBuf[9],rcvBuf[10],rcvBuf[11],rcvBuf[12]);
            receivedPosition_.yaw = getInt32(rcvBuf[13],rcvBuf[14],rcvBuf[15],rcvBuf[16]);
            receivedPosition_.pitch = getInt32(rcvBuf[17],rcvBuf[18],rcvBuf[19],rcvBuf[20]);
            receivedPosition_.roll = getInt32(rcvBuf[21],rcvBuf[22],rcvBuf[23],rcvBuf[24]);

            //convert from from 10^5 to 10^3 [mm]
            currentPosition_.x = receivedPosition_.x / 100000.0;
            currentPosition_.y = receivedPosition_.y / 100000.0;
            currentPosition_.z = receivedPosition_.z / 100000.0;

            //convert tix to rad
            float r2t = 1e6f / (2.f * M_PI);

            currentPosition_.yaw = receivedPosition_.yaw / r2t;
            currentPosition_.pitch = receivedPosition_.pitch / r2t;
            currentPosition_.roll = receivedPosition_.roll / r2t;

            ROS_INFO("Current Position: %f %f %f Current Angle: %f %f %f", currentPosition_.x, currentPosition_.y, currentPosition_.z, currentPosition_.roll, currentPosition_.pitch, currentPosition_.yaw);

            //convert angle to quaternion
            tf::Matrix3x3 matrix;
            tf::Matrix3x3 rot_to_base;
            tf::Matrix3x3 orientation;
            tf::Quaternion q_tf;

            matrix.setEulerYPR(currentPosition_.yaw,currentPosition_.pitch,currentPosition_.roll);

            double XOffset = 0.0;
            double YOffset = 0.0;
            //apply translation
            currentPosition_.x -= XOffset;
            currentPosition_.y -= YOffset;

            //apply rotation to base_link
            tfScalar yaw = 0.0;
            rot_to_base.setEulerYPR(yaw, 0.0, 0.0);

            orientation = matrix * rot_to_base;

            //get quaternion from Euler angles
            orientation.getRotation(q_tf);

            //create ROS-Msg
            geometry_msgs::PoseStamped startracker_pose;
            startracker_pose.header.frame_id = "map";
            startracker_pose.header.stamp = ros::Time::now();
            startracker_pose.pose.position.x = currentPosition_.x;
            startracker_pose.pose.position.y = currentPosition_.y;
            startracker_pose.pose.position.z = currentPosition_.z;
            startracker_pose.pose.orientation.w = q_tf.getW();
            startracker_pose.pose.orientation.x = q_tf.getX();
            startracker_pose.pose.orientation.y = q_tf.getY();
            startracker_pose.pose.orientation.z = q_tf.getZ();

            //publish
            pubPoseStamped.publish(startracker_pose);


        }
        else
        {
            ROS_INFO("Neo_StartTracker: %c", rcvBuf[0]);
            ROS_INFO("Neo_StartTracker: Bytes %d", n);
            ROS_INFO("Neo_StartTracker: Protocoll wrong format!");
        }

    }

    return 0;
}


int StarTracker::shutdown()
{
    ROS_INFO("Neo_StartTracker: Shutting down Node");
    close(sock_);
}

double StarTracker::getDifference(ros::Time now, ros::Time before)
{
    ros::Duration diff = now - before;
    double diff_d;
    diff_d = diff.sec + diff.nsec / 1000000000.0;
    return diff_d;
}


int32_t StarTracker::getInt32(unsigned char cByte1, unsigned char cByte2, unsigned char cByte3, unsigned char cByte4)
{   
    //int32_t a = (cByte1 << 24) | (cByte2 << 16) | (cByte3 << 8) | cByte4;
    int32_t a = (cByte4 << 24) | (cByte3 << 16) | (cByte2 << 8) | cByte1;
    return a;
}



