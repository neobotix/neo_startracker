#include "startracker.h"


StarTracker::StarTracker()
{
    std::string nameSpace = ros::this_node::getName();

    //ros related values
    poseTopicName_  = nh_.param <std::string>(nameSpace + "/ros_topics/abs_topic", "startracker_pose");

    m_pubPoseStamped = nh_.advertise<geometry_msgs::PoseStamped>("startracker_pose", 1);
    m_pubPose2d = nh_.advertise<geometry_msgs::Pose2D>("startracker_pose2d", 1);
    m_pubTwist = nh_.advertise<geometry_msgs::Twist>("startracker_twist", 1);

    port_ = nh_.param(nameSpace + "/device_configuration/port", 18000);
    ROS_INFO("Neo_StartTracker: using UDP Port = %d", port_);

    //set up timer to cyclically call run-method
    m_timerRun = nh_.createTimer(ros::Duration(0.02), &StarTracker::run, this);

    m_bInitialized = false;


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

    m_bWorldTransformAvailable = false;


    return 0;
}

void StarTracker::run(const ros::TimerEvent& e)
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
            currentPosition_.x = round(receivedPosition_.x / 100000.0 * 10000)/10000;
            currentPosition_.y = round(receivedPosition_.y / 100000.0 * 10000)/10000;
            currentPosition_.z = round(receivedPosition_.z / 100000.0 * 10000)/10000;

            //convert tix to rad
            float r2t = 1e6f / (2.f * M_PI);

            currentPosition_.yaw = round(receivedPosition_.yaw / r2t * 10000) / 10000;
            currentPosition_.pitch = round(receivedPosition_.pitch / r2t * 10000) / 10000;
            currentPosition_.roll = round(receivedPosition_.roll / r2t * 10000) / 10000;

            ROS_INFO("--------------------------------");
            ROS_INFO("Star Frame Position: %f %f %f ", currentPosition_.x, currentPosition_.y, currentPosition_.z);
            ROS_INFO("Star Frame Angle: %f %f %f", currentPosition_.roll*180/3.145, currentPosition_.pitch*180/3.145, currentPosition_.yaw*180/3.145);

            
            //convert angles to angle vel
            float vel_x = (currentPosition_.x - oldPosition_.x) / 0.02;
            float vel_y = (currentPosition_.y - oldPosition_.y) / 0.02;
            float vel_alpha = (currentPosition_.yaw - oldPosition_.yaw) / 0.02;

            //ROS_INFO("Current Velocities: %f %f %f ", vel_x, vel_y, vel_alpha);

            tf2::Quaternion myQuat;
            geometry_msgs::Quaternion quat_msg;

            if(!m_bInitialized)
            {
                //calculate transformation from world to odom
                //transform world position to robot odom frame
                

                //base_link_to_leap_motion = tfBuffer.lookupTransform("leap_motion", "base_link", ros::Time(0), ros::Duration(1.0) );

                m_msgTrafoWorldToOdom.header.stamp = ros::Time::now();
                //base_link_to_leap_motion.header.seq = 
                m_msgTrafoWorldToOdom.header.frame_id = "world";
                m_msgTrafoWorldToOdom.child_frame_id = "odom";
                
                m_msgTrafoWorldToOdom.transform.translation.x = 1.0;//currentPosition_.x;
                m_msgTrafoWorldToOdom.transform.translation.y = 1.0;//currentPosition_.y;
                m_msgTrafoWorldToOdom.transform.translation.z = 0.0;//currentPosition_.z;

                tf2::Quaternion myQuaternion;
                myQuaternion.setRPY(0,0,-1.57);//currentPosition_.roll,currentPosition_.pitch,currentPosition_.yaw);//currentPosition_.roll, currentPosition_.pitch, currentPosition_.yaw );  // Create this quaternion from roll/pitch/yaw (in radians)
                //myQuaternion.normalize();
                //myQuaternion.inverse();

                
                quat_msg = tf2::toMsg(myQuaternion);

                m_msgTrafoWorldToOdom.transform.rotation.x = quat_msg.x;
                m_msgTrafoWorldToOdom.transform.rotation.y = quat_msg.y;
                m_msgTrafoWorldToOdom.transform.rotation.z = quat_msg.z;
                m_msgTrafoWorldToOdom.transform.rotation.w = quat_msg.w;

                ROS_INFO("quad msg: %f %f %f %f", quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);

                /*m_msgTrafoWorldToOdom.transform.rotation.x = myQuaternion.getX;
                m_msgTrafoWorldToOdom.transform.rotation.y = myQuaternion.getY;
                m_msgTrafoWorldToOdom.transform.rotation.z = myQuaternion.getZ;
                m_msgTrafoWorldToOdom.transform.rotation.w = myQuaternion.getW;*/

                //initialization done
                m_bInitialized = true;

            }

            //save old position
            oldPosition_.x = currentPosition_.x;
            oldPosition_.y = currentPosition_.y;
            oldPosition_.z = currentPosition_.z;
            oldPosition_.roll = currentPosition_.roll;
            oldPosition_.pitch = currentPosition_.pitch;
            oldPosition_.yaw = currentPosition_.yaw;

            //create pose stamped
            geometry_msgs::PoseStamped msgPoseStamped;
            msgPoseStamped.header.stamp = ros::Time::now();
            msgPoseStamped.pose.position.x = currentPosition_.x;
            msgPoseStamped.pose.position.y = currentPosition_.y;
            msgPoseStamped.pose.position.z = currentPosition_.z;
            tf2::Quaternion tmpQuat1;
            tmpQuat1.setRPY(currentPosition_.roll,currentPosition_.pitch,currentPosition_.yaw);
            geometry_msgs::Quaternion msgQuat1;
            msgQuat1 = tf2::toMsg(tmpQuat1);
            msgPoseStamped.pose.orientation = msgQuat1;

            //pulish pose stamped
            m_pubPoseStamped.publish(msgPoseStamped);

            //create geometry_msgs twist
            geometry_msgs::Twist msgTwist;
            msgTwist.linear.x = vel_x;
            msgTwist.linear.y = vel_y;
            msgTwist.angular.z = vel_alpha;
            
            //publish geometry_msgs twist
            m_pubTwist.publish(msgTwist);

            //create geometry_msgs Pose2D
            geometry_msgs::Pose2D msgPose2d;
            msgPose2d.x = currentPosition_.x;
            msgPose2d.y = currentPosition_.y;
            msgPose2d.theta = currentPosition_.yaw;
            
            //publish geometry_msgs Pose2D
            m_pubPose2d.publish(msgPose2d);


            /*geometry_msgs::PoseStamped pose;
            pose.pose.position.x = 1.0;//currentPosition_.x;
            pose.pose.position.y = 1.0;//currentPosition_.y;
            pose.pose.position.z = 0.0;//currentPosition_.z;

            //tf2::Quaternion myQuaternion;
            myQuat.setRPY(0,0,0);//currentPosition_.roll,currentPosition_.pitch,currentPosition_.yaw);//currentPosition_.roll, currentPosition_.pitch, currentPosition_.yaw );  // Create this quaternion from roll/pitch/yaw (in radians)
            //myQuaternion.normalize();

            geometry_msgs::Quaternion quat_msg2;
            quat_msg2 = tf2::toMsg(myQuat);

            pose.pose.orientation.x = quat_msg2.x;
            pose.pose.orientation.y = quat_msg2.y;
            pose.pose.orientation.z = quat_msg2.z;
            pose.pose.orientation.w = quat_msg2.w;

            tf2Scalar yaw1, pitch1, roll1;
            tf2::Matrix3x3 mat1(myQuat);
            mat1.getEulerYPR(yaw1, pitch1, roll1);

            ROS_INFO("Pose in Star Frame: %f %f %f ", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            ROS_INFO("Angles in Star Frame: %f %f %f", roll1*180/3.145, pitch1*180/3.145, yaw1*180/3.145);


            tf2::doTransform(pose, pose, m_msgTrafoWorldToOdom); // robotPose is the PoseStamped I want to transform
            

            tf2::Quaternion myQuat3;
            myQuat3.setX(pose.pose.orientation.x);
            myQuat3.setY(pose.pose.orientation.y);
            myQuat3.setZ(pose.pose.orientation.z);
            myQuat3.setW(pose.pose.orientation.w);

            tf2Scalar yaw, pitch, roll;
            tf2::Matrix3x3 mat(myQuat3);
            mat.getEulerYPR(yaw, pitch, roll);

            ROS_INFO("Pose in Odom Frame: %f %f %f ", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            ROS_INFO("Angles in Odom Frame: %f %f %f", roll*180/3.145, pitch*180/3.145, yaw*180/3.145);


            //create odometry msg 
            nav_msgs::Odometry msgWorldOdom;
            msgWorldOdom.pose.pose.position.x = pose.pose.position.x;
            msgWorldOdom.pose.pose.position.y = pose.pose.position.y;
            msgWorldOdom.pose.pose.position.z = pose.pose.position.z;*/

            


        }
        else
        {
            ROS_INFO("Neo_StartTracker: %c", rcvBuf[0]);
            ROS_INFO("Neo_StartTracker: Bytes %d", n);
            ROS_INFO("Neo_StartTracker: Protocoll wrong format!");
        }

    }
}

/*int StarTracker::callbackOdom()
{
    ROS_INFO("Neo_StartTracker: Shutting down Node");
    close(sock_);
}*/

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



