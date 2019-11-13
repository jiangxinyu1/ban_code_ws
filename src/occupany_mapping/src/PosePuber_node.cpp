#include "readfile.cpp"
void Pubpose(ros::Publisher &pose_pub);


/*
     发布位姿信息的函数 
*/
void Pubpose(ros::Publisher &pose_pub)
{
    // 把位姿信息存储到3维向量的vector里
    vector<Eigen::Vector3d> Pose_Vec;
    ReadPoseInformation(Pose_Vec, path_pose);
    // 定义 要发布的位姿变量

    geometry_msgs::Pose pose;
    for (int i = 0; i < Pose_Vec.size(); ++i)
    {
        pose.position.x = Pose_Vec[i](0);
        pose.position.y = Pose_Vec[i](1);
        pose.position.z = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = Pose_Vec[i](0);
        pose_pub.publish(pose);
    }
}



    int main(int argc, char **argv)
{
    ros::init(argc, argv, "PosePuber_node");
    ros::NodeHandle n;
    ros::Publisher PosePub = n.advertise<geometry_msgs::Pose>("pose_msg", 1000);
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        Pubpose(PosePub);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

