#include "readfile.cpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PosePuber_node.cpp");
    ros::NodeHandle n;
    //  ros::Publisher posePub = n.advertise<Eigen::Vector3d>("pose_data", 1000);
    //  Pubpose(posePub);
     ros::spin();
    return 0;
}