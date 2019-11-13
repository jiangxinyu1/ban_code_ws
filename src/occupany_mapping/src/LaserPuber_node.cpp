#include "readfile.cpp"

int main(int argc, char **argv)
{
     ros::init(argc, argv, "LaserPuber_node");
     ros::NodeHandle n;
     //ros::Publisher laser_puber = n.advertise<LaserScanVec>("");  
     



     ros::spin();

    return 0;
}
