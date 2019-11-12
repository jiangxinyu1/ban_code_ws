#include "readfile.cpp"

int main(int argc, char **argv)
{
     ros::init(argc, argv, "LaserPuber_node");
     ros::NodeHandle n;

     ros::spin();

    return 0;
}
