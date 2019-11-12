#include <ros/ros.h>
#include <sstream>
#include "readfile.h"

typedef string::size_type  string_size;
const string const_separator = ",";
const string base_path = "/home/banban/workspace/jiangxinyu_ws/ban_code_ws/src/occupany_mapping/data/";
const string path_pose  = base_path + "pose.txt";
const string path_laser_angle = base_path + "scanAngles.txt";
const string path_laser_range = base_path + "ranges.txt";

//判断是否为标识符
bool is_separator(const string &separator, const string &single)
{
    if (separator.size() != single.size())
    {
        cerr << " The separator is wrong ! " << endl;
        return false;
    }
    for (int index = 0; index < separator.size(); ++index)
    {
        if (separator[index] != single[index])
        {
            return false;
        }
    }
    return true;
}

//分离string对象
vector<string> SeparateStr( const string &line, const string &separator)
{
    vector<string> string_vec;
    int front = 0;
    int elem_num = 1;
    string_size i = front;
    while( i < line.size() )
    {
        for ( i = front ; i < line.size(); ++i)
        {
            string tmp = line.substr(i, 1);
            
            if (is_separator(separator, tmp))
            {
                //find the first separator
                elem_num = i - front;
                //copy(front,elem_num)
                string str_1 = line.substr(front, elem_num);
                string_vec.push_back(str_1);
                front = i + 1;
            }
        }
    }
    string str_2 = line.substr(front);
    string_vec.push_back(str_2);
    return string_vec;
}

// 把string对象转化为double
double  Conversion(const string &str)
{
    double  value;
    stringstream ss (str);
    ss >> value; 
    return value;
}

// 读取激光雷达数据
void ReadLaserInformation( LaserScanVec &laserscan  , const string &angle_path ,const string  &range_path)
{
    // 读取角度信息
    string line;
    ifstream is_angle(angle_path.c_str());
    if(is_angle.is_open()==false)
    {
        cerr << "The angle data file open failed !!!" << endl;
        return ;
    }
    while(getline(is_angle,line))
    {
        vector<string> angle = SeparateStr(line,const_separator);
        vector<string>::iterator iter = angle.begin();
        for( ; iter!=angle.end();++iter)
        {
            double angle_ = Conversion(*iter);
            laserscan.angles_vec.push_back(angle_);
        }
    }
    cout << "Complete reading  the angle data. " << endl;
    // 读取距离信息
    ifstream is_range(range_path.c_str());
    if (is_range.is_open() == false)
    {
        cerr << "The range data file open failed !!!" << endl;
        return;
    }
    while (getline(is_range, line))
    {
        vector<string> range = SeparateStr(line, const_separator);
        vector<string>::iterator iter = range.begin();
        for (; iter != range.end(); ++iter)
        {
            double range_ = Conversion(*iter);
            laserscan.ranges_vec.push_back(range_);
        }
    }
    cout << "Complete reading  the angle data. " << endl;
}


// 读取位姿信息
void ReadPoseInformation (vector<Eigen::Vector3d> &pose_vec , const string &path)
{
    string line;
    ifstream is (path.c_str());
    if( is.is_open() == false )
    {
        cerr << "Open the data file failed !!!" << endl;
    }
    while( getline(is , line) )
    {
        vector<string> pose = SeparateStr(line,const_separator);
        double x = Conversion(pose[0]);
        double y = Conversion(pose[1]);
        double theta = Conversion(pose[2]);
        Eigen::Vector3d POSE (x,y,theta);
        pose_vec.push_back(POSE);
    }
}

/*
     发布位姿信息的函数 
*/
// void Pubpose(ros::Publisher & pose_pub)
// {
//     vector<Eigen::Vector3d> Pose_Vec;
//     ReadPoseInformation(Pose_Vec, path_pose);
//     // vector<Eigen::Vector3d>::iterator iter = Pose_Vec.begin();
//     for (int i = 0 ; i < Pose_Vec.size(); ++i)
//     {
//         pose_pub.publish(Pose_Vec[i]);
//     }
// }
