#ifndef  READFILE_H
#define READFILE_H

#include<iostream>
#include <fstream>
#include<vector>
#include<string>
#include<iterator>
#include<eigen3/Eigen/Core>
#include<algorithm>
#include <ros/ros.h>

using namespace std;

// 读取的最大的数据数目 
const int READ_MAX_NUMBER = 100;


/*  
    定义存储laser数据的结构体
*/
typedef struct {
    vector<double> angles_vec;
    vector<double> ranges_vec; 
}LaserScanVec;

/* 
    从对象is读取激光雷达数据，存到laserscan_vec里面
*/
void ReadLaserInformation(LaserScanVec &laserscan, const string &angle_path, const string &range_path);
    /*  
    从对象is读取位姿信息，存储到pose_vec里
*/
void ReadPoseInformation(vector<Eigen::Vector3d> &pose_vec, const string &path);

    /*
    把string对象通过分隔符打散成多个string对象
    存储在vector里面，返回指针
*/
    vector<string> SeparateStr(const string &line, const string &separator);

/*  
    把string对象转换成double对象的函数
*/
double  Conversion(const string & str );

/*  
    判断string是否是separator
    判断两个STRING对象是否相等
*/
bool is_separator(const string& separator , const string & single);

void Pubpose(ros::Publisher &pose_pub);

#endif  