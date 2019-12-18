#ifndef READFILE_H
#define READFILE_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <iostream>


//max 3000
#define READ_DATA_NUMBER  300

/* 
    结构体变量GeneralLaserScan
    有两个vector<double>类型成员变量
    分别存储激光雷达的角度值和距离值
*/
typedef struct general_laser_scan
{
    std::vector<double> range_readings;
    std::vector<double> angle_readings;
}GeneralLaserScan;


/*
    读取位姿信息的函数
    @param 1 : 文件位置
    @param 2 : 存储'Eigen::Vector3d'的 vector reference
*/
void ReadPoseInformation(const std::string path,std::vector<Eigen::Vector3d>& poses);

/*  
    读取激光雷达数据的函数
    @param 1 : 文件位置
    @param 2 : 存储GeneralLaserScan的 vector reference
*/
void ReadLaserScanInformation(const std::string anglePath,
                              const std::string laserPath,
                              std::vector< GeneralLaserScan >& laserscans);

#endif
