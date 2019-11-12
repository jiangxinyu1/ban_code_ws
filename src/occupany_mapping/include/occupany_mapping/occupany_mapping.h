#ifndef  OCCUPANY_MAPPING_H
#define OCCUPANY_MAPPING_H
#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include "readfile.h"

using namespace std;

/*
    地图栅格下标的结构体
*/
typedef struct{
    int x;
    int y;
    // 设置下标的函数
    void SetIndex(int x_ , int y_ )
    {
        x = x_;
        y = y_;
    }
} GridIndex;


/*  
    地图的结构体
*/
/* typedef struct{

}MapParam; */
































#endif