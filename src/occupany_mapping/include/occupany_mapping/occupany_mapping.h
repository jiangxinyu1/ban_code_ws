#ifndef OCCUPANY_MAPPING_H
#define OCCUPANY_MAPPING_H

#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <eigen3/Eigen/Core>

#include "readfile.h"


// 设置栅格下标
typedef struct gridindex_
{
    int x;
    int y;

    void SetIndex(int x_,int y_)
    {
        x  = x_;
        y  = y_;
    }
}GridIndex;


// 地图的相关参数,后续用这个结构体变量给rosmap赋值
typedef struct map_params
{
    // 
    double log_occ,log_free;
    // 
    double log_max,log_min;
    
    double resolution;
    // 地图的原点
    double origin_x,origin_y;
    // 地图的size
    int height,width;
    // 
    int offset_x,offset_y;
}MapParams;


MapParams mapParams;

unsigned char* pMap;
  

#endif

