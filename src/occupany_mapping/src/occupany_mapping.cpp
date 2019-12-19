#include "occupany_mapping.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point32.h"

/*
 * Increments all the grid cells from (x0, y0) to (x1, y1);
 * //不包含(x1,y1)
 * 2D画线算法　把两个点之间的栅格加到容器里
 * 返回存储栅格下标的容器
 */
std::vector<GridIndex> TraceLine(int x0, int y0, int x1, int y1)
{
  GridIndex tmpIndex;
  std::vector<GridIndex> gridIndexVector;

  bool steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep)
  {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1)
  {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  int deltaX = x1 - x0;
  int deltaY = abs(y1 - y0);
  int error = 0;
  int ystep;
  int y = y0;

  if (y0 < y1)
  {
    ystep = 1;
  }
  else
  {
    ystep = -1;
  }

  int pointX;
  int pointY;
  for (int x = x0; x <= x1; x++)
  {
    if (steep)
    {
      pointX = y;
      pointY = x;
    }
    else
    {
      pointX = x;
      pointY = y;
    }

    error += deltaY;

    if (2 * error >= deltaX)
    {
      y += ystep;
      error -= deltaX; 
    }

    //不包含最后一个点．
    if(pointX == x1 && pointY == y1) continue;

    //保存所有的点
    tmpIndex.SetIndex(pointX,pointY);

    gridIndexVector.push_back(tmpIndex);
  }

  return gridIndexVector;
}

/*  
  设置地图参数的函数
  mapParams是一个在头文件里定义的全局对象
*/
void SetMapParams(void )
{
  //  宽度90
   mapParams.width = 900;
  //  高度90
   mapParams.height = 900;
  //  分辨率 0.04
   mapParams.resolution = 0.04;

   //每次被集中的log变化值
   mapParams.log_free = -1;
   mapParams.log_occ = 2;

   //每个栅格的最大最小值．
   mapParams.log_max = 100.0;
   mapParams.log_min = 0.0;

   mapParams.origin_x = 0.0;
   mapParams.origin_y = 0.0;

   //地图的原点，在地图的正中间
   mapParams.offset_x = 700;
   mapParams.offset_y = 600;

  //  申请一块地图大小的动态内存,存储栅格的数组
   pMap = new double[mapParams.width*mapParams.height];

   // 遍历数组,把每一个栅格的初始值设置为50;
   for(int i = 0; i < mapParams.width * mapParams.height;i++)
   {
     pMap[i] = 50.00;
   }
   std::cout << "Complete set the pMap" << std::endl;
        
}


//从世界坐标系转换到栅格坐标系
GridIndex ConvertWorld2GridIndex(double x,double y)
{
    GridIndex index;

    index.x = std::ceil((x - mapParams.origin_x) / mapParams.resolution) + mapParams.offset_x;
    index.y = std::ceil((y - mapParams.origin_y) / mapParams.resolution) + mapParams.offset_y;

    return index;
}

/*  
  把Index转化成pMap数组里的值
*/
int GridIndexToLinearIndex(GridIndex index)
{
    int linear_index;
    linear_index = index.y + index.x * mapParams.width;
    return linear_index;
}


//判断index是否有效的函数
bool isValidGridIndex(GridIndex index)
{
    if(index.x >= 0 && index.x < mapParams.width && index.y >= 0 && index.y < mapParams.height)
        return true;

    return false;
}

// 释放地图空间的函数
void DestoryMap()
{
    if(pMap != NULL)
        delete pMap;
}


/*  
  覆盖栅格建图算法函数
  输入:激光雷达的每一帧数据 和 机器人每一帧的位姿
 */
void OccupanyMapping(std::vector<GeneralLaserScan>& scans,std::vector<Eigen::Vector3d>& robot_poses)
{
    //遍历每一帧的激光雷达数据
    for(int i = 0; i < scans.size();i++)
    {
      // std::cout << " pMap[20] = " << pMap[20] << std::endl;
      //  定义一个scan存储每一帧激光雷达的数据
      GeneralLaserScan scan = scans[i];
      // 定义一个robotPose存储每一帧机器人的位姿
      Eigen::Vector3d robotPose = robot_poses[i];

      //把机器人的位姿从世界坐标转换到栅格地图坐标
      GridIndex robotIndex = ConvertWorld2GridIndex(robotPose(0), robotPose(1));

      // 遍历当前帧中的每一激光束
      for (int id = 0; id < scan.range_readings.size(); id++)
      {
        double dist = scan.range_readings[id];
        // 激光雷达逆时针转，角度取反
        double angle = -scan.angle_readings[id];
        // 如果距离值是无限大或者非数字,这个距离值跳过
        if (std::isinf(dist) || std::isnan(dist))
          continue;
        // 由距离值和角度值计算激光点在雷达坐标系下的世界坐标
        double theta = -robotPose(2);
        double laser_x = dist * cos(angle);
        double laser_y = dist * sin(angle);
        // 把激光点在雷达坐标系下的世界坐标转换到世界坐标系下
        double world_x = cos(theta) * laser_x - sin(theta) * laser_y + robotPose(0);
        double world_y = sin(theta) * laser_x + cos(theta) * laser_y + robotPose(1);
        // 把激光点转换到栅格坐标下
        GridIndex Index = ConvertWorld2GridIndex(world_x, world_y);
        // 利用画线算法把当前帧当前角度的激光经过的栅格坐标求出来
        if (isValidGridIndex(Index) == 1)
        {
          std::vector<GridIndex> Index_v = TraceLine(robotIndex.x, robotIndex.y, Index.x, Index.y);
          // 更新未被击中的栅格值
          int n = 0;
          for (int m = 0; m < Index_v.size(); m++)
          {
            n = GridIndexToLinearIndex(Index_v[m]);
            pMap[n] = pMap[n] + mapParams.log_free - 1.00;
            if (pMap[n] < 0.00)
            {
              pMap[n] = mapParams.log_min;
            }
          }
          // 更新击中的栅格
          int x = GridIndexToLinearIndex(Index);
          pMap[x] = pMap[x] + mapParams.log_occ -1.00 ;
          if (pMap[x] > mapParams.log_max)
          {
            pMap[x] = mapParams.log_max;
          }
          if (pMap[x] < mapParams.log_min)
          {
            pMap[x] = mapParams.log_min;
          }
        }
        }
    }
}


//发布地图．
void PublishMap(ros::Publisher& map_pub)
{
    nav_msgs::OccupancyGrid rosMap;

    rosMap.info.resolution = mapParams.resolution;
    rosMap.info.origin.position.x = 0.0;
    rosMap.info.origin.position.y = 0.0;
    rosMap.info.origin.position.z = 0.0;
    rosMap.info.origin.orientation.x = 0.0;
    rosMap.info.origin.orientation.y = 0.0;
    rosMap.info.origin.orientation.z = 0.0;
    rosMap.info.origin.orientation.w = 1.0;

    rosMap.info.origin.position.x = mapParams.origin_x;
    rosMap.info.origin.position.y = mapParams.origin_y;
    rosMap.info.width = mapParams.width;
    rosMap.info.height = mapParams.height;
    rosMap.data.resize(rosMap.info.width * rosMap.info.height);

    //0~100
    int cnt0,cnt1,cnt2;
    cnt0 = cnt1 = cnt2 = 100;
    for(int i = 0; i < mapParams.width * mapParams.height;i++)
    {
       if(pMap[i] == 50)
       {
           rosMap.data[i] = -1.0;
       }
       else
       {
         rosMap.data[i] = pMap[i];
       }
    }

    rosMap.header.stamp = ros::Time::now();
    rosMap.header.frame_id = "map";

    map_pub.publish(rosMap);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "OccupanyMapping");

    ros::NodeHandle nodeHandler;

    ros::Publisher mapPub = nodeHandler.advertise<nav_msgs::OccupancyGrid>("laser_map",1,true);

    std::vector<Eigen::Vector3d> robotPoses;
    std::vector<GeneralLaserScan> generalLaserScans;

    std::string basePath = "/home/banban/workspace/jiangxinyu_ws/ban_code_ws/src/occupany_mapping/data";

    std::string posePath= basePath + "/pose.txt";
    std::string anglePath = basePath + "/scanAngles.txt";
    std::string scanPath = basePath + "/ranges.txt";

    //读取数据
    ReadPoseInformation(posePath,robotPoses);

    ReadLaserScanInformation(anglePath,
                             scanPath,
                             generalLaserScans);

    //设置地图信息
    SetMapParams();
    std::cout << "complete the SetMapParams()" << std::endl;

    OccupanyMapping(generalLaserScans,robotPoses);
    std::cout << "complete the OccupanyMapping()" << std::endl;

    PublishMap(mapPub);
    std::cout << "Start  the PublishMap(mapPub);" << std::endl;

    ros::spin();
    std::cout << "complete the ros::spin();" << std::endl;

    DestoryMap();
    std::cout <<"Release Memory!!"<<std::endl;
}