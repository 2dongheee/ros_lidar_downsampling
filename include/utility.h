#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

extern float filterRes_X = 0.6;
extern float filterRes_Y = 0.3;
extern float filterRes_Z = 0.5;
