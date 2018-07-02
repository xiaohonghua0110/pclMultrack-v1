#ifndef _UNTIL_HPP
#define _UNTIL_HPP


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <dirent.h>
#include <algorithm> // std::sort

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h> 
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/3dsc.h>


using namespace std;

void finedisplay(vector<vector<int>> cluster,pcl::PointCloud<pcl::PointXYZ>::Ptr Ncloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB);
vector <string> getFiles(string cate_dir);
vector<float>  computeVFHistogram(pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud);
#endif /* _UNTIL_HPP */
