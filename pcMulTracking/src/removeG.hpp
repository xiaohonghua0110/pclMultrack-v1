/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   removeG.hpp
 * Author: xiaohonghua
 *
 * Created on February 8, 2018, 2:36 PM
 */

#ifndef _REMOVEG_HPP
#define _REMOVEG_HPP

#include <stdio.h>
#include <sstream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>

using namespace std;
void int2str(const int &int_temp,string &string_temp);
bool comparePoint(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);
void rmCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &Ncloud,vector<vector<int>> &cluster,vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> &splitPoint);

extern int D,H,W,G_D,G_W,C_D,C_W,C_H;
extern float G_vD, G_vW,C_vD,C_vW,C_vH;
void init_para();
struct CEIL
{
    int num;
    int flag=0;
    bool visit=0;
};
#endif /* REMOVEG_HPP */

