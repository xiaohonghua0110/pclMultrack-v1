#include "removeG.hpp"
#include "until.hpp"
#include <cmath>
#include "matplotlibcpp.h"
#include <unordered_map>
#include <pcl/io/pcd_io.h>
using namespace std;
namespace plt = matplotlibcpp;

int MAXCLUSTER=1000;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);  
pcl::PointCloud<pcl::PointXYZ>::Ptr Ncloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::visualization::PCLVisualizer viewer;
pcl::visualization::PCLVisualizer viewer1;
float Thmac=5;
int IDIndex=0;
int PCindex=0; 
uint32_t rgb[15]={14706431,16724016, 49151,16745131,8388736,16744192,255,16766720,16776960, 10025880,65535, 11403055,16711935,13034239,9124410};
int r[15]={255,255,255,148,155,171,0,100,0,0,0,0,255,255,255};
int g[15]={110,0,131,0,48,130,0,149,191,245,238,255,255,215,165};
int b[15]={180,255,250,211,255,255,255,237,255,255,118,0,0,0,0};
struct FEATURE
{
    Eigen::Vector4f cetriod;
    vector <float> VFHFeature;
    uint32_t ID;
};
//update parameters

vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> lastClusterPoint(MAXCLUSTER); //cluster
vector<FEATURE> lastClusterFeature;


//multi trace
vector<vector<string>> trace(10);


//update parameters

void exCluster(vector<vector<int>> cluster,vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> splitPoint, vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusterPoint,int &clusterNum)
{

int colorcount=0;
/*display cluster in cube*/

    for(int i=0;i<cluster.size();i++)
    {   
        if(cluster[i].size()>5){
            clusterPoint[clusterNum].reset(new pcl::PointCloud<pcl::PointXYZ>());  
            
            for(int j=0;j<cluster[i].size();j++)
            {
                for(int m=0;m<splitPoint[cluster[i][j]]->points.size();m++)
                    clusterPoint[clusterNum]->points.push_back(splitPoint[cluster[i][j]]->points[m]);
            }
          
            stringstream ss;string name;
            ss<<i;ss>>name;
         
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> points_color_handler (clusterPoint[clusterNum], r[colorcount],g[colorcount], b[colorcount]);
            viewer.addPointCloud(clusterPoint[clusterNum],points_color_handler,name);
            colorcount+=1;
            clusterNum+=1;
            if (colorcount==15)
                colorcount=0;
            
        }
              
    }
    clusterNum-=1;
    cout<<clusterNum<<endl;

}
void getClusterFeature(int clusterNum,vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterPoint,vector<FEATURE> &clusterFeature){
    for (int i=0;i<clusterNum;i++)
    {
        clusterFeature[i].VFHFeature.resize(308);
        clusterFeature[i].VFHFeature=computeVFHistogram(clusterPoint[i]);
        pcl::compute3DCentroid( *clusterPoint[i], clusterFeature[i].cetriod);
    }
 
}
double VFHDis(vector<float> a ,vector<float> b){

    double temp0=0;
    double result=0;
    for(int i=0;i<a.size();i++){
        temp0+=pow(a[i]-b[i],2);
    
    }
    result=sqrt(temp0)/10;
    return result;
}
double EurDis( Eigen::Vector4f A , Eigen::Vector4f B){
  
    double temp0=0;
    double result=0;
    for(int i=0;i<3;i++){
        temp0+=pow(A(i)-B(i),2);
    
    }
    result=sqrt(temp0);
    return result;
}
void updateTrace(int clusterNum, vector<FEATURE> &clusterFeature,int lastClusterNum,vector<FEATURE>lastClusterFeature){

    cout<<"clusterNum: "<<clusterNum<<endl;
    cout<<"lastClusterNum: "<<lastClusterNum<<endl;
    for (int i=0;i<clusterNum;i++)
    {
        double dis=0;double minDis=10000;int minindex=0;float p1;float p2;
        for(int j=0;j<lastClusterNum;j++)
        {
            
             p1=EurDis(clusterFeature[i].cetriod,lastClusterFeature[j].cetriod);
             p2=VFHDis(clusterFeature[i].VFHFeature,lastClusterFeature[j].VFHFeature);
            dis=p1+p2;
            
            if (p1+p2<minDis)
            {
                minDis=p1+p2;
                minindex=j;
            }
            if (p1<15&p2<10)
            {
                clusterFeature[i].ID=lastClusterFeature[minindex].ID;
   
            }
            
            else 
            {
                clusterFeature[i].ID=IDIndex;
                IDIndex+=1;
            }
        }      
                cout<<"minDisp1: "<<p1<<endl;
                cout<<"minDisp2: "<<p2<<endl;
                stringstream ss;string ID;
                ss<<clusterFeature[i].ID;ss>>ID;
                trace[PCindex].push_back(ID);
                pcl::PointXYZ center;
                center.x=clusterFeature[i].cetriod[0];
                center.y=clusterFeature[i].cetriod[1];
                center.z=clusterFeature[i].cetriod[2];
                
                stringstream sss;string IID;
                sss<<PCindex;sss>>IID;
                viewer.addText3D(ID,center,0.2, 0, 1, 0,IID+"ID"+ID);
        
    }
}
void initTrace(int clusterNum, vector<FEATURE> &clusterFeature)
{
    for (int i=0;i<clusterNum;i++)
    {
        clusterFeature[i].ID=IDIndex;
        IDIndex+=1;
    
        stringstream ss;string ID;
        ss<<clusterFeature[i].ID;ss>>ID;
        trace[PCindex].push_back(ID);
        pcl::PointXYZ center;
        center.x=clusterFeature[i].cetriod[0];
        center.y=clusterFeature[i].cetriod[1];
        center.z=clusterFeature[i].cetriod[2];
        viewer.addText3D(ID,center,0.2, 0, 1, 0,"ID"+ID);
    }
    cout<<"initIDIndex: "<<IDIndex<<endl;
}

void updateLast(int clusterNum,vector<FEATURE>clusterFeature,int &lastClusterNum,vector<FEATURE> &lastClusterFeature){

    lastClusterNum=clusterNum;
    lastClusterFeature.resize(clusterNum);
     for (int i=0;i<clusterNum;i++)
    {
        lastClusterFeature[i].VFHFeature.resize(308);
     }
    lastClusterFeature=clusterFeature;    
}

int main()
{

    init_para();
    
    int lastClusterNum=0;
    string fileIndex="0000"; 
    int startIndex=0;
    int stopIndex=10;
    vector <string> files;
    files=getFiles("/home/ashley/Documents/lidar/"+fileIndex+"pcd");
    
    
    while(PCindex<stopIndex-startIndex){
    Ncloud->clear();
    cloud->clear();
    pcl::io::loadPCDFile("/home/ashley/Documents/lidar/"+fileIndex+"pcd/"+files[PCindex+startIndex], *cloud);    



    Eigen::Matrix4f m1 = Eigen::Matrix4f::Identity();

           m1 << 0,-1,0,0,
                   0,0,1,0,
                   1,0,0,0,
                   0,0,0,1;

    pcl::transformPointCloud(*cloud,*cloud,m1);
    
    vector<vector<int>> cluster(MAXCLUSTER);
    vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> splitPoint(C_D*C_H*C_W); //all point into voxel
    for (int i=0;i<C_D*C_H*C_W;i++)
        splitPoint[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  
    rmCluster(cloud,Ncloud,cluster,splitPoint);

    viewer.removeAllPointClouds();
    viewer1.removeAllPointClouds();
    viewer.removeAllShapes();
    viewer.addPointCloud(Ncloud,"Ncloud");
    vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterPoint(MAXCLUSTER); //cluster
    int clusterNum=0;

    exCluster(cluster,splitPoint,clusterPoint,clusterNum);
    vector<FEATURE> clusterFeature(clusterNum);

    getClusterFeature(clusterNum,clusterPoint,clusterFeature);

    if(PCindex==0){
        initTrace(clusterNum,clusterFeature);   
    }
    else{
        updateTrace(clusterNum,clusterFeature,lastClusterNum,lastClusterFeature);
    }

    updateLast(clusterNum,clusterFeature,lastClusterNum,lastClusterFeature); 

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> points_color_handler (Ncloud, 255, 0, 255);
    viewer1.addPointCloud(cloud,"cloud");
    viewer1.addPointCloud(Ncloud, points_color_handler,"Ncloud");
  
    cout<<"END"<<endl;
    PCindex+=1;
    

    //viewer.spin();
    viewer1.spin();
    
    

    }
}
