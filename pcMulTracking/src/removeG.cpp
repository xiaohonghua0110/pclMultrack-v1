#include "removeG.hpp"
#include <unordered_map>
int D,H,W,G_D,G_W,C_D,C_W,C_H;
float G_vD, G_vW,C_vD,C_vW,C_vH;
void init_para(){
     D=40;
     W=40;
     H=6;
     
     G_vD=5; G_vW=5;
     G_D=D/G_vD;  G_W=W/G_vW;

     C_vD=0.25; C_vW=0.25; C_vH=0.25;
     C_D=D/C_vD;  C_W=W/C_vW; C_H=H/C_vH;
}

void int2str(const int &int_temp,string &string_temp)  
{  
        stringstream stream;  
        stream<<int_temp;  
        string_temp=stream.str();   
}  
bool comparePoint(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) { 
        if (p1.y < p2.y) { 
                return true; 
        } 
        else { 
                return false; 
        } 
}

void rmCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &Ncloud,vector<vector<int>> &cluster,vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> &splitPoint){
    
    int a[G_W][G_D][30]={0};
    float result[G_W][G_D]={0};
    

    int ID, IH, IW=0;
    int flag=1;
    vector<CEIL> pcCube(C_D*C_H*C_W);  
    int numCube[10000];int numcount=0;
    
    for (long int i=0;i<cloud->points.size();i++)
    {
        
        int d1=floor(cloud->points[i].z/G_vW)+G_W/2;int d0=floor(cloud->points[i].x/G_vD)+G_D/2;int index=0;
        if(cloud->points[i].y>=-3.0&&cloud->points[i].y<=0&&cloud->points[i].x<=D/2&&cloud->points[i].x>=-D/2&&cloud->points[i].z<=W/2&&cloud->points[i].z>=-W/2)
        {
            index=floor((cloud->points[i].y)/-0.1);                 
            a[d1][d0][index]++;
            
        }
    }

        for (int i=0;i<G_W;i++)
    {
        for(int j=0;j<G_D;j++)
        {
            int max=0;int maxindex=0;
            int gradient[29]={0};
            for(int k=29;k>0;k--)
            {
               gradient[29-k]=a[i][j][k]-a[i][j][k-1];
               if (gradient[29-k]>max)
               {
                   max=gradient[29-k];
                   maxindex=k;
               }
            }
            result[i][j]= -0.1*(float)maxindex+0.12;
        }
    }

    for (long int i=0;i<cloud->points.size();i++)
    {
        int d1=floor(cloud->points[i].z/G_vW)+G_W/2;int d0=floor(cloud->points[i].x/G_vD)+G_D/2;
        if (cloud->points[i].y>result[d1][d0]&&cloud->points[i].x<D/2&&cloud->points[i].x>-D/2&&cloud->points[i].z<W/2&&cloud->points[i].z>-W/2)
        {
            Ncloud->push_back(cloud->points[i]);
            ID=floor((cloud->points[i].x+D/2)/C_vD);    
            IH=floor((cloud->points[i].y+H/2)/C_vH);     
            IW=floor((cloud->points[i].z+W/2)/C_vW);  
            //cout<<"ID"<<ID<<endl;cout<<"IH"<<IH<<endl;cout<<"IW"<<IW<<endl;
            pcCube[ID*C_H*C_W+IH*C_W+IW].num+=1; 
            splitPoint[ID*C_H*C_W+IH*C_W+IW]->points.push_back(cloud->points[i]);
            if (pcCube[ID*C_H*C_W+IH*C_W+IW].num==1)
            {
    //            mapCube[ID*C_H*C_W+IH*C_W+IW]=0; 
                pcCube[ID*C_H*C_W+IH*C_W+IW].flag==0;
                numCube[numcount]=ID*C_H*C_W+IH*C_W+IW;
                numcount+=1;
            }
        }
            
    }
    sort(numCube,numCube+numcount);
    cout<<"numcount"<<numcount<<endl;
  ///////

//    for(unordered_map<int, int>::iterator iter = mapCube.begin(); iter != mapCube.end(); iter++)    
    for (int i=0;i<numcount;i++)
    {    
        //int currentCube=iter->first;  
        int currentCube=numCube[i];
        int neighbor[27]={0};
        int c2=currentCube/(C_H*C_W); 
        int c1=(currentCube%(C_H*C_W))/C_W; 
        int c0=(currentCube%(C_H*C_W))%C_W;
        
        if(c0<C_W-1&&c1<C_H-1&&c2<C_D-1&&c0>0&&c1>0&&c2>0)
        {
            for (int k=0;k<27;k++)
                {               
                    int d2=c2+k/9-1; int d1=c1+(k%9)/3-1;int d0=c0+(k%9)%3-1;                     
                    neighbor[k]=pcCube[d2*C_H*C_W+d1*C_W+d0].flag;                             
                }

            int min=10000;
            for (int k=0;k<27;k++)
            {
                if (neighbor[k]>0&neighbor[k]<min)
                    min=neighbor[k];    
            }

            if(min==10000)
            {
                pcCube[currentCube].flag=flag;
                flag+=1;
            } 
             else
                pcCube[currentCube].flag=min;  
            cluster[pcCube[currentCube].flag].push_back(currentCube); 
            for (int k=0;k<27;k++)
            {    
                if (neighbor[k]>0&neighbor[k]>min)
                {

                    for(int p=0;p<cluster[neighbor[k]].size();p++)
                    {                                   
                        cluster[pcCube[currentCube].flag].push_back(cluster[neighbor[k]][p]); 
                    }
                    cluster[neighbor[k]].clear();
                }
            }
        }
        
    }

 

    
    
   
}