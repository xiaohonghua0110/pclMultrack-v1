#include "until.hpp"

bool compareNat(const std::string& a, const std::string& b)
{
    if (a.empty())
        return true;
    if (b.empty())
        return false;
    if (std::isdigit(a[0]) && !std::isdigit(b[0]))
        return true;
    if (!std::isdigit(a[0]) && std::isdigit(b[0]))
        return false;
    if (!std::isdigit(a[0]) && !std::isdigit(b[0]))
    {
        if (std::toupper(a[0]) == std::toupper(b[0]))
            return compareNat(a.substr(1), b.substr(1));
        return (std::toupper(a[0]) < std::toupper(b[0]));
    }

    // Both strings begin with digit --> parse both numbers
    std::istringstream issa(a);
    std::istringstream issb(b);
    int ia, ib;
    issa >> ia;
    issb >> ib;
    if (ia != ib)
        return ia < ib;

    // Numbers are the same --> remove numbers and recurse
    std::string anew, bnew;
    std::getline(issa, anew);
    std::getline(issb, bnew);
    return (compareNat(anew, bnew));
}

vector <string> getFiles(string cate_dir)
{
	vector <string> files;

	DIR *dir;
	struct dirent *ptr;

	if ((dir=opendir(cate_dir.c_str())) == NULL)
        {
		perror("Open dir error...");
                exit(1);
        } 
	while ((ptr=readdir(dir)) != NULL)
	{
		
			files.push_back(ptr->d_name);
		
	
	}
	closedir(dir);    
        std::sort(files.begin(), files.end(), compareNat);
        //std::copy(files.begin(), files.end(),std::ostream_iterator<std::string>(std::cout, "\n"));
	
        return files;
        
}
vector<float>  computeVFHistogram(pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud){

    vector<float> his(308);

 pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
 
  ne.setRadiusSearch (0.06);


  ne.compute (*cloud_normals);

  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (cloud);
  vfh.setInputNormals (cloud_normals);
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ> ());
  vfh.setSearchMethod (tree1);
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
  vfh.compute (*vfhs);


  for (int i = 0; i < 308; i++) 
  { 
    his[i] = vfhs->points[0].histogram[i]; 
  }
  return his;

}
