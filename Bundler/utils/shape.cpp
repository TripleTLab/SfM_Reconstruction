#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using namespace pcl;

int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZRGB >::Ptr cloud_before (new pcl::PointCloud<pcl::PointXYZRGB >);
	pcl::PointCloud<pcl::PointXYZRGB >::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB >);
  	
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(argv[1], *cloud_before) == -1){
        	cout<<"Couldn't read file test_pcd.pcd \n";
        	return (-1);
    	}

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    	sor.setInputCloud(cloud_before);
    	sor.setMeanK(50);//50个临近点
    	sor.setStddevMulThresh(1.0);//距离大于1倍标准方差
    	sor.filter(*cloud);
	

	/*----------------------------------*/
    	

	//pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    	//tree2->setInputCloud(cloud_with_normals);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal > n;//法线估计对象  
    	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal >);//存储估计的法线的指针  
    	pcl::search::KdTree<pcl::PointXYZRGB >::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB >);  
    	tree->setInputCloud(cloud);  
   	n.setInputCloud(cloud);  
    	n.setSearchMethod(tree);  
    	n.setKSearch(20);  
    	n.compute(*normals); //计算法线，结果存储在normals中 

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);  
    	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	
	//创建搜索树  
    	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);  
    	tree2->setInputCloud(cloud_with_normals);  

	pcl::Poisson<pcl::PointXYZRGBNormal> pn;
    	pn.setConfidence(false);
    	pn.setDegree(2);
    	pn.setDepth(8);
    	pn.setIsoDivide(8);
    	pn.setManifold(false);
    	pn.setOutputPolygons(false);
    	pn.setSamplesPerNode(3.0);
    	pn.setScale(1.25);
    	pn.setSolverDivide(8);
    	pn.setSearchMethod(tree2);
    	pn.setInputCloud(cloud_with_normals);
    	pcl::PolygonMesh mesh;
    	pn.performReconstruction(mesh);
	

//RGB
	PointCloud<PointXYZRGB> cloud_color_mesh; 
      	pcl::fromPCLPointCloud2(mesh.cloud, cloud_color_mesh); 

     	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
      	kdtree.setInputCloud (cloud);
	int K = 5;
      	std::vector<int> pointIdxNKNSearch(K);
      	std::vector<float> pointNKNSquaredDistance(K);

	for(int i=0;i<cloud_color_mesh.points.size();++i){
         uint8_t r = 0;
         uint8_t g = 0;
         uint8_t b = 0;
         float dist = 0.0; 
         int red = 0;
         int green = 0;
         int blue = 0;
         uint32_t rgb;

         if ( kdtree.nearestKSearch (cloud_color_mesh.points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
         {
            for (int j = 0; j < pointIdxNKNSearch.size (); ++j) 
            { 

               r = cloud->points[ pointIdxNKNSearch[j] ].r;
               g = cloud->points[ pointIdxNKNSearch[j] ].g;
               b = cloud->points[ pointIdxNKNSearch[j] ].b;

               red += int(r);
               green += int(g);
               blue += int(b);
               dist += 1.0/pointNKNSquaredDistance[j]; 

            }  
         }

         cloud_color_mesh.points[i].r = int(red/pointIdxNKNSearch.size ()+0.5); 
         cloud_color_mesh.points[i].g = int(green/pointIdxNKNSearch.size ()+0.5); 
         cloud_color_mesh.points[i].b = int(blue/pointIdxNKNSearch.size ()+0.5);


      }

	toPCLPointCloud2(cloud_color_mesh, mesh.cloud);

	io::savePLYFile(argv[2],mesh);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  	viewer->setBackgroundColor(0, 0, 0);

    	//viewer->addPointCloud(cloud,"c");
	viewer->addPolygonMesh(mesh, "triangles");

    	//viewer->addCoordinateSystem(1.0);
    	viewer->initCameraParameters();
	viewer->setCameraPosition(0.1, 0.1, 3, -2,-0.5,0);
    	while (!viewer->wasStopped()){
	        viewer->spinOnce(100);
	        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	return (0);
}
