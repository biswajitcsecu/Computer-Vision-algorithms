#define PCL_ALWAYS

// pcl header filesystem----------------
#include<iostream>
#include<filesystem>
#include<vector>
#include<fstream>
#include<utility>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/point_representation.h>
#include<pcl/io/io.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/voxel_grid_label.h>
#include<pcl/pcl_base.h>
#include<pcl/pcl_config.h>
#include<pcl/pcl_exports.h>
#include<pcl/exceptions.h>
#include<pcl/common/common.h>
#include<pcl/common/io.h>
#include<pcl/common/colors.h>
#include<pcl/pcl_macros.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/io/ply/ply.h>
#include<pcl/io/ply/ply_parser.h>
#include<pcl/io/ply/io_operators.h>
#include<pcl/PCLPointField.h>
#include<pcl/filters/passthrough.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/point_cloud_handlers.h>
#include<pcl/visualization/boost.h>
#include <pcl/filters/statistical_outlier_removal.h>

// CGAL header
#include<CGAL/basic.h>
#include<CGAL/basic_classes.h>
#include<CGAL/kernel_basic.h>  
#include<CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include<CGAL/grid_simplify_point_set.h>
#include<CGAL/IO/read_xyz_points.h>
#include<CGAL/property_map.h>
#include<CGAL/Polyhedron_3.h>
#include<CGAL/convex_hull_3.h>
#include <CGAL/Surface_mesh.h>
#include<CGAL/Memory_sizer.h>
#include<CGAL/IO/write_ply_points.h>
#include<CGAL/Scale_space_surface_reconstruction_3.h>
#include<CGAL/IO/read_off_points.h>
#include <CGAL/Timer.h>



using namespace std;
using namespace pcl;
using namespace CGAL;
using namespace boost;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud < pcl::PointXYZ >  pcl_point;


typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polyhedron_3<K>                     Polyhedron3;
typedef K::Point_3                                              Point;
typedef K::Vector_3                                            Vector;
typedef CGAL::Surface_mesh<Point>               Surfacemesh;
 


class DualPlyDisplay{
public:
 DualPlyDisplay(){}
    
 void run(){
     //loadPLYFile --------------
     PointCloudT::Ptr cloud1 (new PointCloudT);
     PointCloudT::Ptr cloud2 (new PointCloudT);
     
     if (pcl::io::loadPLYFile ("/home/picox/Projects/Netbeans/DoubleViewPly/Welsh.ply", *cloud1) < 0)
     {
         PCL_ERROR ("Error loading cloud %s.\n", "");         
     }
     
     //processing points data------------------------
     // down points sampled----------------------------
     VoxelGrid<PointT> passvox;
     passvox.setInputCloud(cloud1);
     passvox.setLeafSize(0.095f, 0.095f, 0.095f);
     passvox.filter(*cloud2);
     
     //extraction point to vector points ---------------------------
     pcl::StatisticalOutlierRemoval <pcl::PointXYZRGB>  outlier_filter;     
     outlier_filter.setInputCloud(cloud2);
     outlier_filter.setMeanK(10);
     outlier_filter.setStddevMulThresh(2.5);
     outlier_filter.filter(*cloud2);
     
     //point formation
     PointCloud<PointXYZ> ::Ptr dest(new PointCloud<PointXYZ>);
     
     //PointXYZRGB to PointXYZ
     pcl::copyPointCloud(*cloud2,*dest);
     
     //declare cgal vector of point
     std::vector<Point> points;	
     pcl_point::iterator start_id = dest->begin(); 
     
     //pick up point pcl to cgal vector object     
     for (; start_id != dest->end(); ++start_id) 
     {
         Point p(start_id->x, start_id->y, start_id->z);
         points.push_back(p);
     }
     
     //-----convex hull ------
     Polyhedron3 poly;
     
     // compute convex hull of non-collinear points
     CGAL::convex_hull_3(points.begin(), points.end(), poly);
     std::cout << "The convex hull contains " << poly.size_of_vertices() << " vertices" << std::endl;

     
     // compute Surface mesh
//     Surfacemesh sm;
//     CGAL::convex_hull_3(points.begin(), points.end(), sm);
//     std::cout << "The convex hull contains " << num_vertices(sm) << " vertices" << std::endl;
//      

     
     
     //visualization----------------------
     pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
     viewer->initCameraParameters ();
     
     int v1(0);
     viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
     viewer->setBackgroundColor (0, 0, 0, v1);
     viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
     pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb1(cloud1);
     viewer->addPointCloud<PointT> (cloud1, rgb1, " cloud1", v1);
     
     int v2(0);
     viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
     viewer->setBackgroundColor (0.0, 0.0, 0.0, v2);
     viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
     pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb2(cloud2);
     viewer->addPointCloud<PointT> (cloud2, rgb2, " cloud2", v2);
     
     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, " cloud1");
     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, " cloud2");
     viewer->addCoordinateSystem (1.0);
     
     viewer->setPosition(10, 10);
     viewer->setSize(1280, 800);
     viewer->setShowFPS(true);
     viewer->setWindowName("Point reduction Filter");
     viewer->resetCameraViewpoint();
     viewer->resetCamera();
     
     while (!viewer->wasStopped())
     {
         viewer->spinOnce(100);
     }  
     viewer->removePolygonMesh("src");
     viewer->close();
     
     
     //save data
     std::ofstream faired_off("out.off");
     faired_off.precision(17);
     faired_off << poly;
     faired_off.close();
 }
 

 
};


int main(int argc, char** argv) {
    DualPlyDisplay *dd = new DualPlyDisplay();
    dd->run();
   delete dd;
    
    return EXIT_SUCCESS;
}

