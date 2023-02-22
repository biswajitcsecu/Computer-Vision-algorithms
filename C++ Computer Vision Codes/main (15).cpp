 
#include <cstdlib>
 #include <iostream>
 #include <pcl/point_types.h>
 #include <pcl/filters/passthrough.h>
#include <iostream>
#include <CGAL/Simple_cartesian.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Segment_2 Segment_2;



using namespace std;

 
int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 25;
  cloud->height = 10;
  cloud->points.resize (cloud->width * cloud->height);

  for (auto& point: *cloud)
  {
      point.x = 1024 * rand () / (RAND_MAX + 1.0f);
      point.y = 1024 * rand () / (RAND_MAX + 1.0f);
      point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before filtering: " << std::endl;
  for (const auto& point: *cloud)
    std::cerr << " " << point.x << " "<< point.y << " "<< point.z << std::endl;

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  
  for (const auto& point: *cloud_filtered)
      std::cerr << " " << point.x << " "<< point.y << " "<< point.z << std::endl;
  
  
  Point_2 p(1,1), q(10,10);
  std::cout << "p = " << p << std::endl;
  std::cout << "q = " << q.x() << " " << q.y() << std::endl;
  std::cout << "sqdist(p,q) = "
            << CGAL::squared_distance(p,q) << std::endl;
  Segment_2 s(p,q);
  Point_2 m(5, 9);
  std::cout << "m = " << m << std::endl;
  std::cout << "sqdist(Segment_2(p,q), m) = "
            << CGAL::squared_distance(s,m) << std::endl;
  std::cout << "p, q, and m ";
  switch (CGAL::orientation(p,q,m)){
      case CGAL::COLLINEAR:
          std::cout << "are collinear\n";
          break;
      case CGAL::LEFT_TURN:
          std::cout << "make a left turn\n";
          break;
      case CGAL::RIGHT_TURN:
          std::cout << "make a right turn\n";
          break;
  }
  std::cout << " midpoint(p,q) = " << CGAL::midpoint(p,q) << std::endl;  

  return (0);
}

