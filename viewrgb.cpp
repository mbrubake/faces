#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace pcl;
using namespace std;

int main(int argc, char** argv)
{
        PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
		stringstream filename;
		filename << argv[1];
        io::loadPCDFile<PointXYZRGB>(filename.str(), *cloud);
		visualization::CloudViewer viewer("Test");
		viewer.showCloud(cloud->makeShared());
		while(!viewer.wasStopped()){}
}
