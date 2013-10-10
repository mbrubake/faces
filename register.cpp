#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Core>

using namespace pcl;
using namespace std;

int main(int argc, char** argv)
{
		PointCloud<PointXYZ>::Ptr cloud1(new PointCloud<PointXYZ>);
		io::loadPCDFile<PointXYZ>("scans/0.pcd", *cloud1);
		Eigen::Matrix4f guess = Eigen::Matrix4f::Identity(); 
		int freq = atoi(argv[3]);
		for(int it = 1; it < atoi(argv[1]); it++)
		{
				PointCloud<PointXYZ>::Ptr cloud2(new PointCloud<PointXYZ>);
				stringstream filename;
				filename << "scans/" << it << ".pcd";
				io::loadPCDFile<PointXYZ>(filename.str(), *cloud2);

				IterativeClosestPoint<PointXYZ, PointXYZ> icp;
				icp.setInputCloud(cloud2);
				icp.setInputTarget(cloud1);

				PointCloud<PointXYZ> transformed;
				int iterations = atoi(argv[2]);
				if(it % freq == 0)
				{
						iterations = 5*iterations;
				}
				icp.setMaximumIterations(iterations);
				icp.align(transformed, guess);
				cout << "frame: " << it << " score: " << icp.getFitnessScore() << endl;
				if(it % freq == 0)
				{
						*cloud1 += transformed;
				}
				guess = icp.getFinalTransformation();
		}
		io::savePCDFileBinary("scans/merged.pcd", *cloud1);
		visualization::CloudViewer viewer("Test");
		viewer.showCloud(cloud1->makeShared());
		while(!viewer.wasStopped()){}
}
