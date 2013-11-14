#include <iostream>
#include <boost/make_shared.hpp>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/filters/filter.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Core>

#include <math.h>

using namespace pcl;
using namespace std;

typedef PointXYZRGB PointT;
typedef PointXYZRGBNormal PointNT;

void view(PointCloud<PointT>::Ptr cloud)
{
		visualization::CloudViewer viewer("Test");
		viewer.showCloud(cloud->makeShared());
		while(!viewer.wasStopped()){}
}

void loadCloud(PointCloud<PointNT>::Ptr cloud, string folder, int num)
{
		stringstream filename;
		filename << folder << num << ".pcd";
		io::loadPCDFile<PointNT>(filename.str(), *cloud);
		cout << "loaded " << filename.str() << endl;
}

void computeNormals(PointCloud<PointNT>::Ptr cloud)
{
		NormalEstimation<PointNT, PointNT> norm_est;
		norm_est.setKSearch(30);
		norm_est.setInputCloud(cloud);
		norm_est.compute(*cloud);
}

int main(int argc, char** argv)
{
		string folder = argv[1];
		int start = atoi(argv[2]);
		int end = atoi(argv[3]);
		int maxit = atoi(argv[4]);
		float maxDist = atof(argv[5]);
		int compute_normals = atoi(argv[6]);
		PointCloud<PointNT>::Ptr cloud1(new PointCloud<PointNT>);
		loadCloud(cloud1, folder, start);
		if(compute_normals>0)
		{
				computeNormals(cloud1);
		}
		Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();

		for(int it = start+1; it <= end; it++)
		{
				PointCloud<PointNT>::Ptr cloud2(new PointCloud<PointNT>);
				loadCloud(cloud2, folder, it);
				if(compute_normals>0)
				{
						computeNormals(cloud2);
				}

				IterativeClosestPointWithNormals<PointNT, PointNT> icp;
				icp.setTransformationEpsilon(1e-7);
				icp.setMaxCorrespondenceDistance(maxDist);
				icp.setInputCloud(cloud2);
				icp.setInputTarget(cloud1);


				PointCloud<PointNT> transformed;
				icp.setMaximumIterations(maxit);
				icp.align(transformed, guess);
				guess = icp.getFinalTransformation();
				cout << "frame: " << it << " score: " << icp.getFitnessScore() << endl;
				PointCloud<PointNT>:: Ptr cloud3(new PointCloud<PointNT>);
				transformPointCloud(*cloud2, *cloud3, guess);
				*cloud1 += *cloud3;
		}
		PointCloud<PointT>::Ptr merged(new PointCloud<PointT>);
		copyPointCloud(*cloud1, *merged);
		visualization::CloudViewer viewer("Test");
		viewer.showCloud(merged, "cloud");
		while(!viewer.wasStopped()){}

		io::savePCDFileBinary(folder + "merged.pcd", *merged);
		io::savePCDFileBinary(folder + "mergednormal.pcd", *cloud1);
}
