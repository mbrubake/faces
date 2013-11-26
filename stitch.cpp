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

void recomputeNormals(PointCloud<PointNT>::Ptr cloud)
{
		NormalEstimation<PointNT, Normal> norm_est;
		PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
		norm_est.setKSearch(30);
		norm_est.setInputCloud(cloud);
		norm_est.compute(*normals);
		for(int i = 0; i < cloud->points.size(); i++)
		{
				if(cloud->points[i].normal_x * normals->points[i].normal_x + cloud->points[i].normal_y * normals->points[i].normal_y + cloud->points[i].normal_z * normals->points[i].normal_z < 0)
				{
						normals->points[i].normal_x *= -1;
						normals->points[i].normal_y *= -1;
						normals->points[i].normal_z *= -1;
				}
				cloud->points[i].normal_x = normals->points[i].normal_x;
				cloud->points[i].normal_y = normals->points[i].normal_y;
				cloud->points[i].normal_z = normals->points[i].normal_z;
		}
}


Eigen::Matrix3f submatrix(Eigen::Matrix4f affine)
{
		Eigen::Matrix3f m;
		m<<affine(0,0),affine(0,1),affine(0,2)
				,affine(1,0),affine(1,1),affine(1,2)
				,affine(2,0),affine(2,1),affine(2,2);
		return m;
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
		Eigen::Matrix3f lastguess = submatrix(guess).inverse();
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
				Eigen::Matrix3f delta = submatrix(guess)*lastguess;
				float angle= acos((delta.trace()-1)/2);
				cout << angle <<endl;
				if(abs(angle) > 0.25)
				{
						lastguess = submatrix(guess).inverse();
				}
				transformPointCloudWithNormals(*cloud2, *cloud3, guess);
				*cloud1 += *cloud3;
		}
		recomputeNormals(cloud1);
		PointCloud<PointT>::Ptr xyz(new PointCloud<PointT>);
		copyPointCloud(*cloud1, *xyz);
		visualization::CloudViewer viewer("Test");
		viewer.showCloud(xyz, "cloud");
		while(!viewer.wasStopped()){}

		//io::savePCDFileBinary(folder + "merged.pcd", *merged);
		io::savePCDFileBinary(folder + "mergednormal.pcd", *cloud1);
}
