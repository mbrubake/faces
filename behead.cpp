#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/min_cut_segmentation.h>

using namespace std;
using namespace pcl;

typedef pcl::PointXYZRGB PointT;

int main (int argc, char** argv)
{
		string folder = argv[1];
		int start = atoi(argv[2]);
		int end = atoi(argv[3]);
		float center = atof(argv[4]);
		float r2 = atof(argv[5]);
		for(int i = start; i <= end ; i++)
		{
				pcl::PointCloud <PointT>::Ptr cloud (new pcl::PointCloud <PointT>);
				std::stringstream filename;
				filename << folder << i << ".pcd";
				if (pcl::io::loadPCDFile <PointT> (filename.str(), *cloud) == -1 )
				{
						std::cout << "Cloud reading failed." << std::endl;
						return (-1);
				}
				int pointcount = 0;
				for(int i = 0; i < cloud->width; i++)
				{
						if(cloud->points[i].y*cloud->points[i].y+cloud->points[i].x*cloud->points[i].x+ (cloud->points[i].z-center)*(cloud->points[i].z-center) < r2)
						{
								cloud->points[pointcount] = cloud->points[i];
								pointcount++;
						}
				}
				cloud->points.resize(pointcount);
				cloud->width = cloud->points.size();

				stringstream sfilename;
				sfilename << folder << "seg" << i << ".pcd";
				io::savePCDFileBinary(sfilename.str(), *cloud);
				cout << "saved cloud of " << cloud->points.size() << " to " << sfilename.str() << endl;

		}
		return (0);
}
