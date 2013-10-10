#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/min_cut_segmentation.h>

using namespace std;
using namespace pcl;
int main (int argc, char** argv)
{

		for(int i = 0; i < atoi(argv[1]); i++)
		{
				pcl::PointCloud <pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
				std::stringstream filename;
				filename << "/home/jonathan/scans/" << i << ".pcd";
				if ( pcl::io::loadPCDFile <pcl::PointXYZ> (filename.str(), *cloud) == -1 )
				{
						std::cout << "Cloud reading failed." << std::endl;
						return (-1);
				}

				/*  pcl::IndicesPtr indices (new std::vector <int>);
					pcl::PassThrough<pcl::PointXYZ> pass;
					pass.setInputCloud (cloud);
					pass.setFilterFieldName ("z");
					pass.setFilterLimits (0.0, 1.0);
					pass.filter (*indices);
					*/
				pcl::MinCutSegmentation<pcl::PointXYZ> seg;
				seg.setInputCloud (cloud);
				//  seg.setIndices (indices);

				pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
				pcl::PointXYZ point;
				point.x = 0;
				point.y = 0;
				point.z = 5;
				foreground_points->points.push_back(point);
				seg.setForegroundPoints (foreground_points);

				seg.setSigma (atof(argv[2]));
				seg.setRadius (atof(argv[3]));
				seg.setNumberOfNeighbours (atof(argv[4]));
				seg.setSourceWeight (atof(argv[5]));

				vector<PointIndices> clusters;
				seg.extract (clusters);

				/*std::vector <int> newindices;
				  newindices = clusters[1].indices;
				  cout << newindices.size() << endl;
				  for(int j = 0; j < newindices.size(); j++)
				  {
				  cloud->points[j] = cloud->points[newindices[j]];
				  }
				  cloud->points.resize(newindices.size());
				  cloud->width = newindices.size();
				  */

				const boost::shared_ptr<vector<int> > indices(new vector<int> (clusters[0].indices));
				ExtractIndices<PointXYZ> pass;
				pass.setInputCloud(cloud);
				pass.setIndices(indices);
				pass.setNegative(true);
				pass.filter(*cloud);

				//  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
				stringstream sfilename;
				sfilename << "/home/jonathan/scans/s" << i << ".pcd";
				io::savePCDFileBinary(sfilename.str(), *cloud);
				cout << "saved cloud of " << cloud->points.size() << " to " << sfilename.str() << endl;

		}
		return (0);
}
