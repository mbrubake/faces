#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/vtk_lib_io.h>
using namespace pcl;
using namespace std;
int main (int argc, char** argv)
{
		// Load input file into a PointCloud<T> with an appropriate type
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCLPointCloud2 cloud_blob;
		pcl::io::loadPCDFile (argv[1], cloud_blob);
		pcl::fromPCLPointCloud2 (cloud_blob, *cloud_raw);

		int poissonDepth = atoi(argv[2]);
		float mlsSearchRadius = atof(argv[3]);
		const float leaf_size = atof(argv[4]);

		if(leaf_size > 0)
		{
				VoxelGrid<PointXYZ> grid;
				grid.setLeafSize(leaf_size,leaf_size,leaf_size);
				grid.setInputCloud(cloud_raw);
				grid.filter(*cloud_raw);
		}
		PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
		if(mlsSearchRadius > 0)
		{
				cout << "MLS..." << endl;
				//* the data should be available in cloud
				MovingLeastSquares<PointXYZ, PointXYZ> mls;
				pcl::search::KdTree<PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
				mls.setInputCloud(cloud_raw);
				mls.setSearchRadius(mlsSearchRadius);
				mls.setPolynomialFit(true);
				mls.setPolynomialOrder(2);
				mls.setSearchMethod(tree1);
				mls.process(*cloud);
		}
		else
		{
				cloud = cloud_raw;
		}
		vector<int> indices;
		removeNaNFromPointCloud(*cloud, *cloud, indices);

		cout << "Normal estimation..." << endl;
		// Normal estimation*
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud (cloud);
		n.setInputCloud (cloud);
		n.setSearchMethod (tree);
		n.setKSearch(40);
		n.setViewPoint()
		n.compute (*normals);
		//* normals should not contain the point normals + surface curvatures

		// Concatenate the XYZ and normal fields*
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
		//* cloud_with_normals = cloud + normals

		// Create search tree*
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud (cloud_with_normals);

		cout << "Poisson reconstruction..." << endl;
		// Initialize objects
		pcl::PolygonMesh triangles;
		pcl::Poisson<pcl::PointNormal> poisson;
		poisson.setDepth(poissonDepth);
		poisson.setInputCloud(cloud_with_normals);
		poisson.setSearchMethod(tree2);
		poisson.performReconstruction(triangles);
		pcl::io::savePolygonFileSTL("mesh.stl",triangles);
		cout << "saved mesh.stl" << endl;
		return (0);
}
