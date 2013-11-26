#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

// Types
typedef pcl::PointXYZRGBNormal PointNT;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

using namespace pcl;

// Align a rigid object to a scene with clutter and occlusions
		int
main (int argc, char **argv)
{
		// Point clouds
		PointCloudT::Ptr object (new PointCloudT);
		PointCloudT::Ptr object_orig(new PointCloudT);
		PointCloudT::Ptr object_aligned (new PointCloudT);
		PointCloudT::Ptr scene (new PointCloudT);
		PointCloudT::Ptr scene_orig(new PointCloudT);
		FeatureCloudT::Ptr object_features (new FeatureCloudT);
		FeatureCloudT::Ptr scene_features (new FeatureCloudT);

		// Load object and scene
		pcl::console::print_highlight ("Loading point clouds...\n");
		if (pcl::io::loadPCDFile<PointNT> (argv[1], *scene_orig) < 0 ||
						pcl::io::loadPCDFile<PointNT> (argv[2], *object_orig) < 0)
		{
				pcl::console::print_error ("Error loading object/scene file!\n");
				return (1);
		}

		float leaf_size = atof(argv[3]);//0.005
		int num_samples = atoi(argv[4]);//3
		int corr_randomness = atoi(argv[5]);//2
		float similarity_threshold = atof(argv[6]);//0.6
		float inlier_threshold = atof(argv[7]);//1.5
		float inlier_fraction = atof(argv[8]);//0.25
		float normal_radius = atof(argv[9]);//0.01
		float feature_radius = atof(argv[10]);//0.025

		// Downsample
		pcl::console::print_highlight ("Downsampling...\n");
		pcl::VoxelGrid<PointNT> grid;
		const float leaf = leaf_size;
		grid.setLeafSize (leaf, leaf, leaf);
		grid.setInputCloud (object_orig);
		grid.filter (*object);
		grid.setInputCloud (scene_orig);
		grid.filter (*scene);

		// Estimate normals for scene
		pcl::console::print_highlight ("Estimating scene normals...\n");
		pcl::NormalEstimationOMP<PointNT,PointNT> nest;
		nest.setRadiusSearch (normal_radius);
		nest.setInputCloud (scene);
		nest.setSearchSurface(scene_orig);
		nest.compute (*scene);
		nest.setInputCloud(scene_orig);
		nest.compute(*scene_orig);
		nest.setInputCloud(object);
		nest.setSearchSurface(object_orig);
		nest.compute(*object);
		nest.setInputCloud(object_orig);
		nest.compute(*object_orig);

		// Estimate features
		pcl::console::print_highlight ("Estimating features...\n");
		FeatureEstimationT fest;
		fest.setRadiusSearch (feature_radius);
		fest.setInputCloud (object);
		fest.setInputNormals (object);
		fest.compute (*object_features);
		fest.setInputCloud (scene);
		fest.setInputNormals (scene);
		fest.compute (*scene_features);

		// Perform alignment
		pcl::console::print_highlight ("Starting alignment...\n");
		pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
		align.setInputSource (object);
		align.setSourceFeatures (object_features);
		align.setInputTarget (scene);
		align.setTargetFeatures (scene_features);
		align.setNumberOfSamples (num_samples); // Number of points to sample for generating/prerejecting a pose
		align.setCorrespondenceRandomness (corr_randomness); // Number of nearest features to use
		align.setSimilarityThreshold (similarity_threshold); // Polygonal edge length similarity threshold
		align.setMaxCorrespondenceDistance (inlier_threshold * leaf); // Set inlier threshold
		align.setInlierFraction (inlier_fraction); // Set required inlier fraction
		align.align (*object_aligned);

		if (align.hasConverged ())
		{
				cout << "Converged" << endl;
		}
		// Print results
		Eigen::Matrix4f transformation = align.getFinalTransformation ();
		pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
		pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
		pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
		pcl::console::print_info ("\n");
		pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
		pcl::console::print_info ("\n");
		pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

		pcl::PointCloud<PointT>::Ptr scenexyz(new PointCloud<PointT>);
		copyPointCloud(*scene, *scenexyz);
		cout << scenexyz->points.size() << endl;
		PointCloud<PointT>::Ptr alignedxyz(new PointCloud<PointT>);
		copyPointCloud(*object_aligned, *alignedxyz);
		*scenexyz += *alignedxyz;
		visualization::CloudViewer viewer("Test");
		viewer.showCloud(scenexyz);
		while(!viewer.wasStopped()){}
		PointCloud<PointT>::Ptr object_fullxyz (new PointCloud<PointT>);
		PointCloud<PointT>::Ptr scene_fullxyz (new PointCloud<PointT>);
		copyPointCloud(*object_orig, *object_fullxyz);
		copyPointCloud(*scene_orig, *scene_fullxyz);
		transformPointCloud(*object_fullxyz, *object_fullxyz,transformation);
		transformPointCloudWithNormals(*object_orig, *object_orig, transformation);
		*scene_orig += *object_orig;
		//io::savePCDFileBinary("0align.pcd", *scene_fullxyz);
		//io::savePCDFileBinary("1align.pcd", *object_fullxyz);
		io::savePCDFileBinary("sac1.pcd", *object_orig);
		return (0);
}
