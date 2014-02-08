#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <math.h>

#include <stdio.h>
#include <ctype.h>
#include <unistd.h>

using namespace std;
using namespace pcl;

typedef pcl::PointXYZRGB PointT;

struct parameter
{
	int i;
	float f;
	string s;
};

map<string, parameter> loadConfig(const char* filename)
{
	ifstream f;
	f.open(filename);
	string t;
	string param;
	string val;
	map<string, parameter> config;
	while(getline(f,t,' '))
	{
		getline(f,param,'=');
		getline(f,val);
		boost::algorithm::trim(param);
		parameter x;
		if(t=="int")
		{
			x.i= atoi(val.c_str());
		}
		else if(t == "string")
		{
			x.s=val;
		}
		else if(t=="float")
		{
			x.f=atof(val.c_str());
		}
		config[param]=x;
	}
	return config;
}

void compute3DCentroid (PointCloud<PointT>::Ptr cloud, const std::vector<int> &indices, Eigen::Vector4f &c)
{
	size_t N = indices.size();
	c.setZero();
	for(size_t i=0; i < N; i++) {
		c[0] += cloud->points[indices[i]].x/N;
		c[1] += cloud->points[indices[i]].y/N;
		c[2] += cloud->points[indices[i]].z/N;
	}
}

void savePCDFiles(vector<PointCloud<PointT>::Ptr> clouds, string output)
{
	for(int i=0; i<clouds.size(); i++)
	{
		io::savePCDFileBinary(output + boost::to_string(i) + ".pcd", *(clouds[i]));
	}
}

vector<PointCloud<PointT>::Ptr> loadPCDFiles(string input, int start,int end)
{
	vector<PointCloud<PointT>::Ptr> scans;
	for(int i=start; i<=end; i++)
	{
		PointCloud<PointT>::Ptr scan(new PointCloud<PointT>);
		io::loadPCDFile<PointT>(input + boost::to_string(i) + ".pcd", *scan);
		scans.push_back(scan);
	}
	return scans;
}
bool
enforceIntensitySimilarity (const PointT& point_a, const PointT& point_b, float squared_distance)
{
	float dr = point_a.r - point_b.r;
	float dg = point_a.g - point_b.g;
	float db = point_a.b - point_b.b;
        float dist = dr*dr + dg*dg + db*db;
	if (dist < 250*250)
		return (true);
	else
		return (false);
}

int main (int argc, char** argv)
{
	map<string,parameter> params = loadConfig("behead.config");

	string input = params["input"].s;
	string output = params["output"].s;
	int start = params["start"].i;
	int end = params["end"].i;
	float cx = params["cx"].f;
	float cy = params["cy"].f;
	float cz = params["cz"].f;
	float r2 = params["r2"].f;

	int c;

	while((c = getopt (argc, argv, "i:o:n:")) !=-1)
	{
		switch(c)
		{
			case 'i':
				input = optarg;
				break;
			case 'o':
				output = optarg;
				break;
			case 'n':
				start = 0;
				end = atoi(optarg)-1;
				break;
		}
	}

#pragma omp parallel for
	for(int i = start; i <= end ; i++)
	{
		pcl::PointCloud <PointT>::Ptr cloud (new pcl::PointCloud <PointT>);
		std::stringstream filename;
		filename << input << i << ".pcd";
		if(pcl::io::loadPCDFile <PointT> (filename.str(), *cloud) == -1)
		{
			continue;
		}


	        pcl::ConditionalEuclideanClustering<PointT> cec(true);
		pcl::IndicesClusters clusters;

		cec.setConditionFunction (&enforceIntensitySimilarity);
		// Points within this distance from one another are going to need
		// to validate the enforceIntensitySimilarity function to be part
		// of the same cluster:
		cec.setClusterTolerance (0.09f);
		// Size constraints for the clusters:
		cec.setMinClusterSize (1000);
		cec.setInputCloud(cloud);
		cec.segment (clusters);

		pcl::PointCloud <PointT>::Ptr cloud_filtered (new pcl::PointCloud <PointT>);
		float cluster_dist = 0;
		size_t max_cluster_size = 0;
		int max_cluster = -1;
		for (int clust_i = 0; clust_i < clusters.size(); clust_i++) {
		        Eigen::Vector4f ci;
		        compute3DCentroid (cloud, clusters[clust_i].indices, ci);

			float dx = cx - ci[0];
			float dy = cy - ci[1];
			float dz = cz - ci[2];
			float cdist = dx*dx + dy*dy + dz*dz;

//			if (cdist < r2 && (max_cluster < 0 || cdist < cluster_dist)) {
//				cluster_dist = cdist;
			if (cdist < r2 && clusters[clust_i].indices.size() > max_cluster_size) {
				max_cluster_size = clusters[clust_i].indices.size();
				max_cluster = clust_i;
			}
		}
		assert(max_cluster >= 0);
		cloud_filtered->points.resize(clusters[max_cluster].indices.size());
		cloud_filtered->width = clusters[max_cluster].indices.size();
		cloud_filtered->height = 1;
		for(int pt_i = 0; pt_i < clusters[max_cluster].indices.size(); pt_i++) {
			cloud_filtered->points[pt_i] = cloud->points[clusters[max_cluster].indices[pt_i]];
		}

		stringstream sfilename;
		sfilename << output << i << ".pcd";
		io::savePCDFileBinary(sfilename.str(), *cloud_filtered);
		cout << "saved cloud of " << cloud_filtered->points.size() << " to " << sfilename.str() << endl;
	}
	return (0);
}
