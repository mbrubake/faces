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

	for(int i = start; i <= end ; i++)
	{
		pcl::PointCloud <PointT>::Ptr cloud (new pcl::PointCloud <PointT>);
		std::stringstream filename;
		filename << input << i << ".pcd";
		if(pcl::io::loadPCDFile <PointT> (filename.str(), *cloud) == -1)
		{
			continue;
		}

		int pointcount = 0;
		for(int i = 0; i < cloud->width; i++)
		{
			if(pow(cloud->points[i].y-cy,2)+pow(cloud->points[i].x-cx,2)+ pow(cloud->points[i].z-cz,2) < r2)
			{
				cloud->points[pointcount] = cloud->points[i];
				pointcount++;
			}
		}
		cloud->points.resize(pointcount);
		cloud->width = cloud->points.size();

		stringstream sfilename;
		sfilename << output << i << ".pcd";
		io::savePCDFileBinary(sfilename.str(), *cloud);
		cout << "saved cloud of " << cloud->points.size() << " to " << sfilename.str() << endl;
	}
	return (0);
}
