#include <iostream>
#include <boost/make_shared.hpp>
#include <boost/lexical_cast.hpp>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/lum.h>
#include <pcl/registration/correspondence_estimation.h>

#include <pcl/filters/filter.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Core>

#include <math.h>
#include <stdio.h>
#include <ctype.h>
#include <unistd.h>


using namespace pcl;
using namespace std;

typedef PointXYZRGB PointT;
typedef PointSurfel PointNT;

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

void savePCDFiles(vector<PointCloud<PointNT>::Ptr> clouds, string output)
{
  for(int i=0; i<clouds.size(); i++)
  {
    io::savePCDFileBinary(output + boost::lexical_cast<std::string>(i) + ".pcd", *(clouds[i]));
  }
}

void loadPCDFiles(vector<PointCloud<PointNT>::Ptr> &scans, string input, int start,int end)
{
  scans.resize(end-start+1);
  #pragma omp parallel for
  for(int i=start; i<=end; i++)
  {
    PointCloud<PointNT>::Ptr scan(new PointCloud<PointNT>);
    io::loadPCDFile<PointNT>(input + boost::lexical_cast<std::string>(i) + ".pcd", *scan);
    scans[i-start] = scan;
  }
}

void computeNormals(PointCloud<PointNT>::Ptr cloud, float normal_radius)
{
  NormalEstimation<PointNT, PointNT> norm_est;
  search::KdTree<PointNT>::Ptr tree (new search::KdTree<PointNT> ());
  norm_est.setSearchMethod(tree);
  norm_est.setRadiusSearch(normal_radius);
  norm_est.setInputCloud(cloud);
  norm_est.compute(*cloud);
  for(int i = 0; i < cloud->points.size(); i++)
  {
    
    float d= (cloud->points[i].x*cloud->points[i].x)+(cloud->points[i].y*cloud->points[i].y)+(cloud->points[i].z*cloud->points[i].z);
    cloud->points[i].confidence = (cloud->points[i].x*cloud->points[i].x)+(cloud->points[i].y*cloud->points[i].y)+(cloud->points[i].z*cloud->points[i].z);
  }
}

void recomputeNormals(PointCloud<PointNT>::Ptr cloud, float normal_radius)
{
  NormalEstimation<PointNT, Normal> norm_est;
  search::KdTree<PointNT>::Ptr tree (new search::KdTree<PointNT> ());
  norm_est.setSearchMethod(tree);
  PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
  norm_est.setRadiusSearch(normal_radius);
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
  map<string,parameter> params = loadConfig("stitch.config");

  string input = params["input"].s;
  string output = params["output"].s;
  int start = params["start"].i;
  int end = params["end"].i;

  double dist = 2.5;
  int closeFrameDiff = 2;
  int farMinFrameDiff = 30;
  int farMaxFrameDiff = 60;
  int globalLUMIter = 3;
  int localLUMIter = 2;
  int maxit = params["maxit"].i;
  float maxDist = params["maxDist"].f;
  int compute_normals = params["compute_normals"].i;
  float min_angle = params["min_angle"].f;//default 0.25
  float normal_radius1 = params["normal_radius1"].f;
  float normal_radius2 = params["normal_radius2"].f;

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

  PointCloud<PointNT>::Ptr combined(new PointCloud<PointNT>);

  vector<PointCloud<PointNT>::Ptr> clouds;
  loadPCDFiles(clouds, input, start, end);
  if(compute_normals>0)
  {
    #pragma omp parallel for
    for(int it = 0; it < clouds.size(); it++)
      computeNormals(clouds[it], normal_radius1);
  }

  IterativeClosestPointWithNormals<PointNT, PointNT> icp;
  icp.setTransformationEpsilon(1e-7);
  icp.setMaxCorrespondenceDistance(maxDist);
  icp.setMaximumIterations(maxit);

  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f accum = Eigen::Matrix4f::Identity();
  PointCloud<PointNT>::Ptr curr_refcloud = clouds[0];
  for(int it = 1; it < clouds.size(); it++) {
    PointCloud<PointNT>::Ptr cloud2 = clouds[it];

    PointCloud<PointNT> transformed;
    bool converged = true;

    icp.setInputSource(cloud2);
    icp.setInputTarget(curr_refcloud);

    icp.align(transformed, guess);


    if (icp.hasConverged()) {
      cout << "Frame " << it << " Score: " << icp.getFitnessScore() << endl;
      guess = icp.getFinalTransformation();
      accum = accum*guess;
      curr_refcloud = cloud2;

      PointCloud<PointNT>::Ptr cloud3(new PointCloud<PointNT>);
      Eigen::Matrix3f delta = submatrix(guess);
      float angle = acos((delta.trace()-1)/2);
      cout << "Angle: " << abs(angle) <<endl;
      transformPointCloudWithNormals(*cloud2, *cloud3, accum);

      clouds[it]=cloud3;
    }
    else {
      guess = guess*guess;
    }
  }

  pcl::registration::LUM<PointNT> lum;
  lum.setMaxIterations (localLUMIter);
  lum.setConvergenceThreshold (0.001f);
  for(int i = 0; i < clouds.size(); i++)
    lum.addPointCloud(clouds[i]);

  for (int it = 0; it < globalLUMIter; it++) {
    size_t i, j;
#pragma omp parallel for collapse(2)
    for (i = 1; i < clouds.size (); i++) {
      for (j = 0; j < clouds.size (); j++) {
        if (j >= i) continue;
        Eigen::Vector4f ci, cj;
        pcl::compute3DCentroid (*(clouds[i]), ci);
        pcl::compute3DCentroid (*(clouds[j]), cj);
        Eigen::Vector4f diff = ci - cj;

        //std::cout << i << " " << j << " " << diff.norm () << std::endl;

        if (diff.norm () < 5.0 && (i - j <= closeFrameDiff || (i - j > farMinFrameDiff && i - j < farMaxFrameDiff))) {
          std::cout << "LUMIt " << it << ": adding connection between " << i << " and " << j << std::endl;
          pcl::registration::CorrespondenceEstimation<PointNT, PointNT> ce;
          ce.setInputTarget (clouds[i]);
          ce.setInputSource (clouds[j]);
          pcl::CorrespondencesPtr corr (new pcl::Correspondences);
          ce.determineCorrespondences (*corr, dist);
          if (corr->size () > 2) {
#pragma omp critical
            lum.setCorrespondences (j, i, corr);
          }
        }
      }
    }
#pragma omp barrier
    lum.compute ();

    for(int i = 0; i < lum.getNumVertices (); i++) {
      std::cout << i << ": " << lum.getTransformation (i) (0, 3) << " " << lum.getTransformation (i) (1, 3) << " " << lum.getTransformation (i) (2, 3) << std::endl;
      clouds[i] = lum.getTransformedCloud (i);
    }
  }
  cout << "Saving clouds to " << output << endl;
  savePCDFiles(clouds, output);
  io::savePCDFileBinary(output + "merged.pcd", *(lum.getConcatenatedCloud()));
}
