#include "openni2/OpenNI.h"
#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>

using namespace openni;
using namespace std;
using namespace pcl;
int main(int argc, char** argv)
{
		OpenNI::initialize();
		Device* cam = new Device();
		cout << cam->open(ANY_DEVICE) << endl;

		VideoStream* vid = new VideoStream();
		vid->create(*cam, SENSOR_DEPTH);
		VideoMode mode;
		mode = vid->getSensorInfo().getSupportedVideoModes()[atoi(argv[1])];
		int width = mode.getResolutionX();
		int height = mode.getResolutionY();
		cout << "Pixel Format: " << mode.getPixelFormat() << endl;
		cout << "Resolution: " << mode.getResolutionX() << "x" << mode.getResolutionY() << endl;

		vid->setVideoMode(mode);
		vid->start();
		for(int it = 0; it < atoi(argv[2]); it++)
		{
				cin.ignore();
				VideoFrameRef* frame = new VideoFrameRef();
				const void* a;
				int pointcount = 0;

				vid->readFrame(frame);

				a = frame->getData();
				RangeImagePlanar range_im;
				PointCloud<PointXYZ> cloud;
				range_im.setDepthImage(((const short unsigned int*) a), width, height, width/2, height/2, (float)(width/2*2.079), (float)(height/2*2.6131));
				cloud.width = height*width;
				cloud.height = 1;
				cloud.points.resize(height*width);
				cloud.is_dense=false;
				for(int i = 0; i < width; i++)
				{
						for(int j = 0; j < height; j++)
						{
								if(range_im.at(i,j).range < 10 && range_im.at(i,j).x != 0)
								{
										cloud.points[pointcount].x = range_im.at(i,j).x;
										cloud.points[pointcount].y = range_im.at(i,j).y;
										cloud.points[pointcount].z = range_im.at(i,j).z;
										pointcount++;
								}
						}
				}
				cloud.points.resize(pointcount);
				vector<int> indices;
				removeNaNFromPointCloud(cloud,cloud,indices);
				cloud.width = cloud.points.size();
				cout << cloud.points.size() << " points" << endl;
				stringstream filename;
				filename << "scans/" << it << ".pcd";
				io::savePCDFileBinary(filename.str(), cloud);
				cout << "saved " << "~/scans/" << it << ".pcd" << endl;
				frame->release();
		}
		cam->close();
		//        visualization::CloudViewer viewer("Test");
		//        viewer.showCloud(cloud.makeShared());
		//        while(!viewer.wasStopped()){}
}
