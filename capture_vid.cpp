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

PointCloud<PointXYZ> toPCD(const void* a, int width, int height)
{
		RangeImagePlanar range_im;
		PointCloud<PointXYZ> cloud;
		range_im.setDepthImage(((const short unsigned int*) a), width, height, width/2, height/2, (float)(width/2*2.079), (float)(height/2*2.6131));
		cloud.width = height*width;
		cloud.height = 1;
		cloud.points.resize(height*width);
		cloud.is_dense=false;
		int pointcount = 0;
		for(int i = 0; i < width; i++)
		{
				for(int j = 0; j < height; j++)
				{
						if(range_im.at(i,j).range < 6 && range_im.at(i,j).x != 0)
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

		return cloud; 
}

int main(int argc, char** argv)
{
		int num_frames = atoi(argv[2]);
		int mode_id = atoi(argv[1]);
		vector<void*> data;
		OpenNI::initialize();
		Device* cam = new Device();
		cam->open(ANY_DEVICE);

		VideoStream* vid = new VideoStream();
		vid->create(*cam, SENSOR_DEPTH);
		VideoMode mode;
		mode = vid->getSensorInfo().getSupportedVideoModes()[mode_id];
		int width = mode.getResolutionX();
		int height = mode.getResolutionY();
		cout << "Pixel Format: " << mode.getPixelFormat() << endl;
		cout << "Resolution: " << mode.getResolutionX() << "x" << mode.getResolutionY() << endl;

		vid->setVideoMode(mode);
		vid->start();
		VideoFrameRef* frame;
		vid->readFrame(frame);
		int datasize = frame->getDataSize();
		frame->release();

		for(int i = 0; i < num_frames; i++)
		{
				data.push_back(new void* [datasize]);
		}
		cin.ignore();
		for(int i = 0; i < num_frames; i++)
		{
				vid->readFrame(frame);
				memcpy(data[i], frame->getData(), datasize);
				frame->release();
		}
		vid->stop();
		cam->close();
		for(int i = 0; i < num_frames; i++)
		{
				PointCloud<PointXYZ> cloud = toPCD(data[i],width,height);
				stringstream filename;
				filename << "/home/jonathan/scans/" << i << ".pcd";
				io::savePCDFileBinary(filename.str(), cloud);
				cout << "saved cloud of " << cloud.points.size() << " to ~/scans/" << i << ".pcd" << endl;
		}
		//        visualization::CloudViewer viewer("Test");
		//        viewer.showCloud(cloud.makeShared());
		//        while(!viewer.wasStopped()){}
}
