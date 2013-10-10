#include "openni2/OpenNI.h"
#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <ctime>

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
						if(range_im.at(i,j).range < 8 && range_im.at(i,j).x != 0)
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

		Array<DeviceInfo>* deviceInfoList = new Array<DeviceInfo>;
		OpenNI::enumerateDevices(deviceInfoList);
		Device* cam1 = new Device();
		Device* cam2 = new Device();
		cam1->open(deviceInfoList->operator[](0).getUri());
		cam2->open(deviceInfoList->operator[](1).getUri());
		
		VideoStream* vid1 = new VideoStream();
		VideoStream* vid2 = new VideoStream();
		vid1->create(*cam1, SENSOR_DEPTH);
		vid2->create(*cam2, SENSOR_DEPTH);
		
		VideoMode mode;
		mode = vid1->getSensorInfo().getSupportedVideoModes()[mode_id];
		int width = mode.getResolutionX();
		int height = mode.getResolutionY();
		cout << "Pixel Format: " << mode.getPixelFormat() << endl;
		cout << "Resolution: " << mode.getResolutionX() << "x" << mode.getResolutionY() << endl;

		vid1->setVideoMode(mode);
		vid2->setVideoMode(mode);
		vid1->start();
		VideoFrameRef* frame;
		vid1->readFrame(frame);
		vid1->stop();
		int datasize = frame->getDataSize();
		frame->release();

		for(int i = 0; i < num_frames; i++)
		{
				data.push_back(new void* [datasize]);
		}
		cin.ignore();

		std::time_t start = time(0);
		for(int i = 0; i < num_frames; i++)
		{
				if(i%2 == 0)
				{
						vid1->start();
						vid1->readFrame(frame);
						vid1->stop();
				}
				else
				{
						vid2->start();
						vid2->readFrame(frame);
						vid2->stop();
				}
				memcpy(data[i], frame->getData(), datasize);
				frame->release();
		}
		time_t end = time(0);
		double diff = difftime(end,start)* 1000.0;
		cout<< diff << endl;
		cam1->close();
		cam2->close();
		for(int i = 0; i < num_frames; i++)
		{
				PointCloud<PointXYZ> cloud = toPCD(data[i],width,height);
				stringstream filename;
				filename << "scans/" << i << ".pcd";
				io::savePCDFileBinary(filename.str(), cloud);
				cout << "saved cloud of " << cloud.points.size() << " to " << filename.str() << endl;
		}
		//        visualization::CloudViewer viewer("Test");
		//        viewer.showCloud(cloud.makeShared());
		//        while(!viewer.wasStopped()){}
}
