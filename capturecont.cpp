#include "OpenNI.h"
#include <PS1080.h>
#include <iostream>
#include <string>
#include <math.h>
#include <unistd.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/filters/filter.h>
#include <ctime>

using namespace openni;
using namespace std;
using namespace pcl;

class FrameGetter : public VideoStream::NewFrameListener
{
		public:
				vector<void*> data;
				VideoFrameRef frame;
				int datasize;
				int i;
				void onNewFrame(VideoStream& stream)
				{
						if(i < data.size())
						{
								stream.readFrame(&frame);
								memcpy(data[i], frame.getData(), datasize);
								frame.release();
								i++;
						}
				}
};

PointCloud<PointXYZ> toPCD(const void* a, int width, int height, float xfov, float yfov)
{
		RangeImagePlanar range_im;
		PointCloud<PointXYZ> cloud;
		range_im.setDepthImage(((const short unsigned int*) a), width, height, width/2, height/2, (float)(width/2)/tan(xfov/2), (float)(height/2)/tan(yfov/2));
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
		int mode_id = 5;
		string folder = argv[1];
		int num_frames = atoi(argv[2]);
		vector<void*> data1;
		vector<void*> data2;
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
		cout << "Field of View: " << vid1->getHorizontalFieldOfView() << "x" << vid1->getVerticalFieldOfView() << endl;
		float xfov = vid1->getHorizontalFieldOfView();
		float yfov = vid1->getVerticalFieldOfView();

		vid1->start();
		vid2->start();

		vid1->setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE,true);
		vid2->setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE,true);

		VideoFrameRef* frame1 = new VideoFrameRef();
		VideoFrameRef* frame2 = new VideoFrameRef();
		vid1->readFrame(frame1);
		int datasize = frame1->getDataSize();
		frame1->release();

		for(int i = 0; i < num_frames; i++)
		{
				data1.push_back(new void* [datasize]);
				data2.push_back(new void* [datasize]);
		}
		cin.ignore();

		FrameGetter d1;
		FrameGetter d2;
		d1.data = data1;
		d2.data = data2;
		d1.datasize = datasize;
		d2.datasize = datasize;
		d1.i=0;
		d2.i=0;
		vid1->addNewFrameListener(&d1);
		vid2->addNewFrameListener(&d2);
		while(d1.i < num_frames || d2.i < num_frames)
		{
				usleep(100000);
		}

		vid1->stop();
		vid2->stop();
		vid1->destroy();
		vid2->destroy();
		cam1->close();
		cam2->close();
		OpenNI::shutdown();
		for(int i = 0; i < num_frames; i++)
		{
				PointCloud<PointXYZ> cloud = toPCD(data1[i],width,height,xfov,yfov);
				stringstream filename;
				filename << folder << i << ".pcd";
				if(cloud.points.size() > 0)
				{	
						io::savePCDFileBinary(filename.str(), cloud);
						cout << "saved cloud of " << cloud.points.size() << " to " << filename.str() << endl;
				}
		}
		for(int i = 0; i < num_frames; i++)
		{
				PointCloud<PointXYZ> cloud = toPCD(data2[i],width,height,xfov,yfov);
				stringstream filename;
				filename << folder << "s" << i << ".pcd";
				if(cloud.points.size() > 0)
				{
						io::savePCDFileBinary(filename.str(), cloud);
						cout << "saved cloud of " << cloud.points.size() << " to " << filename.str() << endl;
				}

		}
}
