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
		int num_cycles = atoi(argv[2]);
		int num_frames = atoi(argv[3]);
		int delay = atoi(argv[4]);
		int delay2 = atoi(argv[5]);
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
		vid1->readFrame(frame1);
		int datasize = frame1->getDataSize();

		for(int i = 0; i < num_frames; i++)
		{
				data1.push_back(new void* [datasize]);
				data2.push_back(new void* [datasize]);
		}

		frame1->release();

		vid1->setEmitterEnabled(false);
		vid2->setEmitterEnabled(false);

		cin.ignore();
		Recorder r1;
		Recorder r2;

		r1.create("record1.oni");
		r2.create("record2.oni");

		r1.attach(*vid1, false);
		r2.attach(*vid2, false);
		r1.start();
		r2.start();		
		for(int i = 0; i < num_cycles; i++)
		{
				cam1->setProperty(XN_MODULE_PROPERTY_EMITTER_STATE, true);
				usleep(delay2*1000);
				cam2->setProperty(XN_MODULE_PROPERTY_EMITTER_STATE, false);
				usleep(delay*1000);
				cout << "a" << endl;

				cam2->setProperty(XN_MODULE_PROPERTY_EMITTER_STATE,true);
				usleep(delay2*1000);
				cam1->setProperty(XN_MODULE_PROPERTY_EMITTER_STATE,false);
				usleep(delay*1000);
				cout << "b" << endl;
		}
		r1.stop();
		r2.stop();
		vid1->stop();
		vid2->stop();
		//vid1->destroy();
		//vid2->destroy();
		cam1->close();
		cam2->close();
		//OpenNI::shutdown();
		Device* rec1 = new Device();
		Device* rec2 = new Device();
		rec1->open("record1.oni");
		rec2->open("record2.oni");
		cout << "here" << endl;
		VideoStream* stream1 = new VideoStream();
		VideoStream* stream2 = new VideoStream();
		stream1->create(*rec1, SENSOR_DEPTH);
		stream2->create(*rec2, SENSOR_DEPTH);
		cout << "here" << endl;
		//stream1->setVideoMode(mode);
		//stream2->setVideoMode(mode);

		stream1->start();
		stream2->start();

		stream1->setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE,true);
		stream2->setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE,true);
		cout << "here" << endl;
		for(int i = 0; i < num_frames; i++)
		{
				stream1->readFrame(frame1);
				memcpy(data1[i], frame1->getData(), datasize);
				frame1->release();
		}

		for(int i = 0; i < num_frames; i++)
		{
				stream2->readFrame(frame1);
				memcpy(data2[i], frame1->getData(), datasize);
				frame1->release();
		}
		cout << "here" << endl;
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
