#include "OpenNI.h"
#include <PS1080.h>
#include <iostream>
#include <string>
#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/filters/filter.h>

using namespace openni;
using namespace std;
using namespace pcl;

PointCloud<PointXYZRGB> toPCD(const void* a, const void* b, int width, int height, float xfov, float yfov)
{
		RangeImagePlanar range_im;
		PointCloud<PointXYZRGB> cloud;
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
								cloud.points[pointcount].r = ((unsigned char*) b)[3*(i+j*width)];
								cloud.points[pointcount].g = ((unsigned char*) b)[3*(i+j*width)+1];
								cloud.points[pointcount].b = ((unsigned char*) b)[3*(i+j*width)+2];
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
		int mode_id_RGB = 9;
		string folder = argv[1];
		int num_frames = atoi(argv[2]);
		int delay = atoi(argv[3]);
		vector<void*> data;
		vector<void*> dataRGB;
		OpenNI::initialize();
		Device* cam = new Device();
		Array<DeviceInfo>* deviceInfoList = new Array<DeviceInfo>;
		OpenNI::enumerateDevices(deviceInfoList);
		cam->open(ANY_DEVICE);
		VideoStream* vid = new VideoStream();
		vid->create(*cam, SENSOR_DEPTH);

		VideoStream* vidRGB = new VideoStream();
		vidRGB->create(*cam, SENSOR_COLOR);	

		VideoMode mode;
		VideoMode modeRGB;
		mode = vid->getSensorInfo().getSupportedVideoModes()[mode_id];
		modeRGB = vidRGB->getSensorInfo().getSupportedVideoModes()[mode_id_RGB];
		int width = mode.getResolutionX();
		int height = mode.getResolutionY();

		cam->setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
		cam->setDepthColorSyncEnabled(true);
		cout << "Registration Mode: " << cam->getImageRegistrationMode() << endl;
		cout << "Depth Pixel Format: " << mode.getPixelFormat() << endl;
		cout << "Resolution: " << mode.getResolutionX() << "x" << mode.getResolutionY() << "x" << mode.getFps() << endl;
		cout << "RGB Pixel Format: " << modeRGB.getPixelFormat() << endl;
		cout << "RGBResolution: " << modeRGB.getResolutionX() << "x" << modeRGB.getResolutionY() << "x" << modeRGB.getFps() << endl;

		vid->setVideoMode(mode);
		vidRGB->setVideoMode(modeRGB);

		cout << "Field of View: " << vid->getHorizontalFieldOfView() << "x" << vid->getVerticalFieldOfView() << endl;
		float xfov = vid->getHorizontalFieldOfView();
		float yfov = vid->getVerticalFieldOfView();
		vid->start();
		vid->setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, true);

		VideoFrameRef* frame = new VideoFrameRef();
		vid->readFrame(frame);
		int datasize = frame->getDataSize();
		cout << "Depth Datasize: " << datasize << endl;
		frame->release();

		vidRGB->start();

		VideoFrameRef* frameRGB = new VideoFrameRef();
		vidRGB->readFrame(frameRGB);
		int datasizeRGB = frameRGB->getDataSize();
		cout << "RGB Datasize: " << datasizeRGB <<endl;
		frameRGB->release();

		for(int i = 0; i < num_frames; i++)
		{
				data.push_back(new void* [datasize]);
				dataRGB.push_back(new void* [datasizeRGB]);
		}
		cin.ignore();
		for(int i = 0; i < num_frames; i++)
		{
				cout << "Frame: " << i << endl;
//				vid->setEmitterEnabled(true);
				vid->readFrame(frame);
				vidRGB->readFrame(frameRGB);
//				vid->setEmitterEnabled(false);
				memcpy(data[i], frame->getData(), datasize);
				memcpy(dataRGB[i], frameRGB->getData(), datasizeRGB);
				frame->release();
				frameRGB->release();
				usleep(delay*1000);
		}
		vid->stop();
		vidRGB->stop();
		cam->close();
		for(int i = 0; i < num_frames; i++)
		{
				PointCloud<PointXYZRGB> cloud = toPCD(data[i],dataRGB[i],width,height,xfov,yfov);
				stringstream filename;
				filename << folder << i << ".pcd";
				io::savePCDFileBinary(filename.str(), cloud);
				cout << "saved cloud of " << cloud.points.size() << " to " << filename.str()  << endl;
		}
}
