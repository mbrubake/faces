#include "OpenNI.h"
#include <PS1080.h>
#include <iostream>
#include <string>
#include <math.h>
#include <complex>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/filters/filter.h>

#include <map>
#include <fstream>
#include <stdio.h>
#include <ctype.h>
#include <unistd.h>

using namespace openni;
using namespace std;
using namespace pcl;

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
PointCloud<PointXYZRGB> toPCD(const void* a, unsigned char* b, int width, int height, float xfov, float yfov, float cutoff)
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
			if(range_im.at(i,j).range < cutoff && range_im.at(i,j).x != 0)
			{
				cloud.points[pointcount].x = range_im.at(i,j).x;
				cloud.points[pointcount].y = range_im.at(i,j).y;
				cloud.points[pointcount].z = range_im.at(i,j).z;
				cloud.points[pointcount].r = b[3*(i+j*width)];
				cloud.points[pointcount].g = b[3*(i+j*width)+1];
				cloud.points[pointcount].b = b[3*(i+j*width)+2];
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
	map<string,parameter> params = loadConfig("capture.config");

	string output = params["output"].s;
	int delay = params["delay"].i;
	int num_frames = params["num_frames"].i;
	float cutoff = params["cutoff"].f;
	
	int c;
	while((c = getopt (argc, argv, "o:n:")) !=-1)
	{
		switch(c)
		{
			case 'o':
				output = optarg;
				break;
			case 'n':
				num_frames = atoi(optarg);
				break;
		}
	}
	int mode_id = 5;
	int mode_id_RGB = 9;
	string folder = output;
	
	vector<void*> data;
	vector<void*> dataRGB;
	OpenNI::initialize();
	Device* cam = new Device();
	
	Array<DeviceInfo>* deviceInfoList = new Array<DeviceInfo>;
	OpenNI::enumerateDevices(deviceInfoList);
	cam->open(deviceInfoList->operator[](0).getUri());

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
	cout << "Field of View: " << vid->getHorizontalFieldOfView() << "x" << vid->getVerticalFieldOfView() << endl;

	vid->setVideoMode(mode);
	vidRGB->setVideoMode(modeRGB);

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

	cin.ignore();

	for(int i = 0; i<num_frames; i++)
	{
		data.push_back(new void* [datasize]);
		dataRGB.push_back(new void*[datasizeRGB]);
		//cout << "Frame: " << i << endl;
		vid->readFrame(frame);
		vidRGB->readFrame(frameRGB);
		memcpy(data[i], frame->getData(), datasize);
		memcpy(dataRGB[i], frameRGB->getData(), datasizeRGB);
		frame->release();
		frameRGB->release();
		
		usleep(delay*1000);
	}
	vid->stop();
	for(int i = 0; i < data.size(); i++)
	{
		PointCloud<PointXYZRGB> cloud = toPCD(data[i],(unsigned char*)dataRGB[i],width,height,xfov,yfov, cutoff);
		stringstream filename;
		filename << folder << i << ".pcd";
		io::savePCDFileBinary(filename.str(), cloud);
		cout << "saved cloud of " << cloud.points.size() << " to " << filename.str()  << endl;
	}
}
