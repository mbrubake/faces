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

using namespace openni;
using namespace std;
using namespace pcl;



int sgn(float f)
{
		if(f>0)
		{
				return 1;
		}
		return 0;
}
complex<float> fourierCoeff(const void* a, unsigned char* rgb, int width, int height, float px, float py)
{
		complex<float> ret = 0;
		complex<float> I (0,1);
		px /= width;
		py /= height;
		int d;
		for(int i = 0; i < width; i++)
		{
				for(int j = 0; j < height; j++)
				{
						//d=((const short unsigned int*)a)[i+j*width];
						//if(d==0 || d > 8000)
						//{
						ret += exp(I*(px*i + py*j))*((float)rgb[3*(i+j*width)] + (float)rgb[3*(i+j*width + 1)] + (float)rgb[3*(i+j*width + 2)]);
						//}
				}
		}
		return ret/((float)(width*height));
}

PointCloud<PointXYZRGB> toPCD(const void* a, unsigned char* b, int width, int height, float xfov, float yfov)
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
		int mode_id = 5;
		int mode_id_RGB = 9;
		string folder = argv[1];
		int delay = atoi(argv[2]);
		int consec_threshold = atoi(argv[3]);
		vector<void*> data;
		vector<void*> data2;
		vector<void*> dataRGB;
		vector<void*> dataRGB2;
		OpenNI::initialize();
		Device* cam = new Device();
		Device* cam2 = new Device();
		Array<DeviceInfo>* deviceInfoList = new Array<DeviceInfo>;
		OpenNI::enumerateDevices(deviceInfoList);
		cam->open(deviceInfoList->operator[](0).getUri());
		cam2->open(deviceInfoList->operator[](1).getUri());

		VideoStream* vid = new VideoStream();
		VideoStream* vid2 = new VideoStream();
		vid->create(*cam, SENSOR_DEPTH);
		vid2->create(*cam2, SENSOR_DEPTH);

		VideoStream* vidRGB = new VideoStream();
		VideoStream* vidRGB2 = new VideoStream();
		vidRGB->create(*cam, SENSOR_COLOR);	
		vidRGB2->create(*cam2, SENSOR_COLOR);

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
		vid2->setVideoMode(mode);
		vidRGB->setVideoMode(modeRGB);
		vidRGB2->setVideoMode(modeRGB);

		float xfov = vid->getHorizontalFieldOfView();
		float yfov = vid->getVerticalFieldOfView();
		vid->start();
		vid->setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, true);
		vid2->setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, true);

		VideoFrameRef* frame = new VideoFrameRef();
		vid->readFrame(frame);
		int datasize = frame->getDataSize();
		cout << "Depth Datasize: " << datasize << endl;
		frame->release();

		vidRGB->start();
		vidRGB2->start();

		VideoFrameRef* frameRGB = new VideoFrameRef();
		vidRGB->readFrame(frameRGB);
		int datasizeRGB = frameRGB->getDataSize();
		cout << "RGB Datasize: " << datasizeRGB <<endl;
		frameRGB->release();

		cin.ignore();

		vector<float> freqs;
		vector<complex<float> > fourier_coeffs;
		vector<complex<float> > dfourier_coeffs;
		complex<float> temp;
		for(int i = 3; i < 4; i++)
		{
				freqs.push_back(4*i+1);
				fourier_coeffs.push_back(complex<float> (1,0));
				dfourier_coeffs.push_back(complex<float> (1,0));
		}
		int consec=0;
		for(int i = 0; true; i++)
		{
				data.push_back(new void* [datasize]);
				dataRGB.push_back(new void*[datasizeRGB]);
				//cout << "Frame: " << i << endl;
				vid->readFrame(frame);
				vidRGB->readFrame(frameRGB);
				memcpy(data[i], frame->getData(), datasize);
				memcpy(dataRGB[i], frameRGB->getData(), datasizeRGB);
				for(int j = 0; j < freqs.size(); j++)
				{
						temp = fourierCoeff(frame->getData(), (unsigned char*) frameRGB->getData(), width, height, freqs[j], 0);
						dfourier_coeffs[j] = (temp / fourier_coeffs[j]);
						fourier_coeffs[j] = temp;
						cout << " " << sgn(arg(dfourier_coeffs[j]));
				}
				cout << endl;
				frame->release();
				frameRGB->release();
				if(sgn(arg(dfourier_coeffs[0]))==0)
				{
						consec++;
				}
				else
				{
						consec = 0;
				}
				if(consec>consec_threshold)
				{
						break;
				}
				usleep(delay*1000);
		}
		cout << "switch..." << endl;
		vid->setEmitterEnabled(false);
		vid2->start();
		consec=0;
		for(int i=0; true; i++)
		{
				data2.push_back(new void* [datasize]);
				dataRGB2.push_back(new void*[datasizeRGB]);
				//cout << "Frame: " << i << endl;
				vid2->readFrame(frame);
				vidRGB2->readFrame(frameRGB);
				memcpy(data2[i], frame->getData(), datasize);
				memcpy(dataRGB2[i], frameRGB->getData(), datasizeRGB);
				for(int j = 0; j < freqs.size(); j++)
				{
						temp = fourierCoeff(frame->getData(), (unsigned char*) frameRGB->getData(), width, height, freqs[j], 0);
						dfourier_coeffs[j] = (temp / fourier_coeffs[j]);
						fourier_coeffs[j] = temp;
						cout << " " << sgn(arg(dfourier_coeffs[j]));
				}
				cout << endl;
				frame->release();
				frameRGB->release();
				if(sgn(arg(dfourier_coeffs[0]))==1)
				{
						consec++;
				}
				else
				{
						consec = 0;
				}
				if(consec>consec_threshold)
				{
						break;
				}
				usleep(delay*1000);
		}
		vid->stop();
		vid2->stop();
		vidRGB->stop();
		vidRGB2->stop();
		cam->close();
		cam2->close();
		for(int i = 0; i < data.size(); i++)
		{
				PointCloud<PointXYZRGB> cloud = toPCD(data[i],(unsigned char*)dataRGB[i],width,height,xfov,yfov);
				stringstream filename;
				filename << folder << "a" << i << ".pcd";
				io::savePCDFileBinary(filename.str(), cloud);
				cout << "saved cloud of " << cloud.points.size() << " to " << filename.str()  << endl;
		}
		for(int i = 0; i < data2.size(); i++)
		{
				PointCloud<PointXYZRGB> cloud = toPCD(data2[i],(unsigned char*) dataRGB2[i],width,height,xfov,yfov);
				stringstream filename;
				filename << folder << "b" << i << ".pcd";
				io::savePCDFileBinary(filename.str(), cloud);
				cout << "saved cloud of " << cloud.points.size() << " to " << filename.str()  << endl;
		}
}
