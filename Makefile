CC=g++ -fopenmp

#ONI2_INC_DIR=/home/jonathan/faces/.lib/OpenNI/Include
#ONI2_LIB_DIR=/home/jonathan/faces/.lib/OpenNI/Redist 
#PCL_INC_DIR=/usr/include/pcl-1.7
#PCL_LIB_DIR=/usr/lib
#VTK_INC_DIR=/usr/include/vtk-5.8
#VTK_LIB_DIR=/usr/lib

ONI2_INC_DIR=/home/mbrubake/build/OpenNI2/Include
ONI2_LIB_DIR=/home/mbrubake/build/OpenNI2/Bin/x64-Release
PCL_INC_DIR=/home/mbrubake/Software/pcl/include/pcl-1.7
PCL_LIB_DIR=/home/mbrubake/Software/pcl/lib
VTK_INC_DIR=/usr/include/vtk
VTK_LIB_DIR=/usr/lib64/vtk

CFLAGS=-O3 -I$(VTK_INC_DIR) -I$(PCL_INC_DIR) -I/usr/include/eigen3 -I$(ONI2_INC_DIR)
LDFLAGS=-L$(PCL_LIB_DIR) -lpcl_common -lpcl_io -lpcl_filters -lpcl_registration -lpcl_search -lpcl_visualization -lpcl_features -lpcl_surface -lpcl_kdtree -lpcl_segmentation -lstdc++ -lm -lboost_system -L$(VTK_LIB_DIR) -lvtkCommon -lvtkFiltering -lvtkRendering -L$(ONI2_LIB_DIR) -lOpenNI2

all: bin/capture bin/stitch bin/behead bin/graft bin/viewrgb

bin/%: %.cpp
	$(CC) $< $(CFLAGS) $(LDFLAGS) -o $@

