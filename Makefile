capture: capture.cpp
	clang -O capture.cpp -I/home/jonathan/faces/.lib/OpenNI/Include -L/home/jonathan/faces/.lib/OpenNI/Redist -lOpenNI2 -I/usr/include/vtk-5.8 -I/usr/include/pcl-1.7 -lpcl_common -lpcl_io -I/usr/lib -lstdc++ -lm -lboost_system -lpcl_filters  -lvtkCommon -I/usr/include/eigen3 -o bin/capture

stitch: stitch.cpp
	clang -O stitch.cpp  -I/usr/lib -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -lstdc++ -lm -lboost_system -lpcl_common -lpcl_io -lpcl_registration -lpcl_search -lpcl_visualization -lpcl_filters -lpcl_features -I/usr/include/vtk-5.8 -lvtkCommon -o bin/stitch

dep.h.gch: dep.h
	g++ -x c++-header -O -c dep.h -lstdc++ -I/usr/lib -I/usr/include/ -I/usr/lib -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/vtk-5.8

transform: transform.cpp
	clang -O transform.cpp  -I/usr/lib -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -lstdc++ -lm -lboost_system -lpcl_common -lpcl_io -lpcl_registration -lpcl_search -lpcl_visualization -lpcl_filters -lpcl_features -I/usr/include/vtk-5.8 -lvtkCommon -o bin/transform

ransac: ransac.cpp
	clang -O ransac.cpp -I/usr/lib -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -lm -lstdc++ -l:libboost_system.so.1.49.0 -lpcl_common -lpcl_kdtree -lpcl_io -lpcl_registration -lpcl_search -lpcl_visualization -lpcl_filters -lpcl_features -lpcl_kdtree -I/usr/include/vtk-5.8 -lvtkCommon -lvtkFiltering -lvtkRendering -o bin/ransac

graft: graft.cpp
	clang -O graft.cpp -I/usr/include/pcl-1.7 -lpcl_common -lpcl_io -I/usr/lib -lstdc++ -lboost_system -lpcl_registration -lpcl_search -lpcl_kdtree -lpcl_filters -lpcl_surface -lpcl_features -I/usr/include/vtk-5.8 -I/usr/include/eigen3 -o bin/graft

super: super.cpp
	g++ -w -O super.cpp -lpcl_common -lpcl_io -lm -lstdc++ -lboost_system -lpcl_search -lpcl_kdtree -lpcl_surface -lpcl_filters -lpcl_registration -lpcl_visualization -I/usr/include/vtk-5.8 -I/usr/include/eigen3 -o bin/super

view: view.cpp
	clang -O view.cpp -I/usr/include/pcl-1.7 -I/usr/lib -lstdc++ -lboost_system -lpcl_common -lpcl_io -lpcl_visualization -I/usr/include/vtk-5.8 -lvtkCommon -o bin/view

viewrgb: viewrgb.cpp
	clang -O viewrgb.cpp -I/usr/include/pcl-1.7 -I/usr/lib -lstdc++ -lboost_system -lpcl_common -lpcl_io -lpcl_visualization -I/usr/include/vtk-5.8 -lvtkCommon -o bin/viewrgb

behead: behead.cpp
	clang -O behead.cpp -I/usr/include/pcl-1.7 -lpcl_common -lpcl_io -I/usr/lib -lstdc++ -lm -lboost_system -lpcl_search -lpcl_visualization -lpcl_segmentation  -lpcl_filters -I/usr/include/vtk-5.8 -lvtkCommon -I/usr/include/eigen3 -lutil -o bin/behead
