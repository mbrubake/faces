capture: capture.cpp
	clang capture.cpp -I/usr/include/openni2 -lOpenNI2 -I/usr/include/vtk-5.8 -I/usr/include/pcl-1.7 -lpcl_common -lpcl_io -I/usr/lib -lstdc++ -lboost_system -lpcl_visualization -lpcl_filters  -lvtkCommon -I/usr/include/eigen3 -o capture

capture_vid: capture_vid.cpp
	clang capture_vid.cpp -I/usr/include/openni2 -lOpenNI2 -I/usr/include/vtk-5.8 -I/usr/include/pcl-1.7 -lpcl_common -lpcl_io -I/usr/lib -lstdc++ -lboost_system -lpcl_visualization -lpcl_filters  -lvtkCommon -I/usr/include/eigen3 -o capture_vid

register: register.cpp
	clang register.cpp  -I/usr/lib -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -lstdc++ -lm -lboost_system -lpcl_common -lpcl_io -lpcl_registration -lpcl_search -lpcl_visualization -lpcl_filters -I/usr/include/vtk-5.8 -lvtkCommon -o register

register2: register2.cpp
	clang register2.cpp  -I/usr/lib -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -lstdc++ -lm -l:libboost_system.so.1.49.0 -lpcl_common -lpcl_io -lpcl_registration -lpcl_search -lpcl_visualization -lpcl_filters -lpcl_features -I/usr/include/vtk-5.8 -lvtkCommon -lvtkFiltering -lvtkRendering -o register2


triangulate: triangulate.cpp
	clang triangulate.cpp -I/usr/include/pcl-1.7 -lpcl_common -lpcl_io -I/usr/lib -lstdc++ -lboost_system -lpcl_registration -lpcl_search -lpcl_filters -lpcl_surface -lpcl_features -I/usr/include/vtk-5.8 -I/usr/include/eigen3 -o triangulate

view: view.cpp
	clang view.cpp -I/usr/include/pcl-1.7 -I/usr/lib -lstdc++ -lboost_system -lpcl_common -lpcl_io -lpcl_visualization -I/usr/include/vtk-5.8 -lvtkCommon -o view

segment: segment.cpp
	clang segment.cpp -I/usr/include/pcl-1.7 -lpcl_common -lpcl_io -I/usr/lib -lstdc++ -lboost_system -lpcl_search -lpcl_visualization -lpcl_segmentation  -lpcl_filters -I/usr/include/vtk-5.8 -lvtkCommon -I/usr/include/eigen3 -o segment

