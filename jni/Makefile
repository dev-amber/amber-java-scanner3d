CLASS_PATH = ../target/classes/:.

vpath %.class $(CLASS_PATH)

all : libjoinedpdccreator.so

libjoinedpdccreator.so : JoinedPDCCreator.o
	g++ -L"/usr/lib/x86_64-linux-gnu/" -shared -Wl,-soname,libjoinedpdccreator.so -o libjoinedpdccreator.so JoinedPDCCreator.o `pkg-config --libs pcl_apps-1.7 pcl_common-1.7 pcl_features-1.7 pcl_filters-1.7 pcl_geometry-1.7 pcl_io-1.7 pcl_kdtree-1.7 pcl_keypoints-1.7 pcl_octree-1.7 pcl_registration-1.7 pcl_sample_consensus-1.7 pcl_search-1.7 pcl_segmentation-1.7 pcl_surface-1.7 pcl_tracking-1.7 pcl_visualization-1.7 flann`

JoinedPDCCreator.o : clean
	g++ -L"/usr/lib/x86_64-linux-gnu/" -fPIC -I"/usr/include/vtk-5.8" -I"/usr/include/eigen3" -I"/usr/local/include/pcl-1.7" -I"/opt/java/64/jdk1.7.0_67/include" -I"/opt/java/64/jdk1.7.0_67/include/linux" -c JoinedPDCCreator.cpp -o JoinedPDCCreator.o `pkg-config --libs pcl_apps-1.7 pcl_common-1.7 pcl_features-1.7 pcl_filters-1.7 pcl_geometry-1.7 pcl_io-1.7 pcl_kdtree-1.7 pcl_keypoints-1.7 pcl_octree-1.7 pcl_registration-1.7 pcl_sample_consensus-1.7 pcl_search-1.7 pcl_segmentation-1.7 pcl_surface-1.7 pcl_tracking-1.7 pcl_visualization-1.7 flann`

clean :
	rm -rf *.o *.so

# Can't get PCLWrapper without prefix, so this target must be invoked manually before all is invoked.
PCLWrapper.h :
	/usr/bin/javah -classpath $(CLASS_PATH) pl.edu.agh.scanner.pcl.PCLWrapper