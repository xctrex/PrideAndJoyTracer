########################################################################
# Makefile for Linux
########################################################################

OPTIMIZE = -g
CXXFLAGS = $(OPTIMIZE) -Wnarrowing -I. -I. -g -Iglsdk/glm -Iglsdk/boost -Iglsdk/glimg/include -Iglsdk/freeglut/include -Iglsdk/glload/include -I/usr/X11R6/include/GL/ -I/usr/include/GL/ -fopenmp -msse3 -DEIGEN_NO_DEBUG -std=c++11
LIBS = -pthread -L/usr/lib  -L/usr/local/lib -lassimp -lAntTweakBar -lfreeglut -lX11 -lGLU -lGL -L/usr/X11R6/lib -Lglsdk/glimg/lib/ -Lglsdk/glload/lib/ -Lglsdk/freeglut/lib/ -lglload -lglimg


target = raytrace.exe

headers = raytrace.h rgbe.h realtime.h
src = main.cpp raytrace.cpp rgbe.cpp realtime.cpp readAssimpFile.cpp
extras = raytrace.vcxproj Makefile realtime.vert realtime.frag Eigen Eigen_unsupported glsdk assimp librgbe.so librply.so

scenes = shapetest.scn letterX.ply letterY.ply jeep1.3ds ~/projects/assets/bunny.ply dwarf.x

pkgFiles = $(src) $(headers) $(extras) $(scenes)
pkgName = CS500-framework

objects = $(patsubst %.cpp,%.o,$(src))


$(target): $(objects) librgbe.so
	g++  $(CXXFLAGS) -o $@  $(objects) $(LIBS)

librgbe.so:
	g++ -c -fPIC rgbe.cpp -o rgbe.o
	g++ rgbe.o -shared -o librgbe.so

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: $(target)
	#python3 driver.py
	./raytrace.exe shapetest.scn

zip:
	rm -rf ../$(pkgName) ../$(pkgName).zip
	mkdir ../$(pkgName)
	cp -r $(pkgFiles) ../$(pkgName)
	cd ..;  zip -r $(pkgName).zip $(pkgName); rm -rf $(pkgName)

pyzip: $(pyFiles)
	rm -rf ../$(pkgName) ../$(pkgName)-py.zip
	mkdir ../$(pkgName)
	cp -r $(pyFiles) ../$(pkgName)
	rm -f `find ../$(pkgName) -name '*.pyc'`
	cd ..;  zip -r $(pkgName)-py.zip $(pkgName); rm -rf $(pkgName)

clean:
	rm -rf *.suo *.sdf *.orig Release Debug ipch *.o *~ raytrace dependencies *13*scn  *13*ppm 

dependencies: 
	g++ -MM $(CXXFLAGS)  $(src) > dependencies

include dependencies
