#CXX      = arm-linux-g++ -g
#CXX      = arm-linux-g++ -O3 #Use this one if you dont want to use the debugger
CXX		  = g++ #use this for testing locally
#CFLAGS   = -pipe  
CXXFLAGS = -DLINUX
#INCPATH  = -I$(QTDIR)/mkspecs/linux-g++ -I/usr/local/include/opencv

#LINK     = arm-linux-g++
LINK = g++ -larmadillo
LIBS     = -lpthread

####### Files

OBJECTS = obj/AckPacket.o obj/CommPacket.o obj/CommSerialLinux.o obj/GPSTelemPacket.o obj/StdTelemPacket.o obj/TranslatePacket.o obj/WriteRawPacket.o obj/Demo1.o
TARGET  = Control

####### Build rules

$(TARGET): $(OBJECTS)  
	$(LINK)  -o $(TARGET) $(OBJECTS)  $(LIBS)

obj/AckPacket.o: src/Serial/AckPacket.cc src/Serial/AckPacket.h src/Serial/CommPacket.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o obj/AckPacket.o src/Serial/AckPacket.cc

obj/CommPacket.o: src/Serial/CommPacket.cc src/Serial/CommPacket.h src/Serial/dllsetup.h src/Serial/type.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o obj/CommPacket.o src/Serial/CommPacket.cc

obj/CommSerialLinux.o: src/Serial/CommSerialLinux.cc src/Serial/CommSerialLinux.h src/Serial/dllsetup.h src/Serial/TranslatePacket.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o obj/CommSerialLinux.o src/Serial/CommSerialLinux.cc

obj/GPSTelemPacket.o: src/Serial/GPSTelemPacket.cc src/Serial/GPSTelemPacket.h src/Serial/CommPacket.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o obj/GPSTelemPacket.o src/Serial/GPSTelemPacket.cc

obj/StdTelemPacket.o: src/Serial/StdTelemPacket.cc src/Serial/StdTelemPacket.h src/Serial/CommPacket.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o obj/StdTelemPacket.o src/Serial/StdTelemPacket.cc

obj/TranslatePacket.o: src/Serial/TranslatePacket.cc src/Serial/TranslatePacket.h src/Serial/StdTelemPacket.h src/Serial/GPSTelemPacket.h src/Serial/CommPacket.h src/Serial/WriteRawPacket.h src/Serial/AckPacket.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o obj/TranslatePacket.o src/Serial/TranslatePacket.cc

obj/WriteRawPacket.o: src/Serial/WriteRawPacket.cc src/Serial/WriteRawPacket.h src/Serial/CommPacket.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o obj/WriteRawPacket.o src/Serial/WriteRawPacket.cc
	
obj/Demo1.o: src/Demo1.cpp
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o obj/Demo1.o src/Demo1.cpp -larmadillo 
	
clean:
	-rm -f $(TARGET) 
	-rm -f $(OBJECTS) 
	-rm -f *~ core *.core

