all:bmtemp

CXXFLAGS = -O2 -Wall

bmtemp:bmtemp.o
	$(CXX) $(LDFLAGS) bmtemp.o -o bmtemp -lusb

bmtemp.o:bmtemp.cpp
	$(CXX) $(CXXFLAGS) -c bmtemp.cpp

clean:
	rm -f bmtemp *.o
