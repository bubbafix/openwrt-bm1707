all:bmtemp

bmtemp:bmtemp.o
	$(CXX) $(LDFLAGS) bmtemp.o -o bmtemp -lusb

bmtemp.o:bmtemp.cpp
	$(CXX) $(CXXFLAGS) -Wall -c bmtemp.cpp

clean:
	rm -f bmtemp *.o
