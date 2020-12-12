CC = g++
INCLUDE_PATH =-I "D:\Program Files\opencv\build\x64\mingw\include"
LIB_PATH =-L "D:\Program Files\opencv\build\x64\mingw\x64\mingw\lib"
ADD_LIB = -lopencv_core341 -lopencv_highgui341 -lopencv_imgproc341 -lopencv_imgcodecs341 -lopencv_videoio341
SRC = main.o hough.o

all: line

line: $(SRC)
	$(CC) $(SRC) $(LIB_PATH) $(ADD_LIB)

%.o: %.cpp %.h
	$(CC) $(INCLUDE_PATH) $(LIB_PATH) -c -o $@ $<

%.o: %.cpp
	$(CC) $(INCLUDE_PATH) $(LIB_PATH) -c -o $@ $<

clean:
	del -f *.o hough