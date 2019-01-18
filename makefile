all: pong

CFLAGS=-fPIC -g -Wall -O2 `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
INCLUDE = -I/usr/local/include/libfreenect
FREE_LIBS = -L/usr/local/lib -lfreenect_sync -lfreenect -L/usr/lib/arm-linux-gnueabihf -lX11


pong: main.cpp camera.cpp game.cpp figure.cpp calibration.cpp
	$(CXX) $(INCLUDE) $(CFLAGS) $? -o $@ $(LIBS) $(FREE_LIBS)

%.o: %.cpp
	$(CXX) -c $(CFLAGS) $< -o $@

clean:
	rm -rf *.o test

