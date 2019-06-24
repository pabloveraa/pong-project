#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <libfreenect_sync.h>
#include "camera.h"
#include "game.h"
#include "calibration.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv){
  //capture a color image and a depth image to set the game area
  Mat imcolor(Size(640,480), CV_8UC3);
  while( capture_color(imcolor)==0 ){
    cout << "Failed to capture a color image!" << endl;
    freenect_sync_stop();
    sleep(4);
  }
  Mat imdepth(Size(320,240), CV_32F);
  while( capture_depth(imdepth)==0 ){
    cout << "Failed to capture image!" << endl;
    freenect_sync_stop();
    sleep(4);
  }

  //detect the floor, compute a rotation matrix to project the people
  //to the floor and compute the floor depth
  floor_parameters fp;
  fp.floor_depth = floor_detection(imdepth, fp.rotation_matrix, fp.floor_vector);

  //set the game area
  camera_parameters cam;
  get_parameters(cam, imcolor.size(),imdepth.size());
  Point2f pfloor[4];  //corners of the game area in the depth image
  set_game_area(pfloor, imcolor, imdepth, fp, cam);

  //scale and offset of the players
  float scale[4];
  scale_players(scale, pfloor, fp, cam);

  //save the calibration data to a file
  char fn[] = "calibration_data.txt";
  FILE* fid = fopen(fn, "w");
  write_calibration_data(fid, fp, scale, pfloor);
  fclose(fid);

  return 0;
}




