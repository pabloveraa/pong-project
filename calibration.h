#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

struct floor_parameters{
  Mat rotation_matrix = Mat::zeros(3,3,CV_32F);
  Mat floor_vector = Mat::zeros(3,1,CV_32F);
  float floor_depth;
};

struct camera_parameters{
  float fc_color[2];  //focal length of the color camera
  float cc_color[2];  //principal point of the color camera
  float fc_depth[2];  //focal length of the depth camera
  float cc_depth[2];  //principal point of the depth camera
};

//get the kinect color and depth camera parameters
void get_parameters(camera_parameters& cam, Size size_color, Size size_depth);

//set the game area using the image of the area illuminated by the projector
void set_game_area(Point2f* pfloor, Mat imcolor, Mat imdepth, floor_parameters fp, camera_parameters cam);

//map the selected points in the color image to the depth image
void color_depth_mapping(Point2f* pfloor, Point2f* pcolor, floor_parameters fp, camera_parameters cam);

//compute the error of mapping a point in the color image to the depth image
float error_mapping(Point2f pfloor, float Zcolor);

//get a scale factor and an offset for the detected positions of the players
//to map them to the plot units used for the game
void scale_players(float* scale, Point2f* pfloor, floor_parameters fp, camera_parameters cam);

//get the 3D coordinates of a point on the floor from the pixel location
Point3f floor_point(Point2f pfloor, floor_parameters fp, camera_parameters cam);

//function to handle mouse clicks
void mouseHandler(int event, int x, int y, int flags, void* ptr);

//save the calibration data to a file
void write_calibration_data(FILE* fid, floor_parameters fp, float* scale, Point2f* pfloor);

//load the calibration data from the calibration file
void read_calibration_data(FILE* fid, floor_parameters& fp, float* scale);


#endif


