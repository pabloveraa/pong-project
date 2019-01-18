#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <libfreenect_sync.h>

using namespace std;
using namespace cv;

//field of view of the depth and color cameras in degrees
const float FOV_DEPTH_CAMERA_HORIZ = 58.5;
const float FOV_DEPTH_CAMERA_VERT = 46.6;
const float FOV_COLOR_CAMERA_HORIZ = 62.0;
const float FOV_COLOR_CAMERA_VERT = 48.6;


//capture a depth image with the kinect
int capture_depth(Mat imdepth);

//capture a color image with the kinect
int capture_color(Mat imcolor);

//detect the floor, compute a rotation matrix to project the depth images
//to the floor and return the floor depth from the camera
float floor_detection(Mat imdepth, Mat rotation_matrix, Mat floor_vector);

//track the players in the depth image
void point_tracker(float* pLoc, Mat imdepth, Mat rotation_matrix, float floor_depth);

//locate the center of an object in a binary image using mean shift
void mean_shift(Scalar& x, Scalar& y, Mat mask, float sigma, float conv_thr);

//compute the coordinates of the 3D points from a depth image
void get_3Dpoints(Mat imdepth, Mat M);

//get a rotated coordinate of the 3D points
void rotate_points(Mat R, Mat M, Mat R_row);

//compute the focal length and the principal point of a camera
void get_camera_parameters(float* focal_length, float* image_center, float* fov, int cols, int rows);

//create a grid of 2D point coordinates
void mesh_grid(Mat Um, Mat Vm);


#endif
