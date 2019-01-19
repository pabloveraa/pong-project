#include <math.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "calibration.h"
#include "camera.h"
#include "game.h"

using namespace std;
using namespace cv;

//rotation and translation from the depth to the color camera systems
Mat ROTATION_DEPTH_TO_COLOR = (Mat_<float>(3,3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
Mat TRANSLATION_DEPTH_TO_COLOR = (Mat_<float>(3,1) << -50.0, 0.0, 0.0);

Point2f pcolor_min;  //point selected in the color image
floor_parameters fp_min;
camera_parameters cam_min;

//class to perform the color to depth mapping using downhill solver
class minMapping:public MinProblemSolver::Function{
  public:
    int getDims() const{ return 3; }
    double calc( const double* x) const{
      Point2f pfloor;
      pfloor.x = (float)x[0];
      pfloor.y = (float)x[1];
      float Zcolor = (float)x[2];
      float error = error_mapping(pfloor, Zcolor);
      return (double)error;
    }
};


//============================================================
//get the kinect color and depth camera parameters
void get_parameters(camera_parameters& cam, Size size_color, Size size_depth){
  float pi = 3.1416;
  //color camera image center
  cam.cc_color[0] = 0.5*(float)size_color.width;
  cam.cc_color[1] = 0.5*(float)size_color.height;
  //color camera focal length in pixel dimensions
  cam.fc_color[0] = cam.cc_color[0] / tan(0.5*FOV_COLOR_CAMERA_HORIZ*pi/180.0);
  cam.fc_color[1] = cam.cc_color[1] / tan(0.5*FOV_COLOR_CAMERA_VERT*pi/180.0);

  //depth camera image center
  cam.cc_depth[0] = 0.5*(float)size_depth.width;
  cam.cc_depth[1] = 0.5*(float)size_depth.height;
  //depth camera focal length in pixel dimensions
  cam.fc_depth[0] = cam.cc_depth[0] / tan(0.5*FOV_DEPTH_CAMERA_HORIZ*pi/180.0);
  cam.fc_depth[1] = cam.cc_depth[1] / tan(0.5*FOV_DEPTH_CAMERA_VERT*pi/180.0);
}

//================================================================
//set the game area using the image of the area illuminated by the projector
void set_game_area(Point2f* pfloor, Mat imcolor, Mat imdepth, floor_parameters fp, camera_parameters cam){
  //select the four corners of the illuminated area
  Point2f p, pcolor[4];
  namedWindow("Set Game Area",1);
  setMouseCallback("Set Game Area", mouseHandler, &p);
  namedWindow("Set Game Area", WINDOW_AUTOSIZE);
  moveWindow("Set Game Area", 100, 100);

  imshow("Set Game Area", imcolor);

  //colors of the points
  Scalar color[] = {Scalar(128,0,0), Scalar(0,128,0), Scalar(0,0,128), Scalar(128,0,128)};

  for(int i=0; i<4; i++){
    while(p.x==0 && p.y==0)
      waitKey(10);
    circle(imcolor, p, 5, color[i], -1);
    imshow("Set Game Area", imcolor);
    pcolor[i] = p;
    p.x = 0;
    p.y = 0;
  }
  waitKey(1000);

  //map the selected points in the color image to the depth image
  color_depth_mapping(pfloor,pcolor, fp, cam);

  //show the mapped points in the depth image
  Mat imgray(imdepth.size(),CV_8U),  img(imdepth.size(),CV_8UC3);
  imdepth.convertTo(imgray,CV_8U,255.0/3000.0,0.0);
  cvtColor(imgray, img, CV_GRAY2RGB);
  for(int i=0; i<4; i++)
    circle(img, pfloor[i], 5, color[i], -1);
  namedWindow("Mapped points", WINDOW_AUTOSIZE);
  moveWindow("Mapped points", 100, 100);
  imshow("Mapped points", img);
  waitKey(0);
}

//=================================================================
//map the selected points in the color image to the depth image
void color_depth_mapping(Point2f* pfloor, Point2f* pcolor, floor_parameters fp, camera_parameters cam){
  //use downhill solver to optimize the mapping
  Ptr<DownhillSolver> solver = DownhillSolver::create();
  Ptr<MinProblemSolver::Function> ptr_F = makePtr<minMapping>();
  solver->setFunction(ptr_F);
  Mat step = (Mat_<double>(3,1)<<-0.5,-0.5,-0.5);
  solver->setInitStep(step);

  //copy variables to be used in error_mapping function
  fp_min = fp;
  cam_min = cam;

  int num_points = 4;
  for(int i=0; i<num_points; i++){
    pcolor_min = pcolor[i];
    //initial values
    float xfloor = cam.cc_depth[0];
    float yfloor = cam.cc_depth[1];
    float Zcolor = 2000.0;
    Mat x = (Mat_<double>(1,3) << xfloor, yfloor, Zcolor);
    solver->minimize(x);

    //update the values
    pfloor[i].x = (float)x.at<double>(0,0);
    pfloor[i].y = (float)x.at<double>(0,1);
  }
}

//==================================================================
//compute the error of mapping a point in the color image to the depth image
float error_mapping(Point2f pfloor, float Zcolor){
  //3D point coordinates in the color camera system
  float Zc = Zcolor;
  float Xc = Zc*(pcolor_min.x - cam_min.cc_color[0]) / cam_min.fc_color[0];
  float Yc = Zc*(pcolor_min.y - cam_min.cc_color[1]) / cam_min.fc_color[1];

  //3D point coordinates in the depth camera system
  Point3f D = floor_point(pfloor, fp_min, cam_min);
  //rotate the point to align it to the color camera system
  Mat P = (Mat_<float>(3,1) << D.x, D.y, D.z);
  Mat Q = ROTATION_DEPTH_TO_COLOR * P + TRANSLATION_DEPTH_TO_COLOR;

  //compute the error
  float error = sqrt(pow(Q.at<float>(0,0)-Xc,2.0) + pow(Q.at<float>(1,0)-Yc,2.0)
    + pow(Q.at<float>(2,0)-Zc,2.0));
  return error;
}

//==================================================================
//get a scale factor and an offset for the detected positions of the players
//to map them to the plot units used for the game
void scale_players(float* scale, Point2f* pfloor, floor_parameters fp, camera_parameters cam){
  //find the left and right corners corresponding to the area
  //illuminated by the proyector mapped to the depth image
  Point2f pleft[2],  pright[2];
  float xm = 0.25*(pfloor[0].x + pfloor[1].x + pfloor[2].x + pfloor[3].x);
  float ym = 0.25*(pfloor[0].y + pfloor[1].y + pfloor[2].y + pfloor[3].y);
  for(int i=0; i<4; i++){
    if( (pfloor[i].x < xm) && (pfloor[i].y < ym) ){ pleft[0] = pfloor[i]; }
    if( (pfloor[i].x < xm) && (pfloor[i].y > ym) ){ pleft[1] = pfloor[i]; }
    if( (pfloor[i].x > xm) && (pfloor[i].y < ym) ){ pright[0] = pfloor[i]; }
    if( (pfloor[i].x > xm) && (pfloor[i].y > ym) ){ pright[1] = pfloor[i]; }
  }

  //location of the corners on the floor in millimeters
  Point3f Dleft[2], Dright[2];
  for(int i=0; i<2; i++){
    Dleft[i] = floor_point(pleft[i], fp, cam);
    Dright[i] = floor_point(pright[i], fp, cam);
  }

  //compute the scale and offset for the rotated coordinates
  game_parameters g;
  float rx = fp.rotation_matrix.at<float>(1,0);
  float ry = fp.rotation_matrix.at<float>(1,1);
  float rz = fp.rotation_matrix.at<float>(1,2);
  float yl1 = rx*Dleft[0].x + ry*Dleft[0].y + rz*Dleft[0].z;
  float yl2 = rx*Dleft[1].x + ry*Dleft[1].y + rz*Dleft[1].z;
  float yr1 = rx*Dright[0].x + ry*Dright[0].y + rz*Dright[0].z;
  float yr2 = rx*Dright[1].x + ry*Dright[1].y + rz*Dright[1].z;
  scale[0] = g.plot_units[0] / (yl2 - yl1);
  scale[1] = -scale[0] * yl1;
  scale[2] = g.plot_units[0] / (yr2 - yr1);
  scale[3] = -scale[2] * yr1;
}

//=====================================================================
//get the 3D coordinates of a point on the floor from the pixel location
Point3f floor_point(Point2f pfloor, floor_parameters fp, camera_parameters cam){
  Point3f D;
  float nx = fp.floor_vector.at<float>(0);
  float ny = fp.floor_vector.at<float>(1);
  float nz = fp.floor_vector.at<float>(2);
  float d = fp.floor_depth;
  float fx = cam.fc_depth[0],  fy = cam.fc_depth[1];
  float cx = cam.cc_depth[0],  cy = cam.cc_depth[1];

  float ud = pfloor.x,  vd = pfloor.y;
  float a11 = fx*nz + (ud - cx)*nx;
  float a22 = fy*nz + (vd - cy)*ny;
  float a12 = (ud - cx)*ny;
  float a21 = (vd - cy)*nx;
  float b1 = (ud - cx)*d;
  float b2 = (vd - cy)*d;
  float detA = a11*a22 - a12*a21;
  if( abs(detA)>1e-6 ){
    D.x = (a22*b1 - a12*b2) / detA;
    D.y = (a11*b2 - a21*b1) / detA;
    D.z = (d - nx*D.x - ny*D.y)/nz;
  }
  return D;
}

//=================================================================
//function to handle mouse clicks
void mouseHandler(int event, int x, int y, int flags, void* ptr){
  if( event!=EVENT_LBUTTONDOWN )
    return;
  Point2f* p = (Point2f*)ptr;
  p->x = (float)x;
  p->y = (float)y;
}


