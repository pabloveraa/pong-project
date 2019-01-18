#include <math.h>
#include <stdlib.h>
#include <sys/timeb.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <libfreenect_sync.h>
#include "camera.h"

using namespace std;
using namespace cv;

//=====================================================================
//capture a depth image with the kinect
int capture_depth(Mat imdepth){
  int capture = 0;
  unsigned short* datadepth;
  unsigned int timestamp;
  if( freenect_sync_get_depth((void**)(&datadepth), &timestamp, 0, FREENECT_DEPTH_MM)==0 ){
    //copy  the captured depth data to an opencv image
    for(int v=0; v<imdepth.rows; v++){
      for(int u=0; u<imdepth.cols; u++){
        int ind = 4*v*imdepth.cols + 2*u;
        imdepth.at<float>(v,u) = (float)datadepth[ind];
      }
    }
    capture = 1;
  }
  return capture;
}

//=====================================================================
//capture a color image with the kinect
int capture_color(Mat imcolor){
  int capture = 0;
  char* datacolor;
  unsigned int timestamp;
  if( freenect_sync_get_video((void**)(&datacolor), &timestamp, 0, FREENECT_VIDEO_RGB)==0 ){
    //copy  the captured color data to an opencv image
    memcpy(imcolor.data, datacolor, sizeof(char)*imcolor.cols*imcolor.rows*3);
    capture = 1;
  }
  return capture;
}

//=====================================================================
//detect the floor, compute a rotation matrix to project the depth images
//to the floor and return the floor depth from the camera
float floor_detection(Mat imdepth, Mat rotation_matrix, Mat floor_vector){
  int cols = imdepth.cols,  rows = imdepth.rows;
  int num_points = cols*rows;
  //compute the 3D points
  Mat M(num_points, 3, CV_32F);
  get_3Dpoints(imdepth, M);

  //find the normal to the floor using RANSAC
  Mat P(3,3,CV_32F), C(1,3,CV_32F), Q(3,3,CV_32F);
  float floor_depth = 0;
  int tol = 10,  np_floor = 0;
  int num_iters = 100, iter = 0;
  while( iter<num_iters ){
    //select 3 random points
    int indz = 1;
    for(int i=0; i<3; i++){
      int ind = (rand() % num_points);
      for(int j=0; j<3; j++)
        P.at<float>(i,j) = M.at<float>(ind,j);
      if( M.at<float>(ind,3)==0 )
        indz = 0;
    }
    //check not zero values and not collinearity
    Mat S1 = P.col(1) - P.col(0),  S2 = P.col(2) - P.col(1);
    Mat R = (S1.t() * S2) / norm(S1,S2,NORM_L2);
    if( indz & (abs(R.at<float>(0,0)) < 0.9) ){
      C = (P.row(0) + P.row(1) + P.row(2)) / 3.0;
      for(int i=0; i<3; i++)
        Q.row(i) = P.row(i) - C;
      //normal to the plane where the three points lie
      Mat fvector;
      SVD::solveZ(Q, fvector);
      if( fvector.at<float>(2,0) < 0.0 )
        fvector = -fvector;
      //number of points lying at the plane
      Mat D = C * fvector;
      float d = D.at<float>(0,0);
      Mat F = M * fvector;
      int nfit = 0;
      for(int i=0; i<num_points; i++){
        if( abs(F.at<float>(i,0) - d)<=tol )
          nfit++;
      }
      if( nfit > np_floor ){
        fvector.copyTo(floor_vector);
        np_floor = nfit;
        floor_depth = d;
      }
      iter++;
    }
  }
  //find the rotation matrix to make the floor horizontal
  float data[] = {floor_vector.at<float>(0), 0.0, floor_vector.at<float>(1),
    0.0, floor_vector.at<float>(2), 1.0};
  Mat A(3,2,CV_32F,data);
  Mat U, S, Vt;
  SVD::compute(A,S,U,Vt,SVD::FULL_UV);
  Mat B = U.t() * A;
  float data1[] = {B.at<float>(0,0), -B.at<float>(1,0),
    B.at<float>(1,0), B.at<float>(0,0)};
  Mat R1(2,2,CV_32F,data1);
  float data2[] = {B.at<float>(0,1), -B.at<float>(1,1),
    B.at<float>(1,1), B.at<float>(0,1)};
  Mat R2(2,2,CV_32F,data2);
  Mat R3 = R2 * R1.t();
  float data3[] = {R3.at<float>(0,0), R3.at<float>(0,1), 0.0,
    R3.at<float>(1,0), R3.at<float>(1,1), 0.0, 0.0, 0.0, 1.0};
  Mat R(3,3,CV_32F,data3);
  rotation_matrix = U*R*U.t();
  return floor_depth;
}

//=======================================================================
//track the players in the depth image
void point_tracker(float* pLoc, Mat imdepth, Mat rotation_matrix, float floor_depth){
  //reduce the image size to speed up this process
  int cols = (int)(0.5*(float)imdepth.cols);
  int rows = (int)(0.5*(float)imdepth.rows);
  Size img_size = Size(cols,rows);
  Mat imd(img_size, CV_32F);
  resize(imdepth, imd, img_size, 0.0, 0.0, INTER_NEAREST);

  //read the 3D points and perform rotation to align them to the floor
  int num_points = cols * rows;
  Mat M(Size(3,num_points), CV_32F);
  get_3Dpoints(imd, M);
  Mat Zr(img_size, CV_32F),  Yr(img_size, CV_32F);
  rotate_points(Zr, M, rotation_matrix.row(2));
  rotate_points(Yr, M, rotation_matrix.row(1));

  //create a binary image highlighting objects above the floor
  float max_depth = floor_depth - 100;
  Mat imb = (Zr > 0.0) & (Zr <= max_depth);
  erode(imb, imb, Mat(), Point(-1,-1), 2);

//  imshow("binary image",imb);
//  waitKey(1);

  //find the mean values of the blobs at each side of the image
  Mat Um(img_size, CV_32F),  Vm(img_size, CV_32F);
  mesh_grid(Um, Vm);
  Mat maskLeft = (imb > 0) & (Um < (cols/2));
  Mat maskRight = (imb > 0) & (Um > (cols/2));
  Scalar xLeft = mean(Um, maskLeft),  yLeft = mean(Vm, maskLeft);
  Scalar xRight = mean(Um, maskRight),  yRight = mean(Vm, maskRight);

  //if the players were detected, refine their locations using mean shift algorithm
  if(xLeft.val[0] > 0.0  &&  xRight.val[0] > 0.0){
    float sigma = 30.0;  //bandwidth used for the mean shift
    float conv_thr = 1.0;  //threshold to determine convergence
    mean_shift(xLeft, yLeft, maskLeft, sigma, conv_thr);
    mean_shift(xRight, yRight, maskRight, sigma, conv_thr);
    pLoc[0] = Yr.at<float>((int)yLeft.val[0], (int)xLeft.val[0]);
    pLoc[1] = Yr.at<float>((int)yRight.val[0], (int)xRight.val[0]);
  }
  else{ pLoc[0] = 0.0;  pLoc[1] = 0.0; }
}

//================================================================
//locate the center of an object in a binary image using mean shift
void mean_shift(Scalar& x, Scalar& y, Mat mask, float sigma, float conv_thr){
  int cols = mask.cols,  rows = mask.rows;
  float dif = 1000.0;
  while( dif > conv_thr ){
    //set the search boudaries
    float xm = x.val[0],  ym = y.val[0];
    int x1 = (int)(xm-3*sigma),  x2 = (int)(xm+3*sigma);
    int y1 = (int)(ym-3*sigma),  y2 = (int)(ym+3*sigma);
    x1 = (x1 > 0)? x1 : 0;
    y1 = (y1 > 0)? y1 : 0;
    x2 = (x2 < cols)? x2 : cols-1;
    y2 = (y2 < rows)? y2 : rows-1;

    //perform a weigthing average
    float sumG = 0.0,  sumX = 0.0,  sumY = 0.0;
    float sigma2 = pow(sigma, 2.0);
    for(int u=x1; u<=x2; u++){
      for(int v=y1; v<=y2; v++){
        if( mask.at<unsigned char>(v,u) > 0 ){
          float r2 = pow((float)u-xm,2.0) + pow((float)v-ym,2.0);
          float G = exp(-0.5*r2 / sigma2);
          sumG += G;
          sumX += G * (float)u;
          sumY += G * (float)v;
        }
      }
    }

    //current displacement
    if( sumG>0 ){
      float dx = sumX/sumG - xm;
      float dy = sumY/sumG - ym;
      dif = sqrt( pow(dx,2.0) + pow(dy,2.0) );
      x.val[0] += dx;
      y.val[0] += dy;
    }
    else{ dif = 0; }
  }
}

//============================================================
//compute the coordinates of the 3D points from a depth image
void get_3Dpoints(Mat imdepth, Mat M){
  int cols = imdepth.cols,  rows = imdepth.rows;

  //get the camera focal length and principal point
  float fc[2], cc[2];
  float fov[] = {FOV_DEPTH_CAMERA_HORIZ, FOV_DEPTH_CAMERA_VERT};
  get_camera_parameters(fc, cc, fov, cols, rows);

  //compute the 3D coordinates of each image pixel
  for(int v=0; v<rows; v++){
    for(int u=0; u<cols; u++){
      float Z = imdepth.at<float>(v,u);
      float X = Z * ((float)u - cc[0])/fc[0];
      float Y = Z * ((float)v - cc[1])/fc[1];
      //store the values in a opencv matrix
      int k = v*cols + u;
      M.at<float>(k,0) = X;
      M.at<float>(k,1) = Y;
      M.at<float>(k,2) = Z;
    }
  }
}

//======================================================================
//get a rotated coordinate of the 3D points
void rotate_points(Mat R, Mat M, Mat R_row){
  for(int v=0; v<R.rows; v++){
    for(int u=0; u<R.cols; u++){
      int k = v*R.cols + u;
      R.at<float>(v,u) = R_row.at<float>(0,0)*M.at<float>(k,0) +
        R_row.at<float>(0,1)*M.at<float>(k,1) +
        R_row.at<float>(0,2)*M.at<float>(k,2);
    }
  }
}

//==============================================================
//compute the focal length and the principal point of a camera
void get_camera_parameters(float* focal_length, float* image_center, float* fov, int cols, int rows){
  float pi = 3.1416;
  //image center
  image_center[0] = 0.5*(float)cols;
  image_center[1] = 0.5*(float)rows;
  //focal length in pixel width dimension
  focal_length[0] = 0.5*(float)cols / tan(0.5*fov[0]*pi/180.0);
  focal_length[1] = 0.5*(float)rows / tan(0.5*fov[1]*pi/180.0);
}

//=================================================
//create a grid of 2D point coordinates
void mesh_grid(Mat Um, Mat Vm){
  for(int v=0; v<Um.rows; v++){
    for(int u=0; u<Um.cols; u++){
      Um.at<float>(v,u) = u;
      Vm.at<float>(v,u) = v;
    }
  }
}










