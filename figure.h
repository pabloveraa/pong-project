#ifndef FIGURE_H
#define FIGURE_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <X11/Xlib.h>
#include "game.h"

using namespace std;
using namespace cv;

struct figure_parameters{
  Scalar figure_color = Scalar(0, 0, 0);

  //wall parameters
  Scalar wall_color = Scalar(204, 77, 77);
  Scalar court_color = Scalar(40, 40, 40);
  int wall_width = 3;

  //ball parameters
  Scalar ball_color = Scalar(25, 178, 25);
  Scalar ball_outline = Scalar(178, 255, 178);

  //paddle parameters
  int paddle_line_width = 2;
  Scalar paddle_color = Scalar(0, 128, 255);

  //lines on court parameters
  float center_radius = 15;
  int center_line_width = 1;
  Scalar center_line_color = Scalar(0, 102, 204);
};

//create the figure for the court
void draw_court(Mat fig, bool set_background, figure_parameters p);

//draw the paddles and the ball
void draw_objects(Mat fig, float* paddle_loc, float* ball_loc, figure_parameters p, game_parameters g);

//convert figure plot units to Opencv screen point location
Point cnvPix(float pUnitW, float pUnitH);

#endif
