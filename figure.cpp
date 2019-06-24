#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <X11/Xlib.h>
#include "figure.h"
#include "game.h"

using namespace std;
using namespace cv;

//get screen resolution
Display* d = XOpenDisplay(NULL);
Screen* s = DefaultScreenOfDisplay( d );

//court size in Matlab plot units
game_parameters g;
float plotH = g.plot_units[1],  plotW = g.plot_units[0];
//ordintes of the goal borders
float topGoal = 0.5*(plotH + g.goal_size),  botGoal = 0.5*(plotH - g.goal_size);

//margins of the area outside the court
float marginHoriz = 0.2*plotW,  marginVert = 0.1*plotH;

//horizontal and vertical scales
float scaleHoriz = (float)s->width/(float)(plotW + 2*marginHoriz);
float scaleVert = (float)s->height/(float)(plotH + 2*marginVert);


//===========================================================================
//convert figure plot units to Opencv screen point location 
Point cnvPix(float pUnitW, float pUnitH){
  Point result = Point((int)(scaleHoriz*(pUnitW+marginHoriz)), (int)(scaleVert*(pUnitH+marginVert)));
  return result;
}


//==============================================================================
//create the figure for the court
void draw_court(Mat fig, bool set_background, figure_parameters p){
  //set the background area
  if( set_background ){
    Point axis_left = cnvPix(0, 0);
    Point axis_right = cnvPix(plotW, plotH);
    rectangle(fig, axis_left, axis_right, p.court_color, CV_FILLED);
  }

  //draw the horizontal court walls
  line(fig, cnvPix(0,0), cnvPix(plotW,0), p.wall_color, p.wall_width);
  line(fig, cnvPix(0,plotH), cnvPix(plotW,plotH), p.wall_color, p.wall_width);

  //draw the vertical court walls
  line(fig, cnvPix(0,topGoal), cnvPix(0,plotH), p.wall_color, p.wall_width);
  line(fig, cnvPix(0,botGoal), cnvPix(0,0), p.wall_color, p.wall_width);
  line(fig, cnvPix(plotW,topGoal), cnvPix(plotW,plotH), p.wall_color, p.wall_width);
  line(fig, cnvPix(plotW,botGoal), cnvPix(plotW,0), p.wall_color, p.wall_width);

  //draw the court lines
  line(fig, cnvPix(plotW/2,0), cnvPix(plotW/2,plotH), p.center_line_color, p.center_line_width);
  Size center_circle = Size((int)(p.center_radius*scaleHoriz),(int)(p.center_radius*scaleVert));
  ellipse(fig, cnvPix(0.5*plotW,0.5*plotH), center_circle, 0, 0, 360, p.center_line_color, p.center_line_width);
}


//=======================================================================================================
//draw the paddles and the ball
void draw_objects(Mat fig, float* paddle_loc, float* ball_loc, figure_parameters p, game_parameters g){
  float x, y;
  Point axis_left, axis_right, ball_center;
  //draw the left and right paddles
  for(int i=0; i<2; i++){
    x = paddle_loc[2*i] - 0.5*g.paddle_size[0];
    y = paddle_loc[2*i+1] - 0.5*g.paddle_size[1];
    axis_left = cnvPix(x, y);
    x = paddle_loc[2*i] + 0.5*g.paddle_size[0];
    y = paddle_loc[2*i+1] + 0.5*g.paddle_size[1];
    axis_right = cnvPix(x, y);
    rectangle(fig, axis_left, axis_right, p.paddle_color, CV_FILLED);
  }

  //draw ball
  ball_center =  cnvPix(ball_loc[0], ball_loc[1]);
  int radius_int = (int)(3.5 * g.ball_radius);
  int radius_ext = (int)(4.0 * g.ball_radius);
  circle(fig, ball_center, radius_ext, p.ball_outline, -1);
  circle(fig, ball_center, radius_int, p.ball_color, -1);
}



