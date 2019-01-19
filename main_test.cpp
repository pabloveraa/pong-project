#include <stdio.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <libfreenect_sync.h>
#include <X11/Xlib.h>
#include <sys/timeb.h>
#include "figure.h"
#include "camera_test.h"
#include "game.h"
#include "calibration.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv){
  //parameters used by the program
  figure_parameters p;
  game_parameters g;

  //get screen resolution
  Display* d = XOpenDisplay(NULL);
  Screen* s = DefaultScreenOfDisplay( d );

  //capture a color image and a depth image to set the game area
  int nimg = 0;
  Mat imcolor(Size(640,480), CV_8UC3);
  while( capture_color(imcolor, nimg)==0 ){
    cout << "Failed to capture a color image!" << endl;
    freenect_sync_stop();
    sleep(4);
  }
  Mat imdepth(Size(320,240), CV_32F);
  while( capture_depth(imdepth, nimg)==0 ){
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

  //figure for the game
  Mat figBg(Size(s->width, s->height), CV_8UC3, p.figure_color);
  draw_court(figBg, true, p);
  Mat figGame;
  namedWindow(" ", WINDOW_AUTOSIZE);
  moveWindow(" ", 50, 10);

  //initialize score
  int score[2] = {0, 0};

  //set initial positions of the paddles and the ball
  float paddle_loc[4],  ball_loc[2],  ball_speed[2];
  reset_objects(paddle_loc, ball_loc, ball_speed, g);

  //show the figure with the paddles and the ball
  figBg.copyTo( figGame );
  draw_objects(figGame, paddle_loc, ball_loc, p, g);
  imshow(" ", figGame);
  waitKey(0);  //wait for the user to press a key

  //play a game
  while( score[0]<g.max_score && score[1]<g.max_score ){
    //take the initial time
    timeb tmb;
    ftime( &tmb );
    double t1 = (double)tmb.time + (double)tmb.millitm/1000;

    if( capture_depth( imdepth, nimg ) ){
      float pLoc[2];  //vertical locations of the players in mm
      point_tracker(pLoc, imdepth, fp.rotation_matrix, fp.floor_depth);

      //move the paddles according to the location of the players
      move_paddles(paddle_loc, pLoc, scale, g);

      //move the ball
      move_ball(ball_loc, ball_speed, paddle_loc, g);

      //redraw the paddles and the ball in the current locations
      figBg.copyTo( figGame );
      draw_objects(figGame, paddle_loc, ball_loc, p, g);

      //write the score
      char score_txt[20];
      sprintf(score_txt, "score: %d - %d", score[0], score[1]);
      string score_str = score_txt;
      putText(figGame, score_str, Point((int)(0.4*s->width),(int)(0.05*s->height)), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0,128,128), 2);
      imshow(" ", figGame);
      waitKey(1);

      //update the score and reset the paddles and the ball if a goal was scored
      if( check_goal(score, ball_loc, g) ){
        reset_objects(paddle_loc, ball_loc, ball_speed, g);
        putText(figGame, "Goal!", Point((int)(0.45*s->width),(int)(0.5*s->height)), FONT_HERSHEY_SIMPLEX, 2.0, Scalar(0,128,128), 2);
        imshow(" ", figGame);
        waitKey(1);  
        sleep(3);
      }
    }

    nimg = (nimg<283)? nimg + 1 : 0;

    //display the frame rate
    ftime( &tmb );
    double t2 = (double)tmb.time + (double)tmb.millitm/1000;
    double fps = 1.0/(t2-t1);
    cout << "fps = " << fps << endl;
  }

  figGame.release();
  freenect_sync_stop();

  return 0;
}




