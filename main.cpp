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
#include "camera.h"
#include "game.h"
#include "calibration.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv){
  //parameters used by the program
  figure_parameters p;
  game_parameters g;

  //read calibration data
  floor_parameters fp;
  float scale[4];
  char fn[] = "calibration_data.txt";
  FILE* fid = fopen(fn, "r");
  if( fid!=NULL ){
    //read the floor parameters
    read_calibration_data(fid, fp, scale);
    fclose(fid);
  }
  else{
    cout << "No se encontró el archivo de calibración" << endl;
    cout << "Ejecute el programa de calibración" << endl;
    return 0;
  } 

  //get screen resolution
  Display* d = XOpenDisplay(NULL);
  Screen* s = DefaultScreenOfDisplay( d );

  //figure for the game
  Mat figBg(Size(s->width, s->height), CV_8UC3, p.figure_color);
  draw_court(figBg, true, p);
  Mat figGame;
  namedWindow(" ", WINDOW_AUTOSIZE);
  moveWindow(" ", 50, 10);

  while(true){
    //initialize score
    int score[2] = {0, 0};

    //set initial positions of the paddles and the ball
    float paddle_loc[4],  ball_loc[2],  ball_speed[2];
    reset_objects(paddle_loc, ball_loc, ball_speed, g);

    //locations of the players in pixel coordinates
    //which are updated using the mean shift algorithm
    float player_loc[4] = {0.0, 0.0, 0.0, 0.0};

    Mat imdepth(Size(320,240), CV_32F);  //depth image

    //show the figure with the paddles and the ball
    figBg.copyTo( figGame );
    draw_objects(figGame, paddle_loc, ball_loc, p, g);

    //Press a key to play or to exit
    char info[50];
    sprintf(info, "Enter => Juego Nuevo");
    putText(figGame, (string)info, Point((int)(0.2*s->width),(int)(0.15*s->height)), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,192,0), 1);
    sprintf(info, "ESC => Salir");
    putText(figGame, (string)info, Point((int)(0.2*s->width),(int)(0.18*s->height)), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,192,0), 1);
    imshow(" ", figGame);

    //press Enter
    int pKey = waitKey(0);
    if( pKey == 10 ){
      //play a game
      while( score[0]<g.max_score && score[1]<g.max_score ){
        if( capture_depth( imdepth ) ){
          float pLoc[2];  //vertical locations of the players in mm
          point_tracker(pLoc, imdepth, fp.rotation_matrix, fp.floor_depth, player_loc);

          //move the paddles according to the location of the players
          move_paddles(paddle_loc, pLoc, scale, g);

          //move the ball
          move_ball(ball_loc, ball_speed, paddle_loc, g);

          //redraw the paddles and the ball in the current locations
          figBg.copyTo( figGame );
          draw_objects(figGame, paddle_loc, ball_loc, p, g);

          //write the score
          char score_txt[20];
          sprintf(score_txt, "Marcador: %d - %d", score[0], score[1]);
          string score_str = score_txt;
          putText(figGame, score_str, Point((int)(0.4*s->width),(int)(0.05*s->height)), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0,128,128), 2);
          imshow(" ", figGame);
          waitKey(1);

          //update the score and reset the paddles and the ball if a goal was scored
          if( check_goal(score, ball_loc, g) ){
            reset_objects(paddle_loc, ball_loc, ball_speed, g);
            putText(figGame, "Gol!", Point((int)(0.47*s->width),(int)(0.5*s->height)), FONT_HERSHEY_SIMPLEX, 2.0, Scalar(0,128,128), 2);
            imshow(" ", figGame);
            waitKey(1);  
            sleep(3);
          }
        }
      }
    }
    //press ESC
    else if( pKey == 27 )
      break;
  }
  figGame.release();
  freenect_sync_stop();

  return 0;
}




