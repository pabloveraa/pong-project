#include <stdlib.h>
#include <iostream>
#include <math.h>
#include "game.h"

using namespace std;

//set the paddles and ball to their initial conditions
void reset_objects(float* paddle_loc, float* ball_loc, float* ball_speed, game_parameters g){
  //height and width of the court in plot units
  float plotH = g.plot_units[1],  plotW = g.plot_units[0];
  paddle_loc[0] = g.paddle_shift;  //left paddle X
  paddle_loc[1] = 0.5*plotH;  //left paddle Y
  paddle_loc[2] = plotW - g.paddle_shift;  //right paddle X
  paddle_loc[3] = 0.5*plotH;  //right paddle Y
  ball_loc[0] = 0.5*plotW;  //ball X
  ball_loc[1] = 0.5*plotH;  //ball Y
  ball_speed[0] = 0.0;  //ball speed X
  ball_speed[1] = 0.0;  //ball speed Y
}


//===================================================================
//move the paddles according to the location of the players
void move_paddles(float* paddle_loc, float* pLoc, float* scale, game_parameters g){
  float paddleH = g.paddle_size[1];
  float plotH = g.plot_units[1];
  //new vertical locations of the left and right paddles
  for(int i=0; i<2; i++){
    //convert the vertical location of the player in millimeters to plot units
    float player_loc = scale[2*i]*pLoc[i] + scale[2*i+1];
    if( (player_loc - paddle_loc[2*i+1]) > g.thr_pad ){
      paddle_loc[2*i+1] += g.paddle_speed;
    }
    else if( (paddle_loc[2*i+1] - player_loc) > g.thr_pad ){
      paddle_loc[2*i+1] -= g.paddle_speed;
    }

    //check the game area limits
    if( (paddle_loc[2*i+1] + 0.5*paddleH) > plotH ){
      paddle_loc[2*i+1] = plotH - 0.5*paddleH;
    }
    else if( (paddle_loc[2*i+1] - 0.5*paddleH) < 0.0 ){
      paddle_loc[2*i+1] = 0.5*paddleH;
    }
  }
}


//===================================================================
//update the score and indicates if a goal was scored
int check_goal(int* score, float* ball_loc, game_parameters g){
  int goal_scored = 0;
  float plotW = g.plot_units[0];
  if( ball_loc[0] > (plotW + g.ball_radius + g.goal_buffer) ){
    score[0]++;
    goal_scored = 1;
  }
  else if( ball_loc[0] < (0 - g.ball_radius - g.goal_buffer) ){
    score[1]++;
    goal_scored = 1;
  }
  return goal_scored;
}


//=======================================================================================
//move the ball checking if it hit the paddles or the wall
void move_ball(float* ball_loc, float* ball_speed, float* paddle_loc, game_parameters g){
  //normalized vector for ball movement
  float speed = (float)sqrt( pow(ball_speed[0],2) + pow(ball_speed[1],2) );
  if( speed < g.min_ball_speed ){
    speed = g.min_ball_speed;
    float angle = (float)(rand() % 360) * (3.1416 / 180.0);
    ball_speed[0] = speed * cos(angle);
    ball_speed[1] = speed * sin(angle);
    bounce(ball_speed, g);
  }

  //paddle boundaries, useful for hit testing ball
  float p1Center[2] = { paddle_loc[0], paddle_loc[1] };
  float p1T = p1Center[1] + 0.5*g.paddle_size[1];
  float p1B = p1Center[1] - 0.5*g.paddle_size[1];
  float p1L = p1Center[0] - 0.5*g.paddle_size[0];
  float p1R = p1Center[0] + 0.5*g.paddle_size[0];
  float p2Center[2] = { paddle_loc[2], paddle_loc[3] };
  float p2T = p2Center[1] + 0.5*g.paddle_size[1];
  float p2B = p2Center[1] - 0.5*g.paddle_size[1];
  float p2L = p2Center[0] - 0.5*g.paddle_size[0];
  float p2R = p2Center[0] + 0.5*g.paddle_size[0];

  //temporary new ball location, only apply if ball doesn't hit anything
  float newX = ball_loc[0] + ball_speed[0];
  float newY = ball_loc[1] + ball_speed[1];
  float plotW = g.plot_units[0];
  float plotH = g.plot_units[1];
  float goalB = 0.5*(plotH - g.goal_size);
  float goalT = 0.5*(plotH + g.goal_size);

  //estimated values of the vertical ball location
  //when it hit the right or left walls or one pallet
  float mb = ball_speed[1] / ball_speed[0];
  float wYL = ball_loc[1] - ball_loc[0] * mb;
  float wYR = ball_loc[1] + (plotW - ball_loc[0]) * mb;
  float pY1L = ball_loc[1] + (p1L - ball_loc[0]) * mb;
  float pY1R = ball_loc[1] + (p1R - ball_loc[0]) * mb;
  float pY2L = ball_loc[1] + (p2L - ball_loc[0]) * mb;
  float pY2R = ball_loc[1] + (p2R - ball_loc[0]) * mb;

  float br = g.ball_radius;

  //hit right wall
  if( (newX > (plotW-br)) && ((wYR < (goalB+br)) || (wYR > (goalT-br))) ){
    ball_speed[0] = -ball_speed[0];
    bounce(ball_speed, g);
  }
  //hit left wall
  else if( (newX < br) && ((wYL < (goalB+br)) || (wYL > (goalT-br))) ){
    ball_speed[0] = -ball_speed[0];
    bounce(ball_speed, g);
  }
  //hit top wall
  else if( (newY > plotH) && (ball_loc[0] >= 0) && (ball_loc[0] <= plotW) ){
    ball_speed[1] = -ball_speed[1];
    bounce(ball_speed, g);
 }
  //hit bottom wall
  else if( (newY < 0) && (ball_loc[0] >= 0) && (ball_loc[0] <= plotW) ){
    ball_speed[1] = -ball_speed[1];
    bounce(ball_speed, g);
  }
  //hit paddle 1 from left
  else if( (ball_loc[0] < p1L) && (newX >= p1L) && (pY1L < (p1T+br)) && (pY1L > (p1B-br)) ){
    ball_speed[0] = -ball_speed[0];
    bounce(ball_speed, g);
  }
  //hit paddle 1 from right
  else if( (ball_loc[0] > p1R) && (newX <= p1R) && (pY1R < (p1T+br)) && (pY1R > (p1B-br)) ){
    ball_speed[0] = -ball_speed[0];
    bounce(ball_speed, g);
  }
  //hit paddle 2 from left
  else if( (ball_loc[0] < p2L) && (newX >= p2L) && (pY2L < (p2T+br)) && (pY2L > (p2B-br)) ){
    ball_speed[0] = -ball_speed[0];
    bounce(ball_speed, g);
  }
  //hit paddle 2 from right
  else if( (ball_loc[0] > p2R) && (newX <= p2R) && (pY2R < (p2T+br)) && (pY2R > (p2B-br)) ){
    ball_speed[0] = -ball_speed[0];
    bounce(ball_speed, g);
  }
  //move the paddles
  ball_loc[0] += ball_speed[0];
  ball_loc[1] += ball_speed[1];
}


//=================================================================
//accelerates ball called by move_ball whenever ball hits something
void bounce(float* ball_speed, game_parameters g){
  //change ball speed helping ball
  //move more horizontally than vertically
  float dx = (ball_speed[0] > 0)? g.ball_acceleration[0] : -g.ball_acceleration[0];
  float dy = (ball_speed[1] > 0)? g.ball_acceleration[1] : -g.ball_acceleration[1];
  ball_speed[0] += dx;
  ball_speed[1] += dy;

  //add a random component to the ball motion
  ball_speed[0] += (float)(rand() % (int)(2000.0*g.ball_random))/1000.0 - g.ball_random;
  ball_speed[1] += (float)(rand() % (int)(2000.0*g.ball_random))/1000.0 - g.ball_random;

  //check the maximum speed
  float speed = (float)sqrt( pow(ball_speed[0],2) + pow(ball_speed[1],2) );
  float ballVecX = ball_speed[0] / speed;
  float ballVecY = ball_speed[1] / speed;
  if( speed > g.max_ball_speed )
    speed = g.max_ball_speed;
  ball_speed[0] = speed * ballVecX;
  ball_speed[1] = speed * ballVecY;
}








