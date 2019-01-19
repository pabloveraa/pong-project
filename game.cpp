#include <stdlib.h>
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
  //estimated value of the vertical ball location
  //when it hit the right or left walls or one pallet
  float estY;

  //hit test right wall
  if( newX > (plotW - g.ball_radius) ){
    estY = (plotW-ball_loc[0]) * (newY-ball_loc[1])/(newX-ball_loc[0]) + ball_loc[1];
    if( estY < (goalB + g.ball_radius) || estY > (goalT - g.ball_radius) ){
      //hit right wall
      ball_speed[0] = -ball_speed[0];
      bounce(ball_speed, g);
    }
  }

  //hit test left wall
  else if( newX < g.ball_radius ){
    estY = -ball_loc[0] * (newY-ball_loc[1])/(newX-ball_loc[0]) + ball_loc[1];
    if( estY < (goalB + g.ball_radius) || estY > (goalT - g.ball_radius) ){
      //hit left wall
      ball_speed[0] = -ball_speed[0];
      bounce(ball_speed, g);
    }
  }

  //hit test top wall
  if( newY > (plotH - g.ball_radius) ){
    //hit top wall
    ball_speed[1] = -ball_speed[1];
    bounce(ball_speed, g);
  }

  //hit test bottom wall
  else if( newY < g.ball_radius ){
    //hit bottom wall
    ball_speed[1] = -ball_speed[1];
    bounce(ball_speed, g);
  }

  //hit test paddle 1
  estY = (paddle_loc[0]-ball_loc[0]) * (newY-ball_loc[1])/(newX-ball_loc[0]) + ball_loc[1];
  if( estY > (p1B - g.ball_radius) && estY < (p1T + g.ball_radius) ){
    //hit test paddle 1 from right
    if( ball_loc[0] > (p1R + g.ball_radius) && newX <= (p1R + g.ball_radius) ){
      ball_speed[0] = -ball_speed[0] * g.pfactor;
      bounce(ball_speed, g);
    }
    //hit test paddle 1 from left
    else if( ball_loc[0] < (p1L - g.ball_radius) && newX >= (p1L - g.ball_radius) ){
      ball_speed[0] = -ball_speed[0];
      bounce(ball_speed, g);
    }
  }

  //hit test paddle 2
  estY = (paddle_loc[2]-ball_loc[0]) * (newY-ball_loc[1])/(newX-ball_loc[0]) + ball_loc[1];
  if( estY > (p2B - g.ball_radius) && estY < (p2T + g.ball_radius) ){
    //hit test paddle 2 from right
    if( ball_loc[0] > (p2R + g.ball_radius) && newX <= (p2R + g.ball_radius) ){
      ball_speed[0] = -ball_speed[0];
      bounce(ball_speed, g);
    }
    //hit test paddle 2 from left
    else if( ball_loc[0] < (p2L - g.ball_radius) && newX >= (p2L - g.ball_radius) ){
      ball_speed[0] = -ball_speed[0] * g.pfactor;
      bounce(ball_speed, g);
    }
  }

  //move ball to new location
  ball_loc[0] += ball_speed[0];
  ball_loc[1] += ball_speed[1];
}


//=================================================================
//accelerates ball called by move_ball whenever ball hits something
void bounce(float* ball_speed, game_parameters g){
  //change ball speed by a random amount
  //helping ball move more horizontally than vertically
  ball_speed[0] *= g.bfactor;
  float angle = (float)(rand() % 360) * (3.1416 / 180.0);
  ball_speed[0] += g.yfactor * cos(angle);
  ball_speed[1] += g.yfactor * sin(angle);
  //bouncing accelerates ball
  float speed = (float)sqrt( pow(ball_speed[0],2) + pow(ball_speed[1],2) );
  float ballVecX = ball_speed[0] / speed;
  float ballVecY = ball_speed[1] / speed;
  speed += g.ball_acceleration;
  if( speed > g.max_ball_speed )
    speed = g.max_ball_speed;
  ball_speed[0] = speed * ballVecX;
  ball_speed[1] = speed * ballVecY;
}








