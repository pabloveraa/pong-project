#ifndef GAME_H
#define GAME_H

using namespace std;

struct game_parameters{
  int max_score = 3;
  float plot_units[2] = {150, 100};
  float ball_radius = 2.5;
  float paddle_size[2] = {2, 12};
  float goal_size = 50;
  float paddle_shift = 10;
  float paddle_speed = 2;
  float thr_pad = 5;
  float min_ball_speed = 2;
  float max_ball_speed = 10;
  float ball_acceleration[2] = {0.1, 0.02};
  float ball_random = 0.05;
  float goal_buffer = 5;
};

//move the paddles according to the location of the players
void move_paddles(float* paddle_loc, float* pLoc, float* scale, game_parameters g);

//move the ball checking if it hit the paddles or the wall
void move_ball(float* ball_loc, float* ball_speed, float* paddle_loc, game_parameters g);

//update the score and indicates if a goal was scored
int check_goal(int* score, float* ball_loc, game_parameters g);

//accelerates ball called by move_ball whenever ball hits something
void bounce(float* ball_speed, game_parameters g);

//set the paddles and ball to their initial conditions
void reset_objects(float* paddle_loc, float* ball_loc, float* ball_speed, game_parameters g);


#endif


