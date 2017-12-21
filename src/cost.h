//https://discussions.udacity.com/t/checking-to-see-if-a-lane-change-is-safe/381989/2
//Ramesh-37

#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;


//float goal_distance_cost(int goal_lane, int intended_lane, int final_lane, float distance_to_goal); 

//float goal_distance_cost(int goal_lane, int intended_lane, float distance_to_goal); 

float lane_change_cost(int intended_lane, int current_lane, int total_available_lane); 

float leading_car_distance_cost(double closest_distance, double max_sensing_dist);

float leading_car_velocity_cost(double leading_car_velocity, double max_velocity);

float d_diff_cost(vector<double> d_traj,int target_d);

#endif