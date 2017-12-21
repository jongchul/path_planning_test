#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>
#include <iostream>

using namespace std;

const float REACH_GOAL = pow(10, 6);
const float EFFICIENCY = pow(10, 5);

//float goal_distance_cost(int goal_lane, int intended_lane, int final_lane, float distance_to_goal) {


//float goal_distance_cost(int goal_lane, int intended_lane, float distance_to_goal) {
    /*
    The cost increases with both the distance of intended lane from the goal
    and the distance of the final lane from the goal. The cost of being out of the 
    goal lane also becomes larger as vehicle approaches the goal.
    */
  //  int delta_d = 2.0*goal_lane - intended_lane - final_lane;
 //  int delta_d = 2.0*goal_lane - intended_lane;
 //   int delta_d = goal_lane - intended_lane;
  //  float cost = 1 - exp(-(abs(delta_d) / distance_to_goal));
 //   return cost;
//}




float d_diff_cost(vector<double> d_traj, int target_d){

    float cost;

    for(int i=0;i<d_traj.size();++i){

        /*
        cout << "target_d " << target_d << endl;
        cout << "d_traj check " << d_traj[i] << endl;
        */

        cost += fabs((target_d * 4 + 2)  -  d_traj[i]);

   //     cout << "cost " << cost << endl;

    }




  
  
    return cost;
}



float lane_change_cost(int intended_lane, int current_lane, int total_available_lane) {
    /*
    The cost increases with both the distance of intended lane from the goal
    and the distance of the final lane from the goal. The cost of being out of the 
    goal lane also becomes larger as vehicle approaches the goal.
    */
  //  int delta_d = 2.0*goal_lane - intended_lane - final_lane;
 //  int delta_d = 2.0*goal_lane - intended_lane;
    int delta_d = intended_lane - current_lane;
    float cost = 1 - exp(-(fabs(delta_d) / total_available_lane));
    return cost;
}


float leading_car_distance_cost(double closest_distance, double max_sensing_dist) {
   
    float delta_d = max_sensing_dist - closest_distance;
    float cost = 1 - exp(-(fabs(delta_d) / max_sensing_dist));
    return cost;
}

float leading_car_velocity_cost(double leading_car_velocity, double max_velocity) {
    /*
    The cost increases with both the distance of intended lane from the goal
    and the distance of the final lane from the goal. The cost of being out of the 
    goal lane also becomes larger as vehicle approaches the goal.
    */
    int delta_d = max_velocity - leading_car_velocity;
    float cost = 1 - exp(-(fabs(delta_d) / max_velocity ));
    return cost;
}








