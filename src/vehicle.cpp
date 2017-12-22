#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"
#include "constant.h"
#include "jmt.h"


/**
 * Initializes Vehicle
 */

Vehicle::Vehicle() {}

Vehicle::Vehicle(double s, double s_d, double s_dd, double d, double d_d, double d_dd) {

  this->s    = s;         // s position
  this->s_d  = s_d;       // s dot - velocity in s
  this->s_dd = s_dd;      // s dot-dot - acceleration in s
  this->d    = d;         // d position
  this->d_d  = d_d;       // d dot - velocity in d
  this->d_dd = d_dd;      // d dot-dot - acceleration in d
  state = "CS";

}

Vehicle::~Vehicle() {}


//vector<vector<double>> Vehicle::generate_predictions(double traj_start_time, double duration) {
vector<vector<double>> Vehicle::generate_predictions(double duration) {

  // Generates a list of predicted s and d positions for dummy constant-speed vehicles
  // Because ego car trajectory is considered from end of previous path, we should also consider the 
  // trajectories of other cars starting at that time.


  vector<vector<double>> predictions;

     for( int i = 0; i < N_SAMPLES * duration ; i++)
  {
 //   double t = traj_start_time + (i * duration/N_SAMPLES);
 //   double t = (i * duration/N_SAMPLES);
 //   double t = (i * 1/N_SAMPLES)*duration;
 //   double t = (i * 1/N_SAMPLES);
    double t = 1.0/N_SAMPLES;

 //   double new_s_base = this->s;

    double new_s = this->s + this->s_d * t * i;
  
  //  DEBUG. check trajectory prediction 
  /* 
    cout << "i " << i << endl;
    cout << "t " << t << endl;


    cout << "new_s " << "s " << s << ", s_d" << s_d  << ", t*i " << double(t*i) <<
    ", new_s" << new_s << endl;
  */

    vector<double> s_and_d = {new_s, this->d};
    predictions.push_back(s_and_d);
  }

  /*
  cout << "predictions =============" << endl;

  for (auto pred: predictions) {
					// 	cout << "(" << sf[0] << ": " << sf[1] << "," << sf[2] << "," << sf[3] << "," << sf[4] << "," << sf[5] << "," << sf[6] << ") (" << distance(pos_x, pos_y, sf[1], sf[2]) << ")" << endl;
					// }
  		cout << pred[0] << endl;
  	}
  */
	

  return predictions;
}

//void Vehicle::update_available_states(bool car_to_left, bool car_to_right, double car_d) {

void Vehicle::update_available_states(bool car_to_left, bool car_to_right) {
  /*  Updates the available "states" based on the current state:
  "KL" - Keep Lane
   - The vehicle will attempt to drive its target speed, unless there is 
     traffic in front of it, in which case it will slow down.
  "LCL" or "LCR" - Lane Change Left / Right
   - The vehicle will change lanes and then follow longitudinal
     behavior for the "KL" state in the new lane. */

  this->available_states = {"KL"};
  if (this->d > 4 && !car_to_left) {
   this->available_states.push_back("LCL");
  }
  if (this->d < 8 && !car_to_right) {
    this->available_states.push_back("LCR");
 }


}

//vector<vector<double>> Vehicle::get_target_for_state(string state, map<int, vector<vector<double>>> predictions, double duration, bool car_just_ahead) {

vector<vector<double>> Vehicle::get_target_for_state(string state, map<int, vector<vector<double>>> predictions
  ,double duration, bool car_just_ahead, double ego_car_s , double ego_car_d) {
  // Returns two lists s_target and d_target in a single vector - s_target includes 
  // [s, s_dot, and s_ddot] and d_target includes the same
  // If no leading car found target lane, ego car will make up PERCENT_V_DIFF_TO_MAKE_UP of the difference
  // between current velocity and target velocity. If leading car is found set target s to FOLLOW_DISTANCE
  // and target s_dot to leading car's s_dot based on predictions
  
 // int target_lane, current_lane = this->d / 4; 

  int target_lane, current_lane = abs(this->d / 4); 

  double target_d; 
  // **** TARGETS ****
  // lateral displacement : depends on state
  // lateral velocity : 0
  double target_d_d = 0;
  // lateral acceleration : 0
  double target_d_dd = 0;

  // longitudinal velocity : current velocity + max allowed accel * duration
 // double target_s_d = min(this->s_d + MAX_INSTANTANEOUS_ACCEL/2 * duration, SPEED_LIMIT);

  double target_s_d = min(this->s_d + MAX_INSTANTANEOUS_ACCEL/2, SPEED_LIMIT);


  // DEBUG 
  /*
  cout << "******* s_d + MAX_INSTANTANEOUS_ACCEL/2 *******" << endl;
  cout << s_d + MAX_INSTANTANEOUS_ACCEL/2 << endl;

  cout << "******* target_s_d *******" << endl;
  cout << "target_s_d "  << target_s_d << endl;

  cout << "******** SPEED_LIMIT ******" << endl;
  cout << SPEED_LIMIT << endl;
  */
  
  

//   double target_s_d = min(this->s_d + MAX_INSTANTANEOUS_ACCEL/2, SPEED_LIMIT);
 // target_s_d = SPEED_LIMIT;  
  // longitudinal acceleration : zero ?
  double target_s_dd = 0;
  // longitudinal acceleration : difference between current/target velocity over trajectory duration?
  //double target_s_dd = (target_s_d - this->s_d) / (N_SAMPLES * DT);
  // longitudinal displacement : current displacement plus difference in current/target velocity times 
  // trajectory duration
  double target_s = this->s + (this->s_d + target_s_d) / 2 * duration;
  //double target_s = this->s + (this->s_d + target_s_d) / 2;
  
  /*
  cout << "target_s, " << target_s << endl;
  cout << "this->s " << this->s << endl;
  cout << "this->s_d " << this->s_d << endl;
  cout << "target_s_d " << target_s_d << endl;
  cout << "duration " << duration << endl;
  */
  
  vector<double> leading_vehicle_s_and_sdot;

 // if(this->d > 0 && this-> d < 12) {

  if(state.compare("KL") == 0)
  {
    target_d = (double)current_lane * 4 + 2;
    target_lane = target_d / 4;
  }
  else if(state.compare("LCL") == 0)
  {
    target_d = ((double)current_lane - 1) * 4 + 2;
    target_lane = target_d / 4;
  }
  else if(state.compare("LCR") == 0)
  {
    target_d = ((double)current_lane + 1) * 4 + 2;
    target_lane = target_d / 4;
  }

//  } else {
 //   target_d = 6;
 //   target_lane = 1;
 // } 

  // replace target_s variables if there is a leading vehicle close enough
  leading_vehicle_s_and_sdot = get_leading_vehicle_data_for_lane(target_lane, predictions, duration);
  
  /*
 cout << "target_d =========== " << target_d << ",      target_lane =========== " << target_lane << endl; 
 cout << "leading_vehicle_s_and_sdot  =========== " << endl;
 cout << "leading_vehicle_s =========== " << endl;
 cout << leading_vehicle_s_and_sdot[0] << endl;

 cout << "leading_vehicle_sdot =========== " << endl;
 cout << leading_vehicle_s_and_sdot[1] << endl;

 */
 
 
 
  double leading_vehicle_s = leading_vehicle_s_and_sdot[0];
  if (leading_vehicle_s - target_s < FOLLOW_DISTANCE && leading_vehicle_s > this->s + 20 ) {

    target_s_d = leading_vehicle_s_and_sdot[1];

    if (fabs(leading_vehicle_s - target_s) < 0.5 * FOLLOW_DISTANCE) {
      //cout << "TOO CLOSE IN LANE " << target_lane << "!! current target speed: " << target_s_d;
      target_s_d -= 1; // slow down if too close
      //cout << "  new target speed: " << target_s_d << endl;
    }

    target_s = leading_vehicle_s - FOLLOW_DISTANCE;
    // target acceleration = difference between start/end velocities over time duration? or just zero?
    //target_s_dd = (target_s_d - this->s_d) / (N_SAMPLES * DT);

    /*
    cout << "target_s ==========" << target_s << endl;
    cout << "target_s_d ==========" << target_s_d << endl;
    */

    /*



    // // DEBUG
    
    cout << "NEARBY LEAD VEHICLE DETECTED!  ";
    cout << "s: " << leading_vehicle_s_and_sdot[0]
        << ", lane: " << target_lane 
        << ", speed: " << leading_vehicle_s_and_sdot[1] << endl;

   cout << "ego car s " << this -> s << endl;
   cout << "distance between cars " << (leading_vehicle_s_and_sdot[0]) - (this -> s) << endl;
    */
  }
  

  // emergency brake
  /*
  if (car_just_ahead) {
    target_s_d = 0.0;
  }
  */

//  return {{target_s, target_s_d, target_s_dd}, {target_d, target_d_d, target_d_dd}};

  return {{target_s, target_s_d, target_s_dd}, {target_d, target_d_d, target_d_dd},{leading_vehicle_s_and_sdot}};
}

vector<double> Vehicle::get_leading_vehicle_data_for_lane(int target_lane, map<int, vector<vector<double>>> predictions, double duration) {
  // returns s and s_dot for the nearest (ahead) vehicle in target lane
  // this assumes the dummy vehicle will keep its lane and velocity, it will return the end position
  // and velocity (based on difference between last two positions)

  double nearest_leading_vehicle_speed = 0, nearest_leading_vehicle_distance = 99999;

  for (auto prediction : predictions) {

    vector<vector<double>> pred_traj = prediction.second;
    int pred_lane = pred_traj[0][1] / 4;
    if (pred_lane == target_lane) {

      double start_s = pred_traj[0][0];
      double predicted_end_s = pred_traj[pred_traj.size()-1][0];
      double next_to_last_s = pred_traj[pred_traj.size()-2][0];
//      double dt = duration / N_SAMPLES;
  //     double dt = duration * N_SAMPLES;
  //    double predicted_s_dot = (predicted_end_s - next_to_last_s) / dt;
      double predicted_s_dot = (predicted_end_s - next_to_last_s) * N_SAMPLES ;
      if (predicted_end_s < nearest_leading_vehicle_distance && start_s > this->s) {
        nearest_leading_vehicle_distance = predicted_end_s;
        nearest_leading_vehicle_speed = predicted_s_dot;
      }
    }
  
  }

  return {nearest_leading_vehicle_distance, nearest_leading_vehicle_speed};
}


vector<vector<double>> Vehicle::generate_best_traj(vector<vector<double>> target_s_and_d, double duration) {

  double sec = 60;

  this -> s;

  target_s_and_d[0][0];

  double s_t = ((target_s_and_d[0][0]) - (this->s))/(duration * sec);

  double d_t = ((target_s_and_d[1][0]) - (this->d))/(duration * sec);

  double s_d_t = ((target_s_and_d[0][1]) - (this->s_d))/(duration * sec);

  double s_dd_t = ((target_s_and_d[0][2]) - (this->s_dd))/(duration * sec);

  cout << "ego_car.s, s_d, s_dd : " << s << " " << s_d << "  " << s_dd << endl;

  cout << " ,  target s, s_d, s_dd : " <<  target_s_and_d[0][0] << "  " <<
  target_s_and_d[0][1] << "  " <<  target_s_and_d[0][2] << endl;

  cout << "s_t should be 'travel distance / 20' : " << s_t << endl;
  cout << "d_t should be 'travel distance / 20' : " << d_t << endl;

 // cout << "s_d_t should be 'travel distance / 60' : " << s_d_t << endl;
 // cout << "s_dd_t should be 'travel distance / 60' : " << s_dd_t << endl;


  vector<double> s_trajectory;
  vector<double> d_trajectory;

  double base_s = this -> s;
  double base_d = this -> d;

  for(int i=0;i<sec;i++){

    s_trajectory.push_back(base_s + s_t * i);
    d_trajectory.push_back(base_d + d_t * i);

  }

  
   vector<vector <double>> s_d_trajectory = {
      s_trajectory,
        d_trajectory
    };


    return s_d_trajectory;

}

vector<vector<double>> Vehicle::generate_traj_for_target(vector<vector<double>> target_s_and_d, double duration) {

//void Vehicle::generate_traj_for_target(vector<vector<double>> target_s_and_d, double duration) {
	
  /*
  cout << "generate_traj_for_target" << endl;


  cout << "target s " << target_s_and_d[0][0] << endl;
  cout << "target sd " << target_s_and_d[0][1]<< endl;
  cout << "target sdd " << target_s_and_d[0][2] << endl;

  */

//  cout << "target d " << target_s_and_d[1][0] << endl;
 // cout << "target dd " << target_s_and_d[1][1] << endl;
 // cout << "target ddd " << target_s_and_d[1][2]<< endl;
  
 


   vector<double> s_coeff;
   vector<double> d_coeff;

   vector<double> s_start;
   vector<double> s_end;

   vector<double> d_start;
   vector<double> d_end;
//   double T;

  
   
   s_start = {this -> s, this -> s_d, this -> s_dd};
   s_end = {target_s_and_d[0][0],target_s_and_d[0][1],target_s_and_d[0][2]};

   s_coeff = JMT(s_start, s_end, 1);

   d_start = {this -> d, this -> d_d, this -> d_dd};
   d_end = {target_s_and_d[1][0],target_s_and_d[1][1],target_s_and_d[1][2]};

   d_coeff = JMT(d_start, d_end, 1);

   // α3,  α4,  α5 values 

    /*
    for(auto s:s_start){
                cout << "start " << s << endl;
              }

    for(auto e:s_end){
                cout << "end " << e << endl;
              }

    for(auto s:s_coeff){
                cout << "s_coeff " << s << endl;
              }    

    for(auto d:d_coeff){
                cout << "d_coeff " << d << endl;
              }         
    */


   vector<double> s_trajectory;
   vector<double> d_trajectory;
   double s_base = target_s_and_d[0][0];
   double temp;
   double temp2;

   /*
    for(int i=0;i<N_SAMPLES;++i){

  //    cout << "i/N_SAMPLES " << i/N_SAMPLES  << endl;
      temp = evaluate_coeffs(s_coeff, i/double(N_SAMPLES));
      

      s_trajectory.push_back(temp);

      temp2 = evaluate_coeffs(d_coeff, i/double(N_SAMPLES));

   //   temp2  += calculate_poly(d_coeff, i/double(N_SAMPLES));
    //  if((i %10) == 0) {
    
      d_trajectory.push_back(temp2);
      */

  //    for(int i=0;i<N_SAMPLES;++i){

      int NO_WAYPOINT = 10;

   //   for(int i=0;i<N_SAMPLES*duration;++i){
      for(int i=0;i<NO_WAYPOINT;++i){
//
  //    cout << "i/N_SAMPLES " << i/N_SAMPLES  << endl;
   //   temp = evaluate_coeffs(s_coeff, i/double(N_SAMPLES));
   //   temp = evaluate_coeffs(s_coeff, i/double(N_SAMPLES*duration));
      temp = evaluate_coeffs(s_coeff, i/double(NO_WAYPOINT));

  //    cout << "s_coeff temp " << temp << endl;

      s_trajectory.push_back(temp);

  //    temp2 = evaluate_coeffs(d_coeff, i/double(N_SAMPLES));
   //   temp2 = evaluate_coeffs(d_coeff, i/double(N_SAMPLES*duration));
       temp2 = evaluate_coeffs(d_coeff, i/double(NO_WAYPOINT));
      
  //    cout << "d_coeff temp2 " << temp2 << endl;

   //   temp2  += calculate_poly(d_coeff, i/double(N_SAMPLES));
    //  if((i %10) == 0) {
    
      d_trajectory.push_back(temp2);

      /*
      if((i %10) == 0) {
      cout << " temp " << temp << endl;
      cout << " temp2 " << temp2 << endl;
      }
      */

      /*
      s_trajectory.push_back(s_base);
     
      temp = s_coeff * (duration/(i/N_SAMPLES));

      s_base += temp;
      */

    }

    vector<vector <double>> s_d_trajectory = {
      s_trajectory,
        d_trajectory
    };


    return s_d_trajectory;
}

double Vehicle::evaluate_coeffs(vector<double> jmt_result, double input){

     double result;

 //    cout << "input " << input << endl;

//     cout << "jmt_result[0] " << jmt_result[0] << endl;
//     cout << "jmt_result[1] " << jmt_result[1] << endl;
 //    cout << "jmt_result[2] " << jmt_result[2] << endl;
//     cout << "jmt_result[3] " << jmt_result[3] << endl;
 //    cout << "jmt_result[4] " << jmt_result[4] << endl;
 //    cout << "jmt_result[5] " << jmt_result[5] << endl;


     result = 
     jmt_result[0] * pow(input,0) + 
     jmt_result[1] * pow(input,1) + 
     jmt_result[2] * pow(input,2) +
     jmt_result[3] * pow(input,3) +
     jmt_result[4]* pow(input,4) +
     jmt_result[5] * pow(input,5);
    

 //    cout << "result " << result << endl;

     return result;
}


