#ifndef VEHICLE
#define VEHICLE

#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  double s;
  double s_d;
  double s_dd;
  double d;
  double d_d;
  double d_dd;
  string state;
  vector<string> available_states;
  vector<double> s_traj_coeffs, d_traj_coeffs;

  /**
  * Constructors
  */
  Vehicle();
  Vehicle(double s, double s_d, double s_dd, double d, double d_d, double d_dd);

  /**
  * Destructor
  */
  virtual ~Vehicle();

 // vector<vector<double>> generate_predictions(double traj_start_time, double duration);
 vector<vector<double>> generate_predictions(double duration);

 void update_available_states(bool car_to_left, bool car_to_right);

 vector<vector<double>> get_target_for_state(string state, map<int, vector<vector<double>>> predictions, double duration, bool car_just_ahead); 

 vector<vector<double>> get_target_for_state(string state, map<int, vector<vector<double>>> predictions, double duration, 
    bool car_just_ahead, double ego_car_s, double ego_car_d); 

 vector<double> get_leading_vehicle_data_for_lane(int target_lane, map<int, vector<vector<double>>> predictions, double duration);


 vector<vector<double>> generate_best_traj(vector<vector<double>> target_s_and_d, double duration);

 vector<vector<double>> generate_traj_for_target(vector<vector<double>> target_s_and_d, double duration);

 double evaluate_coeffs(vector<double> jmt_result, double input);

 

};

#endif