#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "constant.h"
#include "cost.h"

#include <deque>

 

using namespace std;

// for convenience
using json = nlohmann::json;





// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

 // DEBUG
//  cout << "theta" << endl;
//  cout << theta << endl;

 // cout << "heading" << endl;
 // cout << heading << endl;


	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
// https://discussions.udacity.com/t/converting-from-x-y-to-frenet/347334/2

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];

	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //cout << "frenet_d : " << frenet_d << endl;
  /*
  cout<< "n_x : " << n_x << endl;
  cout<< "n_y : " << n_y << endl;
  cout<< "x_x : " << x_x << endl;
  cout<< "x_y : " << x_y << endl;
  cout<< "proj_norm : " << proj_norm << endl;
  cout<< "proj_x : " << proj_x << endl;
  cout<< "proj_y : " << proj_y << endl;
  */


	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

 // DEBUG
//  cout << "center_x" << endl;
//  cout << center_x << endl;
//  cout << "center_y" << endl;
//  cout << center_y << endl;
//  cout << "centerToPos" << endl;
//  cout << centerToPos << endl;
//  cout << "centerToRef" << endl;
//  cout << centerToRef << endl;

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);

 //   cout << "i " << i << "frenet_s " << frenet_s << endl;
	}

	frenet_s += distance(0,0,proj_x,proj_y);
 // cout << "frenet_s " << frenet_s << endl;

	return {frenet_s,frenet_d};

}


// Transform from Frenet s,d coordinates to Cartesian x,y
// current getXY function makes  (s to xy) conversion error on sharp turns.

vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	

 // cout << "getXY_is called " << endl;

  int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;

	}

	int wp2 = (prev_wp+1)%maps_x.size();


	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));

  // the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);
 
	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  //https://discussions.udacity.com/t/transformation-from-frenet-to-global-coordinates/387010/3

	double perp_heading = heading-pi()/2;

 // DEBUG
//  cout << "heading, perp_heading, s, d" << endl;
//  cout << heading << endl;
//  cout << perp_heading << endl;
//  cout << d << endl;
 // cout << s << endl;


	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

// debug
//  cout << "getXY() " << endl;
//  cout << x << endl;
//  cout << y << endl;

	return {x,y};

}







int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
 // double max_s = 6945.554;




  deque<int> aDeque;
//  aDeque.resize(3);

  aDeque.push_back(1);
  aDeque.push_back(1);
  aDeque.push_back(1);
  //aDeque.push_back(1);


  Vehicle ego_car = Vehicle();


  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  int lane = 1;
  double ref_vel = 0.0;

    

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,
    &map_waypoints_dy,&lane,&ref_vel, &ego_car, &aDeque](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];

          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            int prev_size = previous_path_x.size();

        //    cout << "car_d " << car_d << endl;

             // DEBUG

   //         cout << "prev_size" << endl;
  //          cout << prev_size << endl;

          // **************** DETERMINE EGO CAR PARAMETERS AND CONSTRUCT VEHICLE OBJECT ******************
          // Vehicle class requires s,s_d,s_dd,d,d_d,d_dd - in that order
          //  https://discussions.udacity.com/t/error-in-spline-h-on-second-set-of-x-y-vector-values/345081/41
         

            double duration = 1.0;
          
            double car_x1, car_y1, car_x2, car_y2, car_x3, car_y3, vx1, vy1, vx2, 
            vy2, s1, s2, s_dot, d1, d2, d_dot, d_dot_sec;


            if(prev_size > 0)
            {

            /* 
            car_x1 = previous_path_x[prev_size-1];
            car_y1 = previous_path_y[prev_size-1];

            car_x2 = previous_path_x[prev_size-2];
            car_y2 = previous_path_y[prev_size-2];

            car_x3 = previous_path_x[prev_size-3];
            car_y3 = previous_path_y[prev_size-3];

            vx1 = car_x1 - car_x2;
            vy1 = car_y1 - car_y2;

            vx2 = car_x2 - car_x3;
            vy2 = car_y2 - car_y3;

      //      https://discussions.udacity.com/t/calculation-of-speed-in-frenet-space/451483
            s1 = sqrt(pow(vx1, 2) + pow(vy1, 2));
            s2 = sqrt(pow(vx2, 2) + pow(vy2, 2));

            double heading = atan2(car_y1 - car_y2, car_x1 - car_x2);
            vector<double> sd_v1 = getFrenet(car_x1, car_y1, heading, map_waypoints_x,map_waypoints_y);
          
            double heading2 = atan2(car_y2 - car_y3, car_x2 - car_x3);
            vector<double> sd_v2 = getFrenet(car_x2, car_y2, heading2, map_waypoints_x,map_waypoints_y);
            
            d1 = sd_v1[1];
            d2 = sd_v2[1];

            cout << "d1 : " << d1 << endl;
            cout << "d2 : " << d2 << endl;

            d_dot = d1 - d2;
            s_dot = (sqrt(pow(vx1,2) + pow(vy1,2) - pow(d_dot,2)))*20;

            d_dot_sec =  d_dot * 50;

            ego_car.d = d1;
            ego_car.s = sd_v1[0];
            ego_car.s_d = s_dot;
            */
        //     ego_car.s_d = s_dot;
            
             /*
            ego_car.s = car_s;
            ego_car.d = car_d;
              */
          //  ego_car.s_d = 20;

            /*
            car_x1 = previous_path_x[2];
            car_y1 = previous_path_y[2];

            car_x2 = previous_path_x[1];
            car_y2 = previous_path_y[1];

            car_x3 = previous_path_x[0];
            car_y3 = previous_path_y[0];

            vx1 = car_x1 - car_x2;
            vy1 = car_y1 - car_y2;

            vx2 = car_x2 - car_x3;
            vy2 = car_y2 - car_y3;

            s1 = sqrt(pow(vx1, 2) + pow(vy1, 2));
            s2 = sqrt(pow(vx2, 2) + pow(vy2, 2));

            double heading = atan2(car_y1 - car_y2, car_x1 - car_x2);
            vector<double> sd_v1 = getFrenet(car_x1, car_y1, heading, map_waypoints_x,map_waypoints_y);
          
            double heading2 = atan2(car_y2 - car_y3, car_x2 - car_x3);
            vector<double> sd_v2 = getFrenet(car_x2, car_y2, heading2, map_waypoints_x,map_waypoints_y);
            
            d1 = sd_v1[1];
            d2 = sd_v2[1];

            cout << "d1 : " << d1 << endl;
            cout << "d2 : " << d2 << endl;

            d_dot = d1 - d2;
            s_dot = (sqrt(pow(vx1,2) + pow(vy1,2) - pow(d_dot,2)))*20;

            d_dot_sec =  d_dot * 50;

        //    ego_car.d = d1;
         //   ego_car.s = sd_v1[0];
        //    ego_car.s_d = s_dot;

          */

            ego_car.s = car_s;
            ego_car.d = car_d;
            ego_car.s_d  = car_speed;

         //   cout << "d_dot_sec : " <<  d_dot_sec << endl;
         //   cout << "s_dot : " << s_dot  << endl;


        //    cout << "ego_car d_d =======" << ego_car.d_d  << endl;





          } else
          {
            cout << "no previouse path" << endl;

        //    ego_car.s = car_s;
       //     ego_car.d = car_d;

            ego_car.s = car_s;
            ego_car.d = car_d;
            ego_car.s_d  = car_speed;

           
          }

            cout << "ego_car s =======" << ego_car.s << endl;
            cout << "ego_car.s_d =======" << ego_car.s_d  << endl;

            cout << "ego_car d =======" << ego_car.d << endl;
 
            // ********************* GENERATE PREDICTIONS FROM SENSOR FUSION DATA **************************

            vector<Vehicle> other_cars;
            map<int, vector<vector<double>>> predictions;


            for (auto sf: sensor_fusion) {
              double other_car_vel = sqrt(pow((double)sf[3], 2) + pow((double)sf[4], 2));
              Vehicle other_car = Vehicle(sf[5], other_car_vel, 0, sf[6], 0, 0);
              other_cars.push_back(other_car);
              int v_id = sf[0];

              duration = 1.0;

              vector<vector<double>> preds = other_car.generate_predictions(duration);
              predictions[v_id] = preds;

              // DEBUG, check ego_car.s and other vehicle s in the same line.

              /*
              double d_diff = other_car.d - car_d;

              if(d_diff < 2.0){d           
              
              double s_diff = fabs(other_car.s - car_s);
              double s_diff_ahead = other_car.s - car_s;
              double d_diff = other_car.d - car_d;
          
              cout << "ego_car s and d " << endl;
              cout << "ego_car.s " << ego_car.s << endl;
              cout << "ego_car.d " << ego_car.d << endl;
              
              if(d_diff < 2.0){
                cout << "d_diff is different " << endl;
                cout << "v_id " << v_id << endl;
                cout << "other_cars.s " << other_car.s << endl;
                cout << "s_diff " << s_diff << endl;
                cout << "s_diff_ext " << s_diff_ahead << endl;
                }
              }
              */
       
              }

              // DEBUG, check prediction value is correct on the second lane.. 

              /*
              for (auto prediction : predictions) {
   
              if(fabs(ego_car.d - prediction.second[0][1]) < 2.0){

                cout << "second " << prediction.second[0][1] << endl;

           //   if((prediction.second[0][1])/4 ==  1.0){
              
              cout << "predictions.first=========" <<  prediction.first << endl;

              cout << "predictions.second=========" << prediction.second[0][0] 
              << "  " << prediction.second[50][0] << " " << prediction.second[99][0] << endl;
               
               }
              }
            }

              */
            
            bool car_to_left = false, car_to_right = false, car_just_ahead = false;

          //   cout << "************* other_cars **********" << endl;
            
            for (Vehicle other_car: other_cars) {
       
              double s_diff = other_car.s - car_s;
       
              if (s_diff < FOLLOW_DISTANCE) {

                double d_diff = other_car.d - car_d;
                
                if (d_diff > 2 && d_diff < 6) {
                    car_to_right = true;
                  } else if (d_diff < -2 && d_diff > -6) {
                    car_to_left = true;
                  } else if (d_diff > -2 && d_diff < 2) {
                    car_just_ahead = true;
                  }
                
                }
              }

          // DEBUG
       //   if (car_to_right) cout << "CAR ON THE RIGHT!!!" << endl;
       //   if (car_to_left) cout << "CAR ON THE LEFT!!!" << endl;
       //   if (car_just_ahead) cout << "CAR JUST AHEAD!!!" << endl;

          ego_car.update_available_states(car_to_left, car_to_right);

          vector<vector<double>> final_traj; 

          double total_cost;
          double min_cost=9999;

          vector<string> egocar_state;
          egocar_state.push_back("KL");

          string prev_state;
          string last_state;

          // DEBUG. checking ego_car state is pushed to fixed sized queue correctly. 
          
          /*
          if(aDeque.size() < 1) {
          aDeque.push_back(1);
          aDeque.push_back(1);
          aDeque.push_back(1);
          }
          */

 //         double upsample1 = 1/3;
 //         double upsample2 = 2/3; 
          
          int end_lane;
          int target_lane;

          for (auto state:  ego_car.available_states) {

              double cost1;
              double cost2;
              double cost3;

              double d_cost;

              double distance_to_goal;

              double closest_car_distance=9999;
              double car_distance;

              double leading_car_sd;


        //      vector<vector<double>> target_s_and_d = ego_car.get_target_for_state(state, predictions, duration, car_just_ahead);
              vector<vector<double>> target_s_and_d = ego_car.get_target_for_state(state, predictions, 
                duration, car_just_ahead,ego_car.s, ego_car.d);

              //DEBUG. check target s d values are correct.

                
                cout << "============target_s : " << target_s_and_d[0][0] << endl;
                cout << "============target_s_d : " << target_s_and_d[0][1] << endl;
                cout << "============target_s_dd : " << target_s_and_d[0][2] << endl;

                cout << "***** target_d : " << target_s_and_d[1][0] << endl;
                cout << "***** target_d_d : " << target_s_and_d[1][1] << endl;
                cout << "***** target_d_dd : " << target_s_and_d[1][2] << endl;

                cout << "***** leading vehicle s : " << target_s_and_d[2][0] << endl;
                cout << "***** leading vehicle s_d : " << target_s_and_d[2][1] << endl;
                
              

              vector<vector<double>> possible_traj = ego_car.generate_traj_for_target(target_s_and_d, duration);

              // DEBUG. possible trajectory summary

              /*
                cout << " possible traj s summary"  << endl;
                cout << possible_traj[0][0] << endl;
                cout << possible_traj[0][50] << endl;
                cout << possible_traj[0][100] << endl;
                cout << possible_traj[0][149] << endl;

                cout << " possible traj d summary"  << endl;
                cout << possible_traj[1][0] << endl;
                cout << possible_traj[1][50] << endl;
                cout << possible_traj[1][100] << endl;
                cout << possible_traj[1][149] << endl;
                */

                distance_to_goal = GOLE_S - target_s_and_d[0][0];

                int intended_lane =  target_s_and_d[1][49];
                int current_lane = ego_car.d;
                double total_available_lane = 3.0;

                cost1 = lane_change_cost(intended_lane/4, current_lane/4, total_available_lane);

                
                // DEBUG. check lane_change_cost
                /*
                cout << "lane_chage_cost " << cost << endl;
                cout << intended_lane << endl;
                cout << intended_lane/4 << endl;

                cout << current_lane << endl;
                cout << current_lane/4 << endl;

                cout << total_available_lane << endl;
                  */
                
                car_distance = target_s_and_d[2][0] - ego_car.s;

                if(car_distance < closest_car_distance){

                  closest_car_distance = car_distance;
                }


                if(closest_car_distance > MAX_SENSING_DIST){
                  closest_car_distance = MAX_SENSING_DIST;
                }

                cost2=leading_car_distance_cost(closest_car_distance, MAX_SENSING_DIST);

                // below code is not implemented

                leading_car_sd = target_s_and_d[2][1];

                if(leading_car_sd == 0){
                  leading_car_sd = SPEED_LIMIT;
                }

                cost3=leading_car_velocity_cost(leading_car_sd, SPEED_LIMIT);
               
                d_cost = d_diff_cost(possible_traj[1],intended_lane/4);

                 // above code is not implemented

                cout << "ego_car.available_states=========== " << endl;
                cout << state << endl;  
         
                total_cost += cost2 * 5 + cost1 * 10;
  
                //DEBUG. lane change logic test.

                //   cout << "total_cost " << total_cost << endl;

                //DEBUG. aDeque test.
                    /*
                     for(int i=0;i<aDeque.size();++i){

                        cout << "aDeque element " << endl;
                         cout <<  aDeque[i] << endl;
                     }
                     */
                              
                if(min_cost > total_cost){

                   aDeque.pop_front();

              //     cout << "min_cost > total_cost and aDeque pop_front " << endl;
              //     cout << "@@@@@@@@@@@@@@@@@@@@@@@@@ " << endl;

                  min_cost = total_cost;
                  final_traj = possible_traj; 
 
                          if((state).compare("KL")==0){

                        //      cout << "aDeque push back 1 " << endl;
                           
                              aDeque.push_back(1);
                              ego_car.state = state;

                              } else if((state).compare("LCL")==0){

                       //       cout << "---------LLLLLLLLLLLLLLLLLLLLLLLL--------- " << endl;
                      //        cout << "aDeque push back 0 " << endl;
               
                              aDeque.push_back(0);
                              ego_car.state = state;

                              } else{

                    //        cout << "--------RRRRRRRRRRRRRRRRRRRRRRRR-------------- " << endl;  
                   //           cout << "aDeque push back 2 " << endl;
                  //            cout << "end_path_d  " << end_path_d << endl;
                    
                              aDeque.push_back(2);
                              ego_car.state = state;

                              }
   
                            
                }

                total_cost = 0; 

             }
            
              // DEBUG final trajectory summary     
            
              cout << "car status x y s d " << car_x << " , " << car_y <<
              " , " << car_s << " , " << car_d << endl;
              
              //DEBUG
              
               for(int i=0;i<final_traj[0].size();++i){

                cout << final_traj[0][i] << endl;
                cout << final_traj[1][i] << endl;

               }
                      
                //DEBUG end

            // DEBUG. check the distance between other cars and ego_car.

            bool too_close = false;

            if (ego_car.d >= 4 && ego_car.d < 8) {
                    lane = 1;
                  } else if (ego_car.d < 12 && ego_car.d >= 8) {
                    lane = 2;
                  } else if (ego_car.d >= 0 && ego_car.d < 4) {
                    lane = 0;
                  }


            for(int i=0; i < sensor_fusion.size(); i++)
            {

              float d = sensor_fusion[i][6];

              if(d < (2 + 4*lane + 2) && d >( 2 + 4*lane -2))
              {

                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];

                double check_speed = sqrt(vx*vx + vy * vy);
                double check_car_s = sensor_fusion[i][5];

                check_car_s += ((double)prev_size * 0.02 * check_speed);

                car_s = ego_car.s;

                double distance_cal = check_car_s - car_s;

            //    cout << "sf d" << d << endl;
            //    cout << "(check_car_s - car_s)" << distance_cal << endl;

                if((check_car_s > car_s) && ((check_car_s - car_s) < DISTANCE_BETWEEN_CARS))

                {

            //      cout << "***********too close ********* " << endl;
            //      cout << "lane " << lane << endl;

                  too_close = true;
              
                 
                }

              }

            }

              // DEBUG end. check the distance between other cars and ego_car.


            if(too_close)
            {
           
               ref_vel -= 0.4;

            } 
            else if(ref_vel < 47.5)
            {
           
              ref_vel += 0.4;
              
              if(ref_vel < 20){
                ref_vel += 2.4;
                // ref_vel += 1.4;
              }
 
            }           

            //****** check distance logic ends. *********** //
            //DEBUG. smooth waypoint generation, https://discussions.udacity.com/t/follow-lane-breakdown/313481/7
       
       //     vector<double> ptsx;
       //     vector<double> ptsy;



           // double ref_yaw = deg2rad(car_yaw);

            // DEBUG
            // cout << "prev_size" << prev_size << endl;



            double s_val, d_val;
            vector<double> final_s, final_d, final_x, final_y, final_dx, final_dy;

            vector<double> x_way, y_way, dx_way, dy_way, ss_way, d_way;

            //double test = (13-13)/13;

          
            double final_traj_size = final_traj[0].size();

             /*

            int sample_gen = 10;

            cout << "===================" << endl;

            for(int i=0;i<final_traj_size;i++){

                cout <<  final_traj[0][i] << "   " << flush;
              
            }

            cout << "===================" << endl;

            for(int i=0;i<final_traj_size;i++){

                cout << final_traj[1][i] << "   " << flush;;
            }

             cout << "===================" << endl;
             */


           // for(int i=1;i<=sample_num;i++){

            for(int i=0;i<final_traj_size;i++){

        //    s_val = final_traj[0][(final_traj_size-1)  - final_traj_size * (sample_num- i)/sample_num];
       //     d_val = final_traj[1][(final_traj_size-1)  - final_traj_size * (sample_num- i)/sample_num];

            s_val = final_traj[0][i];
            d_val = final_traj[1][i];

         //   cout << "final_traj s_val " << s_val << endl;

        //    cout << (final_traj_size -1) << endl;
        //    cout << final_traj_size * (sample_num- i)/sample_num << endl;
        //    cout << "d_value " << d_value << endl;

            int prev_wp_test = -1;

            if(s_val < ego_car.s){

              "****************** Big Big Big Big Error *******************";
            }

            while(s_val > map_waypoints_s[prev_wp_test+1] && (prev_wp_test < (int)(map_waypoints_s.size()-1) ))
            {
                prev_wp_test++;

           }

            int wp2_test = (prev_wp_test+1)%map_waypoints_s.size();

        //    cout << "maps_s behind " << map_waypoints_s[prev_wp_test] << endl;
       //    cout << "maps_s front " << map_waypoints_s[prev_wp_test+1] << endl;

            double gap = map_waypoints_s[prev_wp_test+1] - map_waypoints_s[prev_wp_test];

        //    cout << "maps_s front - maps_s behind " << gap <<  endl;

            double gap_car = s_val - map_waypoints_s[prev_wp_test];

       //     cout << "gap_car : " << gap_car <<  endl;

        //    cout << "car_s - maps_s behind " << gap_car <<  endl;

            double proj_s = gap_car/gap;

        //    cout << "proj_s " << proj_s <<  endl;
       //     cout << "final_traj s_val " << s_val + (gap * proj_s) << endl;

            double dx1, dx2, dy1, dy2, proj_dx_v, proj_dy_v;

            dx1 = map_waypoints_dx[prev_wp_test];
            dx2 = map_waypoints_dx[prev_wp_test+1];

            dy1 = map_waypoints_dy[prev_wp_test];
            dy2 = map_waypoints_dy[prev_wp_test+1];

            proj_dx_v = (dx2 * (1 - proj_s) + dx1 * proj_s)/2;

            proj_dy_v = (dy2 * (1 - proj_s) + dy1 * proj_s)/2;

          //  proj_dy_v = ((dy2 + dy1)/2) * proj_s;

         //   cout << " proj_dx_v, proj_dy_v : " << proj_dx_v << " , " << proj_dy_v << endl;
        //    cout << "waypoint d " << d_value << endl;
       //     cout << map_waypoints_y[prev_wp_test] << endl;

            double extimated_x = map_waypoints_x[prev_wp_test] 
            +  proj_s * (map_waypoints_x[prev_wp_test+1] - map_waypoints_x[prev_wp_test]);

            double extimated_y = map_waypoints_y[prev_wp_test] 
            +  proj_s * (map_waypoints_y[prev_wp_test+1] - map_waypoints_y[prev_wp_test]);

         //   cout << "estimated x : " << extimated_x << "  " << flush;
         //   cout << "estimated y : " << extimated_y << endl;

            x_way.push_back(extimated_x);
            y_way.push_back(extimated_y);
            dx_way.push_back(proj_dx_v);
            dy_way.push_back(proj_dy_v);
        //    ss_way.push_back(s_val + (gap * proj_s));

            ss_way.push_back(s_val);

            d_way.push_back(d_val);

          }

    //      cout << "ref_x " << ref_x << endl;
    //      cout << "ref_y " << ref_y << endl;
    //      cout << "ref_s " << ego_car.s << endl;

          //spline fit

          /*  
          vector<double> x_sway,y_sway,ss_sway;

          for(int i=0;i<x_way.size();i++){
            x_sway.push_back(x_way[i]-ref_x);
            y_sway.push_back(y_way[i]-ref_y);
            ss_sway.push_back(ss_way[i]-ego_car.s);

          }

          cout << "ss_sway summary : " << endl;
          for(int i=0;i<ss_sway.size();i++){
            cout << ss_sway[i] << endl;
          }

          
          cout << "summary: s, d, x, y, dx, dy " << endl;


          for(int i=0;i<x_sway.size();i++){
            cout << ss_sway[i] << " " << flush;
            cout << d_way[i] << " " << flush;
            cout << x_sway[i] << " " << flush;
            cout << y_sway[i] << " " << flush;
            cout << dx_way[i] << " " << flush;
            cout << dy_way[i] <<" " << endl;
          }
          */
          
          
          cout << "previous path summary " << endl;

          /*

          vector<double> prev_ss_way;
          vector<double> prev_x_way;
          vector<double> prev_y_way;

          double prev_temp_x, prev_temp_y, temp_x, temp_y,temp_heading;

          double prev_s_value;

          prev_s_value = car_s;
          */

          /*

          if(prev_size > 2) {

          prev_temp_x = previous_path_x[0];
          prev_temp_y = previous_path_y[0];

          temp_x = previous_path_x[1];
          temp_y = previous_path_y[1];

          temp_heading = atan2(prev_temp_y - temp_y, prev_temp_x - temp_x);

          vector<double> sd_vector = getFrenet(previous_path_x[1], previous_path_y[1], temp_heading, map_waypoints_x,map_waypoints_y);

          cout << "sd_vector s and d values : " << endl;
          cout << sd_vector[0] << endl;
          cout << sd_vector[1] << endl;

          prev_s_value = sd_vector[0];

         } else{

          cout << "prev_size <2 nothing happens " << endl;

           prev_s_value = car_s - 0.001;

         }

          cout << "prev_s_value : " << prev_s_value << endl;

          */


         for(int i=0;i<final_traj_size;i++){

        //    s_val = final_traj[0][(final_traj_size-1)  - final_traj_size * (sample_num- i)/sample_num];
       //     d_val = final_traj[1][(final_traj_size-1)  - final_traj_size * (sample_num- i)/sample_num];

            double s_val = final_traj[0][i];
            double d_val = final_traj[1][i];
            cout << "s value " << s_val << endl;
        }

        
          vector<double> final_s_way, final_x_way, final_y_way;


          double s_gap;

          tk::spline spline_x_s_test;
          tk::spline spline_y_s_test;


          double ref_x = car_x;
          double ref_y = car_y;
          double ref_s = car_s;

          cout << "============= previous path summary ===========" << endl;
          
          if(prev_size > 2) {

              ref_x = previous_path_x[0];
              ref_y = previous_path_y[0];


              for(int i=0;i<prev_size;i++){
                cout <<  previous_path_x[i] << "      " << flush;
                cout <<  previous_path_y[i] << "      " << flush;
                cout << "=============" << endl;
              }

          } else

          {
              ref_x = x_way[0];
              ref_y = y_way[0];
          }
           




          /*

          if(prev_size > 2){
          double slice_s = (final_traj[0][0]-prev_s_value)/prev_size;
            
            for(int i=0;i<prev_size;i++){
            final_s_way.push_back(slice_s * (i));
            final_x_way.push_back(previous_path_x[i]);
            final_y_way.push_back(previous_path_y[i]);
              }
       
          } 

          else {

           cout << "prev_size <2 push car data to input data vectors " << endl;
         
          double slice_s = (final_traj[0][0]-prev_s_value);

            final_s_way.push_back(prev_s_value);
            final_x_way.push_back(car_x);
            final_y_way.push_back(car_y);
          
          }

          */

        

          for(int i=0;i<ss_way.size();i++){
             final_s_way.push_back(ss_way[i]);
             final_x_way.push_back(x_way[i] );
             final_y_way.push_back(y_way[i] );
          }

          cout << "final input data summary =================" << endl;

          cout << "ref_s, ref_x ,ref_y : " << ref_s << "   " << ref_x << "   " <<
          ref_y << endl;

          cout << "ego_car.s, car_x ,car_y : " << ego_car.s << "   " << car_x << "   " <<
          car_y << endl;

          
          for(int i=0;i<final_s_way[i];i++){
          cout << final_s_way[i] << "  " << flush;
          cout << final_x_way[i] << "  " << flush;
          cout << final_y_way[i] << "  " << flush;    

          cout << "============" << endl;    
          
        }
        

          /*
         for(int i=0;i<final_s_way[i];i++){
            cout << final_s_way[i] -ref_s << ",   "<< flush;
         }

         cout << "=================" << endl;

          for(int i=0;i<final_s_way[i];i++){
            cout << final_x_way[i]  - ref_x << ",   "<< flush;
         }

          cout << "=================" << endl;

          for(int i=0;i<final_s_way[i];i++){
           cout << final_y_way[i] - ref_y << ",   "<< flush;
         }
         
          cout << "=================" << endl;
          */


          vector<double> input_s;
          vector<double> input_x;
          vector<double> input_y;

          for(int i=0;i<final_s_way.size();i++){
            input_s.push_back(final_s_way[i] -ref_s);
            input_x.push_back(final_x_way[i] -ref_x);
            input_y.push_back(final_y_way[i] -ref_y);
          }


         for(int i=0;i<input_s.size();i++){
            cout << input_s[i] << ",   "<< flush;
         }

         cout << "=================" << endl;

          for(int i=0;i<input_s.size();i++){
            cout << input_x[i] << ",   "<< flush;
         }

          cout << "=================" << endl;

         for(int i=0;i<input_s.size();i++){
           cout << input_y[i] << ",   "<< flush;
         }
         
          cout << "=================" << endl;



          vector<double> next_x_vals_final;
          vector<double> next_y_vals_final;


          tk::spline spline_x_s_final;
          tk::spline spline_y_s_final;

          spline_x_s_final.set_points(input_s,input_x);
          spline_y_s_final.set_points(input_s,input_y);

          double target_s = 1;

                    
          double progress = 0;        

          for(int i= 0; i < 50; i++){

                double tg_x, tg_y;

              //  progress = input_s[i];

           //     progress = (s_point +1)/(ref_vel*0.1);

            //    progress = (s_point +1);
             

            //     progress = s_point * (i);
          //      cout << "progress " << endl;
             //   progress = (target_s/ref_vel) * (i+1) * 4;
          //      cout << progress << endl;


            //    tg_x =  base_x  + spline_x_s(progress) +  d_way[progress]*(spline_dx_s(progress));
            //    tg_y = base_y + spline_y_s(progress) +  d_way[progress]*(spline_dy_s(progress));

            //    tg_x =  spline_x_s_final(progress);
            //    tg_y = spline_y_s_final(progress);

                tg_x =  spline_x_s_final(i);
                tg_y = spline_y_s_final(i);

                cout << "progress " << progress << endl;
                cout << "tg_x, tg_y " << tg_x << "  " << tg_y << endl;    
                next_x_vals_final.push_back(tg_x + ref_x);
                next_y_vals_final.push_back(tg_y + ref_y);
            }


            for(int i=0;i< next_x_vals_final.size();i++){
              cout <<  next_x_vals_final[i] << endl;
              cout <<  next_y_vals_final[i] << endl;
            }
                     

                
            json msgJson;

         
      //      msgJson["next_x"] = next_x_vals_test;
      //      msgJson["next_y"] = next_y_vals_test;


            msgJson["next_x"] = next_x_vals_final;
            msgJson["next_y"] = next_y_vals_final;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
