#include <uWS/uWS.h>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include <math.h>
#include "spline.h"
using namespace std;

#define LANE_WIDTH 4
// for convenience
using nlohmann::json;
using std::string;
using std::vector;

#define LEFT_LANE 0    
#define MIDDLE_LANE 1
#define RIGHT_LANE 2




// MSelim: Define Functions in this area

/*
Some Conventions in naming variables:
Car --> refers to agent car.
Cars --> refers to other vehicles.

*/


// to do define states RL, LF, ML


/*
goal: Make a prediction model from the sensor fusion data
service: reads data and compute its distance in the future given its velocities
returns: vector of doubles containing all cars_s .
*/
vector<double> make_predictions(vector<vector<double>>sensor_fusion, int prev_path_size)
{
	vector<double>other_cars_s_predictions;
	for(auto cars_readings : sensor_fusion)
	{
    // everything in here represesnt other cars.

		double vx = cars_readings[3]; // reads x,y compoenent of the velocity
		double vy = cars_readings[4];

		double cars_speed = sqrt(vx*vx + vy*vy);
		double cars_s = cars_readings[5];


		cars_s += ((double)prev_path_size * 0.02 * cars_speed); // where the car would be given its velocity
		other_cars_s_predictions.push_back(cars_s);
	}
	return other_cars_s_predictions;
}






/*
goal: cost function
service: give a cost=1 when lane is unsafe, when a car is ahead or behind by 30m.
returns: 1 or 0, punish or reward.
*/

bool distance_cost_is_near(vector<vector<double>>sensor_fusion,vector<double> other_cars_s_predictions,double car_s,int lane)
{

int current_car_lane = 2+4*lane;
for( long i =  0; i < sensor_fusion.size(); i++){
  
  float cars_d = sensor_fusion[i][6];
  
  // Check that car is in the Agent lane
  if(cars_d < current_car_lane + 2 && cars_d > current_car_lane - 2) {
  
      // if distance between the agent and another car becomes < 35
       if(abs(other_cars_s_predictions[i] - car_s) < 35){

         return true;
       }

  }
}

return false;
 
}


/*

FSM : There are 3 states
0: Left Lane
1: Middle Lane
2: Right Lane

goal: to make a State function
service: compute all the possible state for a car given that the car is at  some state.
returns: vector of ints representing all states that the car can go given some state where the car is at.
*/

vector<int>get_states(int current_lane_state){

  vector <int> all_states;

  if(current_lane_state==RIGHT_LANE) 
  {
    //right --> middle
    all_states.push_back(MIDDLE_LANE); // note: lane 0 is not considered since 2 sudden lane changes may lead to crash.
  
    
   } 
  
  else if(current_lane_state == MIDDLE_LANE)
  {
    //There are two possible states if agent is in middle lane
    //middle --> left or right
    all_states.push_back(LEFT_LANE);
    all_states.push_back(RIGHT_LANE);

  }

  else { 
    //left --> middle
    all_states.push_back(MIDDLE_LANE);
    }

return all_states;
}





/**
goal:  choose_ next_state function
service: function that get the next_state which specifies the best next lane_state
returns: the best_state that the car should follow.
**/

int choose_next_state(vector<vector<double>>sensor_fusion,double car_s,int lane,int prev_path_size)
{
  
	  //vector<int> available_states =get_states(lane);
    vector<double> predictions = make_predictions(sensor_fusion,prev_path_size);
    // 1st Priority : Check for near car in current lane
    // 2nd Priority : Check for near car in left lane
    // 3rd Priority : Check for near car in right lane
    if (distance_cost_is_near(sensor_fusion,predictions,car_s,lane)) 
    {
							int check_left_lane = lane - 1; 
							int check_right_lane = lane + 1; 
							bool lane_switch = false;
							cout << "Car is Near the Agent: " << lane << ", Finding Possible Lane Change." << endl;
				      // Safety Check that Agent is still in the road (not negative)
							if (check_left_lane >= 0) {
								lane_switch = !distance_cost_is_near(sensor_fusion,predictions,car_s,check_left_lane);
                // no car is near in left lane so move left
								if(lane_switch)
								{
									lane = check_left_lane;
									cout << "Agent Moved To Left Lane.  " << check_left_lane << endl;
								}
							}

							if(check_right_lane <= 2 && !lane_switch) {
                // no car is near in right lane so move right
								lane_switch = !distance_cost_is_near(sensor_fusion,predictions,car_s,check_right_lane); 
								if(lane_switch) {
									lane = check_right_lane;
									cout << "Agent Moved To Right Lane. " << check_right_lane << endl;
								}
							}

    }
return lane;
} 







// MSelim: End custom functions definitions

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
  double max_s = 6945.554;
  
  // Agent starts in middle lane
  int lane  = MIDDLE_LANE; 
  double ref_vel = 0;    // reference velocity     

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  h.onMessage([&lane,&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;


          // MSelim: Begin your code starting from this point
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
            

            // for spline fitting
            int prev_size = previous_path_x.size();
            vector <double>ptsx;
            vector <double>ptsy;

            if (prev_size > 0) {
							car_s = end_path_s;
						}

            // Safety Check --> Agent doesn't change the lane is speed below 30 to avoid collision from behind
            if(car_speed >= 30)          
            lane=choose_next_state(sensor_fusion, car_s,lane, prev_size);
          

              int current_car_lane = 2 + 4 * lane;
              // this loops to each car in the map and reads some of its attributes  to compare with the my car simulator agent
              bool is_too_close=false;
              for(auto &car_readings: sensor_fusion)
              {
                    float d = car_readings[6]; // reads the d for other cars which specfies the lane at which this car at.
                    if(d < current_car_lane + 2 && d > current_car_lane - 2)
                    {
                      double vx = car_readings[3]; // reads x,y compoenent of the velocity
                      double vy = car_readings[4];
                      double check_speed = sqrt(vx*vx + vy*vy);
                      double check_car_s = car_readings[5]; // reads s dimension of the car where it's located exactly in d lane.
                      
                      check_car_s += (double)prev_size*0.02*check_speed; //the other cars locations given there velocities
                           if((check_car_s > car_s) && ((check_car_s - car_s) < 35))
                              { // if car location is after my agent by less than 35 meters
                                //ref_vel = 37.7;
                                //lane = 2;
                                is_too_close = true;
                              }
                    }
              }

            if(is_too_close){
              ref_vel-=0.42;

            }else if (ref_vel<49.5){
              ref_vel+=0.224;
              is_too_close=false;
            }

            double ref_x = car_x;
						double ref_y = car_y;
						double ref_yaw = deg2rad(car_yaw);

						// When previous state is empty, use the car as the beginning point
						if (prev_size < 2) {
								double prev_car_x = car_x - cos(car_yaw);
								double prev_car_y = car_y - sin(car_yaw);

								ptsx.push_back(prev_car_x);
								ptsx.push_back(car_x);

								ptsy.push_back(prev_car_y);
								ptsy.push_back(car_y);

						} else {
						 
								ref_x = previous_path_x[prev_size - 1];
								ref_y = previous_path_y[prev_size - 1];

								 
								double ref_x_prev = previous_path_x[prev_size - 2];
								double ref_y_prev = previous_path_y[prev_size - 2];
								ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

								 
								ptsx.push_back(ref_x_prev);
								ptsx.push_back(ref_x);

								ptsy.push_back(ref_y_prev);
								ptsy.push_back(ref_y);
						}

					 
						vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
						vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
						vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

						ptsx.push_back(next_wp0[0]);
						ptsx.push_back(next_wp1[0]);
						ptsx.push_back(next_wp2[0]);

						ptsy.push_back(next_wp0[1]);
						ptsy.push_back(next_wp1[1]);
						ptsy.push_back(next_wp2[1]);


						for (int i = 0; i < ptsx.size(); i++) {
							 

							double shift_x = ptsx[i] - ref_x;
							double shift_y = ptsy[i] - ref_y;

							ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
							ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

						}

						tk::spline s;

						s.set_points(ptsx, ptsy);

						 
					  

					 
						for (int i = 0; i < previous_path_x.size(); i++ ) {
							next_x_vals.push_back(previous_path_x[i]);
							next_y_vals.push_back(previous_path_y[i]);
						}

						// Calculate how to break up spline so we travel at the desired velocity
						double target_x = 30.0;
						double target_y = s(target_x);
						double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

						double x_add_on = 0;

						for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

							// 2.24 because of mph -> mps conversion
							double N = (target_dist / (.02 * ref_vel/2.24));
							double x_point = x_add_on + (target_x) / N;
							double y_point = s(x_point);

							x_add_on = x_point;

							double x_ref = x_point;
							double y_ref = y_point;

							// Rotate back to normal
							x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
							y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

							x_point += ref_x;
							y_point += ref_y;

							next_x_vals.push_back(x_point);
							next_y_vals.push_back(y_point);

						}



          // MSelim: End your code before this point


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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


    /*  for illustration and trial
          double dist_inc = 0.5;
	        double next_s,next_d;
	        for(int i = 0; i < 50; i++)
	         {
			        next_s = car_s + (i+1)*dist_inc;
			        next_d = 1.5*4;
				
		        	vector <double> xy = getXY(next_s,next_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);



      			 next_x_vals.push_back(xy[0]); // The next x and y
	 	         next_y_vals.push_back(xy[1]);

			 
	        }
			 
		
          */