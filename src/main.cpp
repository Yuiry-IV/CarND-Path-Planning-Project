#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
           
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


#include "steps.h"

#include "constants.h"

inline double mph_to_ms( double mph) noexcept {
   return ((mph*K_MILE_TO_KM)*M_IN_KM)/SEC_PER_HOUR;
}

inline int d_to_lane( double d) noexcept {
   int lane = -1;
   if(d >= LANE_WIDTH*0.0 && d < LANE_WIDTH*1.0) {
      lane = 0;
   } else if(d >= LANE_WIDTH*1.0 && d < LANE_WIDTH*2.0) {
      lane = 1;
   } else if(d >= LANE_WIDTH*2.0 && d < LANE_WIDTH*3.0) {
      lane = 2;
   }   
   return lane;
}

inline double lane_to_d( int lane ) noexcept {
   double d = LANE_WIDTH*0.5 + LANE_WIDTH*lane;
   
   /*
   // adjust car position from the edge of road
   switch(lane){
     case 0:
      d += LANE_WIDTH*0.0625;
      break;
     case 2:
      d -= LANE_WIDTH*0.0625;
      break;
     default: 
      break;
   }
   */
   
   return d;
}


struct sensor_fusion_result {
   bool car_left= false;
   bool car_right = false;
   bool car_ahead = false;   
};


inline 
sensor_fusion_result 
process_sensor_fusion( const int lane, 
                  const double car_x,
                  const double car_y,
                  const double car_s,
                  const double car_d,
                  const double end_path_s,
                  const double car_speed, const nlohmann::json &telemetry ) noexcept {

   sensor_fusion_result result;
   auto sensor_fusion = telemetry["sensor_fusion"];
   
   for(unsigned i=0; i < sensor_fusion.size(); i++) {
   
      const int other_car_id = sensor_fusion[i][0];
      const double other_car_d = sensor_fusion[i][6];
      const int other_car_lane=d_to_lane(other_car_d);

      if( other_car_lane == -1 ){
         // std::cerr<<id<<" is out of road, and will be ignored."<<std::endl;
         continue;
      }


      const double other_car_x=sensor_fusion[i][1];
      const double other_car_y=sensor_fusion[i][2];


      const double vx=sensor_fusion[i][3];
      const double vy=sensor_fusion[i][4];
      
      const double other_car_speed = std::sqrt(vx*vx+vy*vy);
         
      const double other_car_s = sensor_fusion[i][5]; 
            
      if( (abs(car_s - other_car_s)<1.5) && abs(car_d - other_car_d)<1.0 ) {
         std::cerr <<" proximity detected: id:" << other_car_id<<", other lane:";
         std::cerr <<other_car_lane<<" ("<< other_car_d<<");";
         std::cerr <<"{"<<d_to_lane(car_d)<<",";
         std::cerr <<lane<<"}(";
         std::cerr <<car_d<<"), diff:"<<(other_car_s - car_s)<<",";
         std::cerr <<(car_d - other_car_d);
         std::cerr <<std::endl;
      }
      
      const double deltaF = SENSOR_FUSION_FRONT_BUFFER_K*(end_path_s - car_s); // forward 5/4 x path lenght 
      const double deltaB = SENSOR_FUSION_BACK_BUFFER_K*(end_path_s - car_s); // backward 3/4 x path lenght 
      
      if(other_car_lane == lane) {
         // other car in front of ego car:
         if( other_car_s > car_s && (other_car_s - car_s) < deltaF ){
            //std::cout<<"SF:front:id:"<<other_car_id<<" in {"<<(other_car_s - car_s);
            //std::cout<<(car_d - other_car_d)<<"}";
            //std::cout<<std::endl;
            result.car_ahead = true;
         }           
      } else if((other_car_lane - lane) == -1) {
         // other car in left range of ego car
         if( (car_s+deltaF) > other_car_s  && (car_s-deltaB) < other_car_s ){
            //std::cout<<"SF:_left:id:"<<other_car_id<<" in {"<<(other_car_s - car_s);
            //std::cout<<(car_d - other_car_d)<<"}";
            //std::cout<<std::endl;
            result.car_left = true;
         }
      } else if((other_car_lane - lane) == 1) {
         // other car in right range of ego car
         if( (car_s+deltaF) > other_car_s  && (car_s-deltaB) < other_car_s ){
            //std::cout<<"SF:right:id:"<<other_car_id<<" in {"<<(other_car_s - car_s);
            //std::cout<<(car_d - other_car_d)<<"}";
            //std::cout<<std::endl;
            result.car_right = true;
         }
      }
   }
   
   return result;
}

struct car_state {
   int lane;
   double velocity_current;
};

car_state update_lane_and_velocity( const car_state &state,
   const sensor_fusion_result result, 
   const double & velocity_delta, 
   const double & velocity_max ) noexcept {
   
   car_state new_state = state;
   
   if(result.car_ahead) {
      if(!result.car_left && new_state.lane > 0) {
         new_state.lane--;
      } else if(!result.car_right && new_state.lane < 2) {
         new_state.lane++;
      } else {
         if( new_state.velocity_current - velocity_delta > 0 ) { 
            new_state.velocity_current -= velocity_delta;
         }
      }
   } else {
      // Not in the center lane. Check if it is safe to move back
      
      if( new_state.lane != 1 ) {
		   if ((new_state.lane == 2 && !result.car_left) || (new_state.lane == 0 && !result.car_right)) {
			   // Move back to the center lane
			   new_state.lane = 1;
		   }
      }      
        
      if(new_state.velocity_current < velocity_max){
         new_state.velocity_current += velocity_delta;
      }
   }
   return new_state;
}


struct path {
   vector<double> next_x_vals; 
   vector<double> next_y_vals;
};

path generate_path( const car_state &new_state,
                  const double car_x,
                  const double car_y,
                  const double car_s,
                  const double car_yaw,
                  
                  const vector<double>& previous_path_x,
                  const vector<double>& previous_path_y,
                  
                  const vector<double>& map_waypoints_x,
	               const vector<double>& map_waypoints_y,
	               const vector<double>& map_waypoints_s  
   ) {
   
   path new_path;

   const unsigned int prev_size = previous_path_x.size();


   vector<double> ptsx;
   vector<double> ptsy;

   //Refrence x,y, and yaw states
   double ref_x = car_x;
   double ref_y = car_y;
   double ref_yaw = deg2rad(car_yaw);

   // If previous states are almost empty, use the car as a startting point
   if ( prev_size < 2 ) {
      //Use two points thats makes path tangent to the car
      double prev_car_x = car_x - cos(car_yaw);
      double prev_car_y = car_y - sin(car_yaw);
      ptsx.push_back(prev_car_x);
      ptsx.push_back(car_x);
      ptsy.push_back(prev_car_y);
      ptsy.push_back(car_y);
   } else {
      //Redefine the reference point to previous point
      ref_x = previous_path_x[prev_size - 1];
      ref_y = previous_path_y[prev_size - 1];

      double ref_x_prev = previous_path_x[prev_size - 2];
      double ref_y_prev = previous_path_y[prev_size - 2];
      ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

      ptsx.push_back(ref_x_prev);
      ptsx.push_back(ref_x);
      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);
   }

   // Setting up target points in the future.
   for( unsigned i=0; i<3; ++i){
      vector<double> wp( getXY( car_s + PATH_FUTURE_STEP_SIZE*(1+i), 
            lane_to_d(new_state.lane), 
            map_waypoints_s, 
            map_waypoints_x, 
            map_waypoints_y) 
         );
      ptsx.push_back(wp[0]);
      ptsy.push_back(wp[1]);
   }
   
   // Making coordinates to local car coordinates.
   for ( unsigned i = 0; i < ptsx.size(); i++ ) {
      double shift_x = ptsx[i] - ref_x;
      double shift_y = ptsy[i] - ref_y;

      ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
      ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
   }
   
   // create the spline.
   tk::spline s;
   s.set_points(ptsx, ptsy);

   //For the smooth transition, we are adding previous path points
   for ( unsigned i = 0; i < prev_size; i++ ) {
      new_path.next_x_vals.push_back(previous_path_x[i]);
      new_path.next_y_vals.push_back(previous_path_y[i]);
   }

   // Calculate distance y position on 51 m ahead.
   double target_x = PATH_POS_AHEAD;
   double target_y = s(target_x);
   double target_dist = sqrt(target_x*target_x + target_y*target_y);

   double x_add_on = 0.0;

   const unsigned count = PATH_NUMBER_OF_POINTS - prev_size;

   for( unsigned i = 1; i <count; i++ ) {

      double N = double(PATH_NUMBER_OF_POINTS) * 
                     target_dist / mph_to_ms(new_state.velocity_current);
      double x_point = x_add_on + target_x/N;
      double y_point = s(x_point);

      x_add_on = x_point;

      double x_ref = x_point;
      double y_ref = y_point;

      //Rotate back to normal after rotating it earlier
      x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
      y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

      x_point += ref_x;
      y_point += ref_y;

      new_path.next_x_vals.push_back(x_point);
      new_path.next_y_vals.push_back(y_point);
   }   
   return new_path;
}


void do_highway_driving( vector<double>& next_x_vals, 
                  vector<double>& next_y_vals,
                  const vector<double>& previous_path_x,
                  const vector<double>& previous_path_y,
                  const double car_x,
                  const double car_y,
                  const double car_s,
                  const double car_d,
                  const double car_yaw,
                  const double car_speed,
                  const double end_path_s,
                  const nlohmann::json &telemetry,
                  int &lane,
                  double &velocity_current,
	               double &velocity_delta,
	               double &velocity_max,
	               const vector<double>& map_waypoints_x,
	               const vector<double>& map_waypoints_y,
	               const vector<double>& map_waypoints_s                  
   ){
   // process car environment      
   sensor_fusion_result result = process_sensor_fusion( lane,
                        car_x, car_y, car_s, car_d, end_path_s, car_speed, telemetry );
   
   // update car parametrs according to control strategy 
   car_state new_state = update_lane_and_velocity( car_state{lane, velocity_current},
            result, velocity_delta, velocity_max );
   
   // generate a path
   path new_path = generate_path(new_state, 
                        car_x,
                        car_y,
                        car_s,                        
                        car_yaw,                  
                        previous_path_x,
                        previous_path_y,                  
                        map_waypoints_x,
	                     map_waypoints_y,
	                     map_waypoints_s
   );
   
   lane = new_state.lane;
   velocity_current = new_state.velocity_current;
   
   next_x_vals = new_path.next_x_vals;
   next_y_vals = new_path.next_y_vals;
   
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
  double max_s = 6945.554;

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

  //Car's lane. Stating at middle lane.
  int lane = 1;
  
  //Reference velocity.
  double velocity_current = SPEED_INITIAL; 
  double velocity_delta = SPEED_DELTA;
  double velocity_max = SPEED_LIMIT;
	
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,
               &lane, &velocity_current, &velocity_delta, &velocity_max
               ]
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
          
          /** 
            .FILL THE PATH
            */
          //fill_next_1(next_x_vals, next_y_vals);
          //fill_next_2(next_x_vals, next_y_vals,
          //  previous_path_x,previous_path_y,
          //  car_x, car_y, car_yaw
          //);      
          
          do_highway_driving(next_x_vals, next_y_vals,
            previous_path_x,previous_path_y,
            car_x, car_y, car_s, car_d, car_yaw, car_speed,
            end_path_s,
            j[1],
            lane, velocity_current, velocity_delta, velocity_max,
            map_waypoints_x, map_waypoints_y, map_waypoints_s
          );
          /**
            .END
            */

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
