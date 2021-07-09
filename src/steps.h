void fill_next_1(vector<double>& next_x_vals, vector<double>& next_y_vals,
   const double car_x,
   const double car_y,
   const double car_yaw){
   /**
      BEGIN->
      Getting Started

         Start by simply trying to move the car forward in a straight line at a constant 50 MPH velocity. Use the car's (x, y) localization information and its heading direction to create a simple, straight path that is drawn directly in front of the car.

         In main.cpp, instead of setting the speed directly, we pass next_x_vals, and next_y_vals to the simulator. We will set the points 0.5 m apart. Since the car moves 50 times a second, a distance of 0.5m per move will create a velocity of 25 m/s. 25 m/s is close to 50 MPH.

   */
   double dist_inc = 0.5;
   for (int i = 0; i < 50; ++i) {
   next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
   next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
   }
   /**
      A simple test moving the car forward at constant velocity
      <-END.BEGIN
   */ 

}

void fill_next_2(vector<double>& next_x_vals, 
   vector<double>& next_y_vals,
   const vector<double>& previous_path_x,
   const vector<double>& previous_path_y,
   const double car_x,
   const double car_y,
   const double car_yaw      
   ){
   
   double pos_x;
   double pos_y;
   double angle;
   
   int path_size = previous_path_x.size();

   for (int i = 0; i < path_size; ++i) {
     next_x_vals.push_back(previous_path_x[i]);
     next_y_vals.push_back(previous_path_y[i]);
   }

   if (path_size == 0) {
     pos_x = car_x;
     pos_y = car_y;
     angle = deg2rad(car_yaw);
   } else {
     pos_x = previous_path_x[path_size-1];
     pos_y = previous_path_y[path_size-1];

     double pos_x2 = previous_path_x[path_size-2];
     double pos_y2 = previous_path_y[path_size-2];
     angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
   }

   double dist_inc = 0.5;
   for (int i = 0; i < 50-path_size; ++i) {    
     next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
     next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
     pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
     pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
   }
}

