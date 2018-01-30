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

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
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

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
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

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}
/*
  * Predict if there are cars within 30m vicinity in left, middle and right predictCarsInLanes
  * 0, 1, 2 index represent Left, Middle and Right lanes respectively
*/
vector <bool> predictCarsInLanes(const vector<vector<double>> &sensor_fusion, int current_lane, int prev_path_size, double car_s, double range = 30.0){
    std::vector<bool> vResult;
    vResult.push_back(false);
    vResult.push_back(false);
    vResult.push_back(false);

    cout << "Size = " << sensor_fusion.size() << endl;

    // Traverse thru all of the vehicles on the road
    // cout << "Sensor fusion list size = " << returnSensorFusionSize(sensor_fusion) << endl;
    for (int i = 0; i < sensor_fusion.size(); i++){
      //Find if each car is in the same lane as this car
      // Vehicle data id, veh_x, veh_y, vel_x, vel_y, veh_s, veh_d
      float d = sensor_fusion[i][6];

      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = sensor_fusion[i][5];

      // Look into the future where this vehicle will be forward in prev_path_size steps
      // Simlator goes 50 steps in one sec (1 step  = 1/50 sec)
      check_car_s += (((double)prev_path_size)*.02*check_speed);

      // for each lane index 0, 1, 2
      for (int j = 0; j < 3; j++){
        // Find the vehicle lane
        if ((d < (2+4*j+2)) && (d > (2+4*j-2))){

          // If the car and the sensor vehicle in the same lane
          if (j == current_lane){
            // Check if s of this vehicle is atleast 30 m from the ahead only
            if ( (check_car_s > car_s) && (check_car_s-car_s< range)){
              // Do somethig here, for now reduce the speed
              // ref_velocity = 29.5; //mph
              // bIsTooClose = true;
              vResult[j] = true;
            }
          } else {
            // This vehicle is in different lane compared to the car
            // Check if the vehicle is within 30 m so that it is not safe
            // to change the lane
            if ((check_car_s < car_s+ range) && (-check_car_s+car_s< range)){
              vResult[j] = true;
            }
          }
        } // Loop to detect if the vehicle is in the same lane as the car
      }
    }

    return vResult;
}

int returnChangedLanewithUpdatedSpeed(int current_lane, vector <bool> &lane_occupancy, double &ref_velocity, double target_velocity, double increment_Step){
  // Act on the predctions
  int new_lane = current_lane;

  // See if there is a vehicle in the same lane in the front
  if (lane_occupancy[current_lane]){
    // See if there is no vehicle on the left or right for lane Change
    if (current_lane < 1) { // Only right lane change possible
      // Check if there is no vehicle on the right lane
      if (!lane_occupancy[current_lane+1])
        new_lane = current_lane+1;
    }
    else
      ref_velocity -= increment_Step; // decrease the speed if no lane change allowed and vehicle in the front

    if (current_lane == 1) { // If it is in the middle lane - can change to either lanes
      // Check if there is no vehicle on the right lane
      if (!lane_occupancy[current_lane+1])
        new_lane = current_lane+1;
      // Check if there is no vehicle on the left lane
      else if (!lane_occupancy[current_lane-1])
        new_lane = current_lane-1;
      else  ref_velocity -= increment_Step; // decrease the speed if no lane change allowed and vehicle in the front

    }

    if (current_lane > 1) { // Only left lane change possible
      // Check if there is no vehicle on the left lane
      if (!lane_occupancy[current_lane-1])
        new_lane = current_lane-1;
    }
    else
      ref_velocity -= increment_Step; // decrease the speed if no lane change allowed and vehicle in the front

  }else if (ref_velocity < (target_velocity-2*increment_Step)){
    ref_velocity += increment_Step;
  }

  return new_lane;
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

  // Ref ref_velocity
  double target_velocity = 50.0;
  // Increment step of the velocity
  double increment_Step = 0.224;

  // Have reference velocoty (little less than target velocity of 50 mph)
  // This is essentially 50  -  2 * setp increment in velocity
  double ref_velocity = 0.0; //target_velocity - 2* increment_Step;

  h.onMessage([&target_velocity, &increment_Step, &ref_velocity, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    // Start the car in lane 1




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

          	json msgJson;


          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            int waypoint_spacing = 30;  //in meter
            // Create a list of widely spaced (x, y) waypoints evenly spaces at waypoint_spacing
            vector<double> ptsx;
            vector<double> ptsy;

            // Reference x, y, yaw
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);  // in rad

            //  prev path list sent from the simulator
            int prev_path_size = previous_path_x.size();

            // Check for sensor fusion data to find out all the cars that are
            // on the road within the vicinity

            // Get the current poistion of the car
            if (prev_path_size > 0){
              car_s = end_path_s;
            }

            bool bIsTooClose = false;

/*
            // Traverse thru all of the vehicles on the road
            // cout << "Sensor fusion list size = " << returnSensorFusionSize(sensor_fusion) << endl;
            for (int i = 0; i < sensor_fusion.size(); i++){
              //Find if each car is in the same lane as this car
              // Vehicle data id, veh_x, veh_y, vel_x, vel_y, veh_s, veh_d
              float d = sensor_fusion[i][6];

              // Find the vehicle lane
              if ((d < (2+4*lane+2)) && (d > (2+4*lane-2))){
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx+vy*vy);
                double check_car_s = sensor_fusion[i][5];

                // Look into the future where this vehicle will be forward in prev_path_size steps
                // Simlator goes 50 steps in one sec (1 step  = 1/50 sec)
                check_car_s += (((double)prev_path_size)*.02*check_speed);

                // Check if s of this vehicle is atleast 30 m from the car_righ
                if ( (check_car_s > car_s) && (check_car_s-car_s< 30.0)){

                  // Do somethig here, for now reduce the speed
                  // ref_velocity = 29.5; //mph
                  bIsTooClose = true;

                  if (lane > 0){
                    lane = 0;
                  }
                }
              } // Loop to detect if the vehicle is in the same lane as the car
            }

            // Act on the predctions
            if (lanes_tracking[lane]){
              ref_velocity -= increment_Step; //
            }else if (ref_velocity < (target_velocity-2*increment_Step)){
              ref_velocity += increment_Step;
            }
*/
            // Predict the lanes occupancy information within 30 m reach
            vector <bool> lanes_tracking = predictCarsInLanes(sensor_fusion, lane, prev_path_size, car_s);
            cout << "Lanes TRacking vector size " << lanes_tracking.size() << " with values " << lanes_tracking[0] << " " <<  lanes_tracking[1] << " " << lanes_tracking[2] << " " << endl;

            int new_lane = returnChangedLanewithUpdatedSpeed(lane, lanes_tracking, ref_velocity, target_velocity, increment_Step);
            if (new_lane != lane)
              cout << "Lane changed from " << lane << " to " << new_lane << endl;

            lane = new_lane;

            if (prev_path_size < 2){
              // Use the two points - one unit forward tangent to the existing car pos, car pos
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsy.push_back(prev_car_y);

              ptsx.push_back(car_x);
              ptsy.push_back(car_y);
            } else{
              // Use the prev path end points as the Reference
              ref_x = previous_path_x[prev_path_size-1];
              ref_y = previous_path_y[prev_path_size-1];
              // Get the last but one point from the prev path list
              double ref_x_prev = previous_path_x[prev_path_size-2];
              double ref_y_prev = previous_path_y[prev_path_size-2];

              // Find the yaw based on the end points from the prev end_path_d
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              // Add these to the way points ptsx, map_waypoints_y
              ptsx.push_back(ref_x_prev);
              ptsy.push_back(ref_y_prev);

              ptsx.push_back(ref_x);
              ptsy.push_back(ref_y);

            }

            //In Frenet, add evenly spaces at 30m interval ahead of starting Reference
            // Lane d = lane_width/2 + lane# * lane_width
            vector<double> next_wp0 = getXY(car_s+30, (2.0+lane*4.0), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, (2.0+lane*4.0), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, (2.0+lane*4.0), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsy.push_back(next_wp0[1]);

            ptsx.push_back(next_wp1[0]);
            ptsy.push_back(next_wp1[1]);

            ptsx.push_back(next_wp2[0]);
            ptsy.push_back(next_wp2[1]);

            // Convert the co-rodinates to car co-ordinate system
            for (int i = 0; i < ptsx.size(); i ++){
              // Make car reference position and angle to 0
              // Shift
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              // Rotation
              ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0- ref_yaw);
              ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);

            }

            // Debug
            // for (int d = 0; d < ptsx.size(); d++){
            //   std::cout << "ptsx " << d << " = " << ptsx[d] << endl;
            // }

            // Create the spline
            tk::spline s;

            // Set the ptsx, ptsy to the spline
            s.set_points(ptsx, ptsy);

            // If the prev list is empty, then use the car's existing pos
            for(int i = 0; i < prev_path_size; i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            // Calculate the distance for s = 30 and then find the x and y for eah increment_Step
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x+target_y*target_y);

            double x_add_on = 0.0;

            // Find N points based on the 20 msec interval the simlator sends new waypoints Data
            // 20 msec = 20/1000 sec.
            // mile = 1609.34 m
            double N = target_dist / ((20.0/1000)*ref_velocity*(1609.34/3600));

            // Fill the rest of the path Planner points to make it 50 set_points
            for (int i = 0; i < 50 - prev_path_size; i ++){
                double x_point = x_add_on + target_x / N;
                double y_point = s(x_point);

                x_add_on = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                // Rotate the points to normal
                x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
                y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

                x_point += ref_x;
                y_point += ref_y;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }

            /*
            if(prev_path_size == 0)
            {
                pos_x = car_x;
                pos_y = car_y;
                angle = deg2rad(car_yaw);
            }
            else
            {
                pos_x = previous_path_x[prev_path_size-1];
                pos_y = previous_path_y[prev_path_size-1];

                double pos_x2 = previous_path_x[prev_path_size-2];
                double pos_y2 = previous_path_y[prev_path_size-2];
                angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
            }

            double dist_inc = 0.5;
            for(int i = 0; i < 50-prev_path_size; i++)
            {
                next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
                next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
                pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
                pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
            }
            */
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

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
