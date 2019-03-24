#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object.
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
		  double delta = j[1]["steering_angle"];
		  double a = j[1]["throttle"];
		  
		  // Init state 
          auto transformed_ptsx = Eigen::VectorXd(ptsx.size());
          auto transformed_ptsy = Eigen::VectorXd(ptsx.size());
		  
		  
		  // Transforming the simulator global coordinates into vehicle coordinates.
          for (unsigned int i = 0; i < ptsx.size(); i++) {
            double dX = ptsx[i] - px;
            double dY = ptsy[i] - py;
			
            transformed_ptsx[i] = (dX * cos(-psi) - dY * sin(-psi));
            transformed_ptsy[i] = (dX * sin(-psi) + dY * cos(-psi));
          }
		 
		  // Calculate the cte and epsi.
		  auto coeffs = polyfit(transformed_ptsx, transformed_ptsy, 3);  
		  double cte = polyeval(coeffs, 0); 
          double epsi = -atan(coeffs[1]); //psi - atan(coeffs[1] + 2 * px* coeffs[2] + 3 * coeffs[3] * pow(px,2))
		  const double Lf = 2.67;
		  
		  // Calculating the independent variables with latency
		  double dt = 0.1;
		  double px_current = 0.0 + v * dt; //psi equals to 0 therefore cos(psi) is 1.
		  double py_current = 0.0;  //psi equals to 0 therefore sin(psi) is 0.
		  double psi_current = 0.0 + v * -delta / Lf * dt;
		  double v_current = v + a * dt;
		  double cte_current = cte + v * sin(epsi) * dt;
		  double epsi_current = epsi + v * -delta / Lf * dt;

		  Eigen::VectorXd state(6);
		  state << px_current, py_current, psi_current, v_current , cte_current, epsi_current;
		  
		  auto vars = mpc.Solve(state, coeffs);
		  
          double steer_value = vars[0]/deg2rad(25);
		  double throttle_value = vars[1];

          json msgJson;
          // Divide by deg2rad(25). Values will be in between [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

        
          // Add (x,y) points, points are in reference to 
		  // the vehicle's coordinate system the points in the simulator are 
          // connected by a Green line
		  for (unsigned int i = 2; i < vars.size(); i++) {
			if (i % 2 == 0) {
              mpc_x_vals.push_back(vars[i]);
			} else {
              mpc_y_vals.push_back(vars[i]);
			}
		  }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

		  //Add (x,y) points, points are in reference to 
          //the vehicle's coordinate system the points in the simulator are 
          //connected by a Yellow line
	
          for (unsigned int i = 0; i < 50; i++) {
            next_x_vals.push_back(i);
            next_y_vals.push_back(polyeval(coeffs, i));
		  }	 	
			
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
		  
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
		  
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
		  
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
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