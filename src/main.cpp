#include <math.h>
#include <uWS/uWS.h>

#include <iostream>
#include <string>

#include "PID.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

bool doReset = false;
bool isResetSent = false;

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * INIT PID Coefficients
   */

  // Manual
  // double Kp_init = 0.5;
  // double Ki_init = 0.5;
  // double Kd_init = 0;

  // 2.5 - latLimit
  // double Kp_init = 0.1891;
  // double Ki_init = 0.4641;
  // double Kd_init = 0;

  // 2.2 - latLimit
  double Kp_init = 0.170478;
  double Ki_init = 0.4641;
  double Kd_init = 0;

  pid.Init(Kp_init, Ki_init, Kd_init);

  // PID pid2;
  // double p2[3] = {0.01, 0.0045, 0.072};
  // pid2.Init(p2[0], p2[1], p2[2]);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          // double speed = std::stod(j[1]["speed"].get<string>());
          // double angle = std::stod(j[1]["steering_angle"].get<string>());
          /**
           * Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          // calculate the steering angle out of the conroller
          pid.UpdateError(cte);
          double steer_value = pid.TotalError();
          // contant speed
          double throttle = 0.2;

          bool doPIDTuning = false;
          if (doPIDTuning && !doReset) {
            // left the lane
            if (abs(cte) > pid.latLimit) {
              // exceeded the first time the lateral limits in this round

              // adjust the PID gains
              pid.Twiddle();

              doReset = true;
            }

            // one lap done
            int oneLapCounts = 1500;
            if (pid.counter > oneLapCounts) {
              // lower the limit
              pid.latLimit -= 0.3;
              if (pid.latLimit < 0.3) {
                pid.latLimit = 0.3;
              }

              doReset = true;
              pid.maxCounter = 0;
              pid.counter = 0;
            }
          }

          // safely reset the simulator once
          if (doReset) {
            if (!isResetSent) {
              std::cout << "counter:" << pid.counter << " Kp:" << pid.K[0]
                        << " Kd:" << pid.K[1] << " Ki:" << pid.K[2]
                        << " Dp:" << pid.D[0] << " Dd:" << pid.D[1]
                        << " Di:" << pid.D[2]
                        << " maxCounter:" << pid.maxCounter
                        << " latLimit:" << pid.latLimit << std::endl;

              // reset the simulation
              std::string reset_msg = "42[\"reset\",{}]";
              ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

              isResetSent = true;
            }

            if (abs(cte) < pid.latLimit * 0.2) {
              // wait until reset is finished
              doReset = false;
              isResetSent = false;
            }
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  });  // end h.onMessage

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