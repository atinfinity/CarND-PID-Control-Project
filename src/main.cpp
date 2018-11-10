#include "json.hpp"
#include "PID.h"
#include <uWS/uWS.h>
#include <cmath>
#include <string>
#include <iostream>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

bool twiddle = false;

int main() {
  uWS::Hub h;
  PID pid;


  double p[3]  = {0.1,  0.0001, 1.5};
  double dp[3] = {0.01, 0.0001, 0.1};
  double best_p[3] = {p[0], p[1], p[2]};

  // Initialize the pid variable.
  if (twiddle == true) {
    pid.Init(p[0], p[1], p[2]);
  }
  else {
    pid.Init(0.194872, 0.000871561, 1.89404); // tuned parameters using twiddle
  }

  h.onMessage([&pid, &p, &dp, &best_p](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          json msgJson;

          if (twiddle == true) {
            pid.total_cte = pid.total_cte + std::pow(cte, 2);
            if (pid.step == 0) {
              pid.Init(p[0], p[1], p[2]); 
            }

            // Steering value
            pid.UpdateError(cte);
            double steer_value = pid.TotalError();
            pid.step++;
            if (pid.step > pid.max_step) { 
              if (pid.first == true) {
                std::cout << "(p[0], p[1], p[2]) = (" << p[0] << ", " << p[1] << ", " << p[2] << ")" << std::endl;
                p[pid.p_iterator] += dp[pid.p_iterator];
                pid.first = false;
              }
              else {
                pid.error = pid.total_cte / pid.max_step;
                
                if ((pid.error < pid.best_error) && (pid.second == true)) {
                    pid.best_error = pid.error;
                    best_p[0] = p[0];
                    best_p[1] = p[1];
                    best_p[2] = p[2];
                    dp[pid.p_iterator] *= 1.1;
                    pid.sub_move++;

                    std::cout << "iteration = " << pid.total_iterator << std::endl;
                    std::cout << "p_iterator = " << pid.p_iterator << std::endl;
                    std::cout << "(p[0], p[1], p[2]) = (" << p[0] << ", " << p[1] << ", " << p[2] << ")" << std::endl;
                    std::cout << "error = " << pid.error << std::endl;
                    std::cout << "best_error = " << pid.best_error << std::endl;
                    std::cout << "best (p[0], p[1], p[2]) = (" << p[0] << ", " << p[1] << ", " << p[2] << ")" << std::endl;
                }
                else {
                  if (pid.second == true) {
                    std::cout << "(p[0], p[1], p[2]) = (" << p[0] << ", " << p[1] << ", " << p[2] << ")" << std::endl;
                    p[pid.p_iterator] -= 2 * dp[pid.p_iterator];
                    pid.second = false;
                  }
                  else {
                    std::cout << "iteration = " << pid.total_iterator << std::endl;
                    std::cout << "p_iterator = " << pid.p_iterator << std::endl;
                    std::cout << "(p[0], p[1], p[2]) = (" << p[0] << ", " << p[1] << ", " << p[2] << ")" << std::endl;
                    if (pid.error < pid.best_error) {
                        pid.best_error = pid.error;
                        best_p[0] = p[0];
                        best_p[1] = p[1];
                        best_p[2] = p[2];
                        dp[pid.p_iterator] *= 1.1;
                        pid.sub_move++;
                    }
                    else {
                        p[pid.p_iterator] += dp[pid.p_iterator];
                        dp[pid.p_iterator] *= 0.9;
                        pid.sub_move++;
                    }
                    std::cout << "error = " << pid.error << std::endl;
                    std::cout << "best_error = " << pid.best_error << std::endl;
                    std::cout << "best (p[0], p[1], p[2]) = (" << p[0] << ", " << p[1] << ", " << p[2] << ")" << std::endl;
                  }
                }
                
              }

              if (pid.sub_move > 0) {
                pid.p_iterator++;
                pid.first = true;
                pid.second = true;
                pid.sub_move = 0;
              }

              // reset p_iterator
              if (pid.p_iterator == 3) {
                pid.p_iterator = 0;
              }

              pid.step      = 0;
              pid.total_cte = 0.0;
              pid.total_iterator++;

              double sum_dp = dp[0] + dp[1] + dp[2];
              if (sum_dp < pid.tolerance) {
                std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << best_p[1] << best_p[2] << std::endl;
              }
              else {
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              }
            }
            else {
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = 0.3;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              //std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
          }
          else {
            /*
             * Calcuate steering value here, remember the steering value is [-1, 1].
             * NOTE: Feel free to play around with the throttle and speed. Maybe use
             * another PID controller to control the speed!
             */

            pid.UpdateError(cte);
            double steer_value = pid.TotalError();

            // DEBUG
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      }
      else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    }
    else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  const int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  }
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();

  return 0;
}
