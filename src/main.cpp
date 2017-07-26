#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <fstream>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.

namespace
{
  enum class MODE {
    NORMAL,
    FAST
  };

  const bool IS_TWIDDLE = false;
  const double TOLERANCE = 0.05;
  const double MAX_CTE = 3.0;
  const size_t MAX_ITER_TWIDDLE = 6000;
  const size_t NUM_COEFF = 3;

  const MODE DRIVING_MODE = MODE::NORMAL;

  const std::array<double, NUM_COEFF> FINAL_COEFF = { 0.2, 0.0003, 4 };

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

  double limitAngle(const double angle) {
    double result = angle;
    if(result > 1) {
      result = 1;
    }else if(result < -1) {
      result = -1;
    }
    return result;
  }

  void twiddle(PID& pid, std::array<double, NUM_COEFF>& dp, double& bestErr, size_t& bestTravPoints, size_t& curAdjCoeff, 
    size_t& prevAdjCoeff) {

    const size_t counter = pid.GetCounter();
    const double err = pid.TotalError();
    auto coeff = pid.GetCoeffs();

    if(curAdjCoeff != prevAdjCoeff) {
      if(counter > bestTravPoints || (counter == bestTravPoints && err < bestErr)) {
        bestTravPoints = counter;
        bestErr = err;
        dp[curAdjCoeff] *= 1.1;
        prevAdjCoeff = curAdjCoeff;
        curAdjCoeff = (curAdjCoeff + 1) % NUM_COEFF;
        while (dp[curAdjCoeff] < 1e-7) {
          prevAdjCoeff = curAdjCoeff;
          curAdjCoeff = (curAdjCoeff + 1) % NUM_COEFF;
        }
        coeff[curAdjCoeff] += dp[curAdjCoeff];
      }else {
        prevAdjCoeff = curAdjCoeff;
        coeff[curAdjCoeff] -= 2 * dp[curAdjCoeff];
      }
    }else {
      if (counter > bestTravPoints || (counter == bestTravPoints && err < bestErr)) {
        bestTravPoints = counter;
        bestErr = err;
        dp[curAdjCoeff] *= 1.1;
        prevAdjCoeff = curAdjCoeff;
        curAdjCoeff = (curAdjCoeff + 1) % NUM_COEFF;
        while (dp[curAdjCoeff] < 1e-7) {
          prevAdjCoeff = curAdjCoeff;
          curAdjCoeff = (curAdjCoeff + 1) % NUM_COEFF;
        }
        coeff[curAdjCoeff] += dp[curAdjCoeff];
      }
      else {
        coeff[curAdjCoeff] += dp[curAdjCoeff];
        dp[curAdjCoeff] *= 0.9;
        curAdjCoeff = (curAdjCoeff + 1) % NUM_COEFF;
        while(dp[curAdjCoeff] < 1e-7) {
          prevAdjCoeff = curAdjCoeff;
          curAdjCoeff = (curAdjCoeff + 1) % NUM_COEFF;
        }
        coeff[curAdjCoeff] += dp[curAdjCoeff];
      }
    }

    pid.Init(coeff[0], coeff[1], coeff[2]);
  }
}

int main()
{
  uWS::Hub h;
  size_t counterIter = 0;
  double bestErr = std::numeric_limits<double>::max();
  size_t bestTravPoints = 0;
  std::array<double, NUM_COEFF> dp = { 0.1, 0.0001, 1 };
  size_t curAdjustableCoeff = 2, prevAdjustableCoeff = NUM_COEFF;
  std::ofstream output;

  PID pid;
  
  if(IS_TWIDDLE) {
    output.open("params.txt");
    pid.Init(0.08969, 0.0001, 1);
  }else {
    pid.Init(FINAL_COEFF[0], FINAL_COEFF[1], FINAL_COEFF[2]);
  }

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          //double speed = std::stod(j[1]["speed"].get<std::string>());
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          std::string msg;
          json msgJson;

          if(IS_TWIDDLE && std::abs(cte) > MAX_CTE && counterIter == 0) {
            msgJson["steering_angle"] = 0;
            msgJson["throttle"] = 0;
            msg = "42[\"steer\"," + msgJson.dump() + "]";
          }else {
            pid.UpdateError(cte);
            double steerValue = pid.Compute();
            limitAngle(steerValue);
            
            // DEBUG
            //std::cout << "CTE: " << cte << " Steering Value: " << steerValue << std::endl;

            
            msgJson["steering_angle"] = steerValue;
            msgJson["throttle"] = DRIVING_MODE == MODE::NORMAL ? 0.4 : 0.5;

            if (IS_TWIDDLE) {
              ++counterIter;

              if (MAX_ITER_TWIDDLE < counterIter || std::abs(cte) > MAX_CTE) {
                counterIter = 0;
                const auto coeff = pid.GetCoeffs();
                std::cout << "[ " << coeff[0] << ", " << coeff[1] << ", " << coeff[2] << "]" << std::endl;

                twiddle(pid, dp, bestErr, bestTravPoints, curAdjustableCoeff, prevAdjustableCoeff);

                if (dp[0] + dp[1] + dp[2] < TOLERANCE) {
                  std::cout << "Coefficients are set." << std::endl;
                  output << "Coefficients are set." << std::endl;
                  exit(0);
                }

                std::cout << "Best counter: " << bestTravPoints << ", best err: " << bestErr << ";\n";
                msg = "42[\"reset\"]";
                
                output << "[ " << coeff[0] << ", " << coeff[1] << ", " << coeff[2] << "] - " << bestErr << std::endl;
              }
              else {
                msg = "42[\"steer\"," + msgJson.dump() + "]";
              }
            }
            else {
              msg = "42[\"steer\"," + msgJson.dump() + "]";
            }
          }


          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
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
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
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

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
