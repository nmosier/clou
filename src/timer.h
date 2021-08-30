#pragma once

#include <string>
#include <chrono>

class Timer {
public:
   Timer(std::ostream& os, bool enable, const std::string& msg): msg(msg) {
      do_start();
   }

   ~Timer() {
      stop = now();
      
   }

private:
   std::string msg;
   std::chrono::time_point<std::chrono::steady_clock> start;

   static auto now() { return std::chrono::steady_clock::now(); }
};
