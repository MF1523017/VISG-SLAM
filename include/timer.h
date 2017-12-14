#ifndef TIMER_H
#define TIMER_H

#include <chrono>

class Timer {
 public:
  Timer();

  double ElapsedMS() const;
  double Reset();
 private:
  double Diff(std::chrono::time_point<std::chrono::high_resolution_clock> t1,
    std::chrono::time_point<std::chrono::high_resolution_clock> t2) const;
  std::chrono::time_point<std::chrono::high_resolution_clock> now_;
};

#endif  // TIMER_H
