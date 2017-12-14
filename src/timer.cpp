#include "timer.h"

Timer::Timer() {
  Reset();
}

double Timer::ElapsedMS() const {
  return Diff(now_, std::chrono::high_resolution_clock::now());
}

double Timer::Reset() {
  auto old = now_;
  now_ = std::chrono::high_resolution_clock::now();
  return Diff(old, now_);
}

double Timer::Diff(std::chrono::time_point<std::chrono::high_resolution_clock> t1,
    std::chrono::time_point<std::chrono::high_resolution_clock> t2) const {
  std::chrono::duration<double> diff = t2 - t1;
  return diff.count() * 1000;
}
