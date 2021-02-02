#include <stdlib.h>

class RateLimiter {
 public:
  void setRate(double rate) { changeRate = abs(rate); }

  void reset(double value) { currentValue = value; }

  void update(double target, double dt) {
    double maxChange = changeRate * dt;

    if (abs(target - currentValue) >= maxChange) {
      if (target - currentValue > 0) {
        currentValue += maxChange;
      } else {
        currentValue -= maxChange;
      }
    } else {
      currentValue = target;
    }
  }

  [[nodiscard]] double getValue() const { return currentValue; }

 private:
  double changeRate = 1;
  double currentValue = 0;
};
