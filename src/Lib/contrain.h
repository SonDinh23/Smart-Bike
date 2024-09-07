#ifndef CONTRAIN_H_
#define CONTRAIN_H_

#define UTILS_LOW_PASS_FILTER(value, sample, constant)	(value -= (constant) * ((value) - (sample)))
#define UTILS_LOW_PASS_FILTER_2(value, sample, lconstant, hconstant)\
  if (sample < value) {\
    UTILS_LOW_PASS_FILTER(value, sample, lconstant);\
  }\
  else {\
    UTILS_LOW_PASS_FILTER(value, sample, hconstant);\
  }

#define SIMPLE_KALMAN_FILTER(value, sample, gain) (value += gain * (sample - value))

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#endif