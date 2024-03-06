
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

class PIDController {
public:
  float kp, ki, kd, ff, out_min, out_max, err_last, p_term, i_term, d_term, output;

  PIDController(float kp0, float ki0, float kd0, float ff0, float out_min0, float out_max0) {
    kp = kp0;
    ki = ki0;
    kd = kd0;

    ff = ff0;

    out_min = out_min0;
    out_max = out_max0;

    err_last = 0;   // last error, used to calculate dErr
    p_term = 0; 
    i_term = 0;     // Integral term: ki*Si
    d_term = 0;
    output = 0;
  }

  float compute(float err) {

    p_term = kp * err;
    p_term = constrain(p_term, out_min, out_max);

    i_term += ki * err;
    i_term = constrain(i_term, out_min, out_max);

    d_term = kd*(err-err_last);
    d_term = constrain(d_term, out_min, out_max);

    output = p_term + i_term + d_term;

    output += ff; // adds feedforward to output
    
    output = constrain(output, out_min, out_max);

    err_last = err;

    return output;
  }

  void reset() {
    err_last = 0;
    i_term = 0;
  }
};