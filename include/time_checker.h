
class TImeChecker {
 private:
  double last_time_point_ = 0.0;
  double time_interval_threshold_;
  /* data */
 public:
  TImeChecker(double time_interval_threshold) : time_interval_threshold_(time_interval_threshold){};

  bool CheckTime(double time) {
    bool ret = false;
    if (last_time_point_ != 0.0) {
      ret = (time - last_time_point_) > time_interval_threshold_;
      if(ret){
	      std::cout << "time interupt: " << (time-last_time_point_)*1000 << "ms" << std::endl;
      }
    }
    last_time_point_ = time;
    return ret;
  };
};
