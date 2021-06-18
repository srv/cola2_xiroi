#ifndef SENSOR_H
#define SENSOR_H

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <cola2_xiroi/MinimalDiagnostic.h>

// # Possible levels of operations
// byte OK=0    # Sensor working correctly
// byte WARN=1  # Sensor has problem
// byte ERROR=2 # There has been an error
// byte STALE=3 # Waiting for init
// byte OFF=4   # Sensor is disabled

enum Status {OK, WARN, ERROR, STALE, OFF};

class SensorError {
 public:
  SensorError(std::string name) : name_(name) {
    time_ = ros::Time::now();
  }
  ros::Time getTime(void) {return time_;}
  std::string getName(void) {return name_;}
 private:
  std::string name_;
  ros::Time time_;
};

typedef std::vector<SensorError> SensorErrors;

template<typename T1, typename T2>
class Sensor {
 public:
  typedef boost::function<void (const T1&)> Callback;
  std::vector<double> samples;
  ros::Subscriber sub;
  ros::Publisher pub1;
  ros::Publisher pub2;
  Sensor(std::string name, std::string in_topic, std::string out_topic, std::string out_topic_2 = "");
  void setCallback(Callback callback) {
    callback_ = callback;
  }

  void publish(const T2& m) {pub1.publish(m);}
  void publish(const T2& m1, const T2& m2) {
    pub1.publish(m1);
    pub2.publish(m2);
  }
  size_t getNumOfErrors(void) {return errors_.size();}
  void error(std::string msg) {
    ROS_WARN_STREAM_THROTTLE(1, node_name_ << ": Sensor " << name_ << " error: " << msg);
    errors_.push_back(SensorError(msg));
  }
  bool ok(void) {return errors_.size()>0;}
  void report(diagnostic_updater::DiagnosticStatusWrapper &stat);

  bool ready() {
    // To be ready either:
    //  1) Sensor is not enabled
    //  2) Sensor is enabled, initialized and is required -> init
    //  3) Sensor is enabled, not initialized, but not required -> !required
    return (!enabled_ || init_ || !required_at_startup_);
  }

  bool init() { return init_;}
  std::string getName(void) {return name_;}
  bool enabled() { return enabled_;}

 private:
  std::string name_;
  std::string node_name_;
  bool init_;
  bool enabled_;
  bool required_at_startup_;
  int min_init_samples_;
  double max_init_time_;
  double init_time_;
  double time_since_last_msg_;
  double min_frequency_;
  SensorErrors errors_;
  Callback callback_;
  ros::Timer timer_;
  diagnostic_updater::Updater updater_;
  cola2_xiroi::MinimalDiagnostic min_diagnostic_msg_;
  ros::Publisher min_diag_pub_;
  void defaultCallback(const T1& m) {
    ROS_ERROR_STREAM(node_name_ << ": Sensor " << name_ << " no callback provided.");
  }
  bool goodSamples();
  void genericCallback(const T1& m);
  void timerCallback(const ros::TimerEvent&);
  void diagnose(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void publishMinimalDiagnostics();
  void deleteOldErrors();
};

template<typename T1, typename T2>
Sensor<T1, T2>::Sensor(std::string name, std::string in_topic, std::string out_topic_1, std::string out_topic_2)
      : name_(name), init_(false), enabled_(false), required_at_startup_(false), updater_() {
  // Node name
  node_name_ = ros::this_node::getName();


  // Prepare diagnostics
  min_diagnostic_msg_.header.frame_id = name_;
  min_diagnostic_msg_.name = name_;
  min_diagnostic_msg_.status = enabled_?STALE:OFF;

  // Read parameters
  ros::NodeHandle nh;
  nh.param("sensors/" + name_ + "/max_init_time", max_init_time_, 30.0);
  nh.param("sensors/" + name_ + "/min_init_samples", min_init_samples_, 1);
  nh.param("sensors/" + name_ + "/min_frequency", min_frequency_, 0.0);
  nh.param("sensors/" + name_ + "/required_at_startup", required_at_startup_, false);
  nh.param("sensors/" + name_ + "/enable", enabled_, false);

  if (!enabled_) return;

  init_time_ = ros::Time::now().toSec();
  callback_ = boost::bind(&Sensor::defaultCallback, this, _1);
  // Set up publisher and subscriber
  sub = nh.subscribe(in_topic, 1, &Sensor::genericCallback, this);
  pub1 = nh.advertise<T2>(out_topic_1, 1);
  min_diag_pub_ = nh.advertise<cola2_xiroi::MinimalDiagnostic>("/sensors/min_diagnostics", 1) ;

  if (out_topic_2 != "") {
    pub2 = nh.advertise<T2>(out_topic_2, 1);
    ROS_INFO_STREAM(node_name_ << ": For sensor " << name_ << " I'm subscribing to " << in_topic << " and publishing to " << out_topic_1 << " and " << out_topic_2);
  } else {
    ROS_INFO_STREAM(node_name_ << ": For sensor " << name_ << " I'm subscribing to " << in_topic << " and publishing to " << out_topic_1);
  }

  updater_.setHardwareID("none");
  updater_.broadcast(0, name_ + std::string(" correctly set up!"));
  updater_.add(name_, this, &Sensor::diagnose);
  timer_ = nh.createTimer(ros::Duration(1.0), &Sensor::timerCallback, this);

  if (!enabled_) {
    ROS_WARN_STREAM(node_name_ << ":  Sensor " << name_ << " is disabled. If this is not correct, check SensorAggregator config file.");
  } else if (!required_at_startup_ && enabled_) {
    ROS_WARN_STREAM(node_name_ << ":  Sensor " << name_ << " is not required at startup. If this is not correct, check SensorAggregator config file.");
  }
}

template<typename T1, typename T2>
void Sensor<T1, T2>::timerCallback(const ros::TimerEvent&) {
  deleteOldErrors();
  publishMinimalDiagnostics();
}

template<typename T1, typename T2>
void Sensor<T1, T2>::deleteOldErrors() {
  //std::cout << "delete old errors" << std::endl;
  while (errors_.size() > 0) {
    ros::Time now = ros::Time::now();
    if (errors_[0].getTime() - now > ros::Duration(10.0)) {
      errors_.erase(errors_.begin());
    }
  }
}

template<typename T1, typename T2>
bool Sensor<T1, T2>::goodSamples() {

  if (min_frequency_ <= 0 && !init_) {
    ROS_INFO_STREAM(node_name_ << ": Sensor " << name_ << " is receiving data!");
    init_ = true;
  }

  if (min_frequency_ > 0 && (int)samples.size() >= min_init_samples_) {
    double acc_incr = 0;
    for (size_t i = 1; i < samples.size(); i++) {
      double incr = samples[i] - samples[i - 1];
      acc_incr += incr;
    }
    double freq = static_cast<double>(samples.size())/acc_incr;
    if (freq > min_frequency_) {
      if (!init_)
        ROS_INFO_STREAM(node_name_ << ": Sensor " << name_ << " desired frequency met: " << freq << " > " << min_frequency_);
      init_ = true;
    } else {
      ROS_WARN_STREAM(node_name_ << ": Sensor "  << name_ << " frequency too low: " << freq << " < " << min_frequency_);
    }
  }
  return init_;
}

template<typename T1, typename T2>
void Sensor<T1, T2>::genericCallback(const T1& m) {
  ros::Time now = ros::Time::now();
  samples.push_back(now.toSec());
  if ((int)samples.size() > min_init_samples_) {
    // Remove first element
    assert(!samples.empty());
    samples.erase(samples.begin());
  }
  // Callback only gets called if message samples are OK
  if (goodSamples()) {
    callback_(m);
  }
  updater_.update();
}

template<typename T1, typename T2>
void Sensor<T1, T2>::diagnose(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  // Compute init time
  double now = ros::Time::now().toSec();
  // Compute time since last message callback
  time_since_last_msg_ = now - samples[samples.size()-1];
  if (!init_ && now - init_time_ > max_init_time_) {
    ROS_WARN_STREAM(node_name_ << ": Sensor " << name_ << "exceeded maximum initialization time. Is your sensor publishing data?");
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Exceeded maximum initialization time. Is your sensor publishing data?");
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  }
  stat.add("Sensor name", name_);
  stat.add("Init", init_);
  stat.add("Last valid messages", samples.size());
  stat.add("Last published time", samples[samples.size()-1]);
}

template<typename T1, typename T2>
void Sensor<T1, T2>::report(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  for (size_t i = 0; i < errors_.size(); i++) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, errors_[i].getName());
    stat.add("Error", errors_[i].getName());
  }
}

template<typename T1, typename T2>
void Sensor<T1, T2>::publishMinimalDiagnostics() {
  // update time
  min_diagnostic_msg_.header.stamp = ros::Time::now();
  std::string description("");

  if (errors_.size() > 0) {
    min_diagnostic_msg_.status = WARN;
    // Get last error
    description = errors_[errors_.size()-1].getName();
  }
  if (errors_.size() > 10) {
    min_diagnostic_msg_.status = ERROR;
    description = "10 errors reached!";
  } else {
    min_diagnostic_msg_.status = init_?OK:STALE;
  }
  min_diagnostic_msg_.description = description;
  min_diag_pub_.publish(min_diagnostic_msg_);
}

#endif // SENSOR_H
