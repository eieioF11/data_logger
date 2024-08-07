#pragma once
#include <sys/stat.h>

#include <chrono>
#include <execution>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
using namespace std::chrono_literals;

class DataLogger : public rclcpp::Node
{
public:
  DataLogger(const rclcpp::NodeOptions &options) : DataLogger("", options) {}
  DataLogger(
      const std::string &name_space = "",
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node("data_logger_node", name_space, options)
  {
    RCLCPP_INFO(this->get_logger(), "start data_logger_node");
    // get param
    std::string log_dir_ = param<std::string>("log_dir", "/log");
    mkdir((log_dir_).c_str(), 0777); // log_dir作成
    std::cout << "dir:" << log_dir_ << std::endl;
    dir_name_ = "/" + currentDateTime();
    if (mkdir((log_dir_ + dir_name_).c_str(), 0777) == 0)
    {
      std::cout << "dir_name_:" << dir_name_ << std::endl;
      output_csv_path_ = log_dir_ + dir_name_;
      start_time_ = rclcpp::Clock().now();
      init_str_sub_ = this->create_subscription<std_msgs::msg::String>(
          "/data_logger/init", rclcpp::QoS(10).reliable(), [&](const std_msgs::msg::String::SharedPtr msg)
          {
          std::cout << "get init:" << msg->data << std::endl;
          std::vector<std::string> srt_list = split(msg->data, ",");
          if (srt_list.size() > 0) {
            std::string file_name = srt_list[0];
            std::cout << "file name:" << file_name << std::endl;
            std::ofstream ofs_csv_file(output_csv_path_ + "/" + file_name + ".csv");
            if (ofs_csv_file) {
              ofs_csv_file << "time_stamp" << ',';
              for (size_t i = 1; i < srt_list.size(); i++) {
                ofs_csv_file << srt_list[i] << ',';
              }
              ofs_csv_file << std::endl;
            } else
              RCLCPP_WARN(this->get_logger(), "can not open csv %s", file_name.c_str());
          } });
      log_str_sub_ = this->create_subscription<std_msgs::msg::String>(
          "/data_logger/log", rclcpp::QoS(10).reliable(), [&](const std_msgs::msg::String::SharedPtr msg)
          {
          std::cout << "get log:" << msg->data << std::endl;
          std::vector<std::string> srt_list = split(msg->data, ",");
          if (srt_list.size() > 0) {
            std::string file_name = srt_list[0];
            std::cout << "file name:" << file_name << std::endl;
            std::ofstream ofs_csv_file(output_csv_path_ + "/" + file_name + ".csv", std::ios::app);
            if (ofs_csv_file) {
              double t = (rclcpp::Clock().now() - start_time_).seconds();
              ofs_csv_file << t << ',';
              for (size_t i = 1; i < srt_list.size(); i++) {
                ofs_csv_file << srt_list[i] << ',';
              }
              ofs_csv_file << std::endl;
            } else
              RCLCPP_WARN(this->get_logger(), "can not open csv %s", file_name.c_str());
          } });
    }
    else
      RCLCPP_WARN(this->get_logger(), "can not open dir %s", (log_dir_ + dir_name_).c_str());
  }

private:
  std::string log_dir_;
  std::string dir_name_;
  std::string output_csv_path_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr log_str_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr init_str_sub_;
  rclcpp::Time start_time_;

  template <class T>
  T param(const std::string &name, const T &def)
  {
    T value;
    declare_parameter(name, def);
    get_parameter(name, value);
    return value;
  }

  inline std::vector<std::string> split(const std::string str, const std::string delim = "/")
  {
    std::vector<std::string> out;
    size_t start;
    size_t end = 0;
    while ((start = str.find_first_not_of(delim, end)) != std::string::npos)
    {
      end = str.find(delim, start);
      out.push_back(str.substr(start, end - start));
    }
    return out;
  }

  std::string currentDateTime()
  {
    std::time_t t = std::time(nullptr);
    std::tm *now = std::localtime(&t);

    char buffer[128];
    strftime(buffer, sizeof(buffer), "%Y%m%d_%X", now);
    return buffer;
  }
};