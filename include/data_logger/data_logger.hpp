#include <sys/stat.h>

#include <chrono>
// #include <execution>
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
// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "string_util.hpp"


class DataLogger
{
public:
	DataLogger(ros::NodeHandle &nh) : nh_(nh)
	{
        ROS_INFO("start data_logger_node");
        // get param
        nh_.param<std::string>("log_dir",log_dir_, "./log");
        mkdir((log_dir_).c_str(), 0777);  //log_dir作成
        std::cout << "dir:" << log_dir_ << std::endl;
        dir_name_ = "/" + currentDateTime();
        if (mkdir((log_dir_ + dir_name_).c_str(), 0777) == 0) {
            std::cout << "dir_name_:" << dir_name_ << std::endl;
            output_csv_path_ = log_dir_ + dir_name_;
            start_time_= ros::Time::now();
            init_str_sub_ = nh_.subscribe<std_msgs::String>("/data_logger/init", 1,
            [&](const std_msgs::String::ConstPtr &msg) {
                std::cout << "get init:" << msg->data << std::endl;
                std::vector<std::string> srt_list = string_util::split(msg->data, ",");
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
                    ROS_WARN("can not open csv %s", file_name.c_str());
                }
            });
            log_str_sub_ = nh_.subscribe<std_msgs::String>("/data_logger/log", 1, 
            [&](const std_msgs::String::ConstPtr &msg) {
                std::cout << "get log:" << msg->data << std::endl;
                std::vector<std::string> srt_list = string_util::split(msg->data, ",");
                if (srt_list.size() > 0) {
                    std::string file_name = srt_list[0];
                    std::cout << "file name:" << file_name << std::endl;
                    std::ofstream ofs_csv_file(output_csv_path_ + "/" + file_name + ".csv", std::ios::app);
                    if (ofs_csv_file) {
                    double t = (double)(ros::Time::now() - start_time_).toSec();
                    ofs_csv_file << t << ',';
                    for (size_t i = 1; i < srt_list.size(); i++) {
                        ofs_csv_file << srt_list[i] << ',';
                    }
                    ofs_csv_file << std::endl;
                    } else
                    ROS_WARN("can not open csv %s", file_name.c_str());
                }
            });
        }
        else
        ROS_WARN("can not open dir %s", (log_dir_ + dir_name_).c_str());
    
	}

private:
    std::string log_dir_;
    std::string dir_name_;
    std::string output_csv_path_;
	// node handle
	ros::NodeHandle nh_;
	// subscriber
	ros::Subscriber init_str_sub_;
	ros::Subscriber log_str_sub_;

    ros::Time start_time_;
    std::string currentDateTime()
    {
        std::time_t t = std::time(nullptr);
        std::tm * now = std::localtime(&t);

        char buffer[128];
        strftime(buffer, sizeof(buffer), "%Y%m%d_%X", now);
        return buffer;
    }
};

