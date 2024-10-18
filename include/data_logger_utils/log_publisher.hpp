#include <iostream>
#include <unordered_map>
#include <iterator>
#include <algorithm>
#include <string>
// ros2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace data_logger
{
  class LogPublisher
  {
  private:
    /*
    data format : file_name|header1,header2,header3,...|data1,data2,data3,...
    */
    struct LogData
    {
      std::string file_name;
      std::string headers; // header1,header2,header3,...,
      std::string data;    // data1,data2,data3,...,
      std::string make_pub_data()
      {
        return file_name + "|" + headers + "|" + data;
      }
    };

    rclcpp::Node *node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
    std::unordered_map<std::string, int> data_sel_;
    std::vector<std::string> data_strs_;
    LogData log_data_;
    void publish_str(const std::string &str)
    {
      std_msgs::msg::String msg;
      msg.data = str;
      log_pub_->publish(msg);
    }

  public:
    LogPublisher() {}
    LogPublisher(rclcpp::Node *node) : node_(node) {}
    LogPublisher(rclcpp::Node *node, const std::string &name) : node_(node) { log_data_.file_name = name; }
    LogPublisher(rclcpp::Node *node, const std::string &name, const std::vector<std::string> &column_names) : node_(node)
    {
      init_data_logger(name, column_names);
    }
    /**
     * @brief ノードのセット
     *
     * @param node
     */
    void set_node(rclcpp::Node *node) { node_ = node; }
    /**
     * @brief ファイル名設定
     *
     * @param name
     */
    void set_name(const std::string &name) { log_data_.file_name = name; }
    /**
     * @brief data_loggerの初期化
     *
     * @param name : file name
     * @param column_names : colmnのname list
     */
    void init_data_logger(const std::string &name, const std::vector<std::string> &column_names)
    {
      log_data_.file_name = name;
      using namespace std::chrono_literals;
      log_pub_ = node_->create_publisher<std_msgs::msg::String>("/data_logger/log", rclcpp::QoS(10).reliable());
      int count = 0;
      log_data_.headers = "";
      for (const auto &column_name : column_names)
      {
        data_sel_.emplace(column_name, count);
        data_strs_.push_back("");
        count++;
        log_data_.headers += column_name + ",";
      }
    }
    /**
     * @brief data_loggerの初期化
     *
     * @param column_names : colmnのname list
     */
    void init_data_logger(const std::vector<std::string> &column_names)
    {
      init_data_logger(log_data_.file_name, column_names);
    }
    /**
     * @brief データのセット
     *
     * @param column_name : dataのcolumn name
     * @param data : 値
     */
    template <class T>
    void set(const std::string &column_name, const T &data)
    {
      if (data_sel_.find(column_name) == data_sel_.end())
      {
        RCLCPP_ERROR(node_->get_logger(), "column_name is not found");
        return;
      }
      data_strs_[data_sel_[column_name]] = std::to_string(data);
    }
    /**
     * @brief データの送信 set_logでセットしたデータを送信
     *
     */
    std::string publish()
    {
      log_data_.data = "";
      for (const auto &data_str : data_strs_)
        log_data_.data += data_str + ",";
      std::string pub_data = log_data_.make_pub_data();
      publish_str(pub_data);
      return pub_data;
    }
  };
} // namespace data_logger