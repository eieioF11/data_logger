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
    rclcpp::Node *node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr init_log_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
    std::unordered_map<std::string, int> data_sel_;
    std::vector<std::string> data_strs_;
    std::string name_;
    void publish_str(const std::string &str)
    {
      std_msgs::msg::String msg;
      msg.data = str;
      log_pub_->publish(msg);
    }

  public:
    LogPublisher() {}
    LogPublisher(rclcpp::Node *node) : node_(node) {}
    LogPublisher(rclcpp::Node *node,const std::string &name, const std::vector<std::string> &column_names) : node_(node)
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
     * @brief data_loggerの初期化
     *
     * @param name : file name
     * @param column_names : colmnのname list
     */
    void init_data_logger(const std::string &name, const std::vector<std::string> &column_names)
    {
      name_ = name;
      using namespace std::chrono_literals;
      init_log_pub_ = node_->create_publisher<std_msgs::msg::String>("/data_logger/init", rclcpp::QoS(10).reliable());
      log_pub_      = node_->create_publisher<std_msgs::msg::String>("/data_logger/log", rclcpp::QoS(10).reliable());
      while (init_log_pub_->get_subscription_count() < 1 && rclcpp::ok()) {
        RCLCPP_WARN(node_->get_logger(), "Waiting for data logger to start!");
        rclcpp::sleep_for(500ms);
      }
      std_msgs::msg::String msg;
      msg.data = name + ",";
      int count = 0;
      for (const auto &column_name : column_names)
      {
        data_sel_.emplace(column_name, count);
        data_strs_.push_back("");
        count++;
        msg.data += column_name + ",";
      }
      init_log_pub_->publish(msg);
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
      std::string pub_data = name_ + ",";
      for (const auto &data_str : data_strs_)
        pub_data += data_str + ",";
      publish_str(pub_data);
      return pub_data;
    }
    /**
     * @brief データを作成して送信
     *
     * @param args : 送信するデータ(column_namesの順番にすること)
     */
    template <class... Args>
    std::string publish(Args... args)
    {
      std::string pub_data = name_ + ",";
      for (auto &&x : {args...})
        pub_data += std::to_string(x) + ",";
      publish_str(pub_data);
      return pub_data;
    }
  };
} // namespace data_logger