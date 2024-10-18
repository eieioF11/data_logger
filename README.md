# data_logger

Node to save log in csv format.

## Data Format

file_name|header1,header2,header3,...,|data1,data2,data3,...,

## Data Topic
- topic name : /data_logger/log
- topic type : std_msgs::msg::String
- topic QoS : rclcpp::QoS(10).reliable()

## Log publish library
### Include
```C++
#include <data_logger_utils/log_publisher.hpp>
```
[log_publisher.hpp](https://github.com/eieioF11/data_logger/blob/main/include/data_logger_utils/log_publisher.hpp)

# log_viewer
- https://eieiof11.github.io/log_viewer/log_viewer.html
