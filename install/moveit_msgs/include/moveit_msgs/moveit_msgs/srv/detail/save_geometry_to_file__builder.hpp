// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from moveit_msgs:srv/SaveGeometryToFile.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "moveit_msgs/srv/save_geometry_to_file.hpp"


#ifndef MOVEIT_MSGS__SRV__DETAIL__SAVE_GEOMETRY_TO_FILE__BUILDER_HPP_
#define MOVEIT_MSGS__SRV__DETAIL__SAVE_GEOMETRY_TO_FILE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "moveit_msgs/srv/detail/save_geometry_to_file__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace moveit_msgs
{

namespace srv
{

namespace builder
{

class Init_SaveGeometryToFile_Request_file_path_and_name
{
public:
  Init_SaveGeometryToFile_Request_file_path_and_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::moveit_msgs::srv::SaveGeometryToFile_Request file_path_and_name(::moveit_msgs::srv::SaveGeometryToFile_Request::_file_path_and_name_type arg)
  {
    msg_.file_path_and_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::moveit_msgs::srv::SaveGeometryToFile_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::moveit_msgs::srv::SaveGeometryToFile_Request>()
{
  return moveit_msgs::srv::builder::Init_SaveGeometryToFile_Request_file_path_and_name();
}

}  // namespace moveit_msgs


namespace moveit_msgs
{

namespace srv
{

namespace builder
{

class Init_SaveGeometryToFile_Response_success
{
public:
  Init_SaveGeometryToFile_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::moveit_msgs::srv::SaveGeometryToFile_Response success(::moveit_msgs::srv::SaveGeometryToFile_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::moveit_msgs::srv::SaveGeometryToFile_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::moveit_msgs::srv::SaveGeometryToFile_Response>()
{
  return moveit_msgs::srv::builder::Init_SaveGeometryToFile_Response_success();
}

}  // namespace moveit_msgs


namespace moveit_msgs
{

namespace srv
{

namespace builder
{

class Init_SaveGeometryToFile_Event_response
{
public:
  explicit Init_SaveGeometryToFile_Event_response(::moveit_msgs::srv::SaveGeometryToFile_Event & msg)
  : msg_(msg)
  {}
  ::moveit_msgs::srv::SaveGeometryToFile_Event response(::moveit_msgs::srv::SaveGeometryToFile_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::moveit_msgs::srv::SaveGeometryToFile_Event msg_;
};

class Init_SaveGeometryToFile_Event_request
{
public:
  explicit Init_SaveGeometryToFile_Event_request(::moveit_msgs::srv::SaveGeometryToFile_Event & msg)
  : msg_(msg)
  {}
  Init_SaveGeometryToFile_Event_response request(::moveit_msgs::srv::SaveGeometryToFile_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SaveGeometryToFile_Event_response(msg_);
  }

private:
  ::moveit_msgs::srv::SaveGeometryToFile_Event msg_;
};

class Init_SaveGeometryToFile_Event_info
{
public:
  Init_SaveGeometryToFile_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SaveGeometryToFile_Event_request info(::moveit_msgs::srv::SaveGeometryToFile_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SaveGeometryToFile_Event_request(msg_);
  }

private:
  ::moveit_msgs::srv::SaveGeometryToFile_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::moveit_msgs::srv::SaveGeometryToFile_Event>()
{
  return moveit_msgs::srv::builder::Init_SaveGeometryToFile_Event_info();
}

}  // namespace moveit_msgs

#endif  // MOVEIT_MSGS__SRV__DETAIL__SAVE_GEOMETRY_TO_FILE__BUILDER_HPP_
