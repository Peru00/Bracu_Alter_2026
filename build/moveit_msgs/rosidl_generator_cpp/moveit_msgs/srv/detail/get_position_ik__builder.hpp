// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from moveit_msgs:srv/GetPositionIK.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "moveit_msgs/srv/get_position_ik.hpp"


#ifndef MOVEIT_MSGS__SRV__DETAIL__GET_POSITION_IK__BUILDER_HPP_
#define MOVEIT_MSGS__SRV__DETAIL__GET_POSITION_IK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "moveit_msgs/srv/detail/get_position_ik__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace moveit_msgs
{

namespace srv
{

namespace builder
{

class Init_GetPositionIK_Request_ik_request
{
public:
  Init_GetPositionIK_Request_ik_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::moveit_msgs::srv::GetPositionIK_Request ik_request(::moveit_msgs::srv::GetPositionIK_Request::_ik_request_type arg)
  {
    msg_.ik_request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::moveit_msgs::srv::GetPositionIK_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::moveit_msgs::srv::GetPositionIK_Request>()
{
  return moveit_msgs::srv::builder::Init_GetPositionIK_Request_ik_request();
}

}  // namespace moveit_msgs


namespace moveit_msgs
{

namespace srv
{

namespace builder
{

class Init_GetPositionIK_Response_error_code
{
public:
  explicit Init_GetPositionIK_Response_error_code(::moveit_msgs::srv::GetPositionIK_Response & msg)
  : msg_(msg)
  {}
  ::moveit_msgs::srv::GetPositionIK_Response error_code(::moveit_msgs::srv::GetPositionIK_Response::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return std::move(msg_);
  }

private:
  ::moveit_msgs::srv::GetPositionIK_Response msg_;
};

class Init_GetPositionIK_Response_solution
{
public:
  Init_GetPositionIK_Response_solution()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetPositionIK_Response_error_code solution(::moveit_msgs::srv::GetPositionIK_Response::_solution_type arg)
  {
    msg_.solution = std::move(arg);
    return Init_GetPositionIK_Response_error_code(msg_);
  }

private:
  ::moveit_msgs::srv::GetPositionIK_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::moveit_msgs::srv::GetPositionIK_Response>()
{
  return moveit_msgs::srv::builder::Init_GetPositionIK_Response_solution();
}

}  // namespace moveit_msgs


namespace moveit_msgs
{

namespace srv
{

namespace builder
{

class Init_GetPositionIK_Event_response
{
public:
  explicit Init_GetPositionIK_Event_response(::moveit_msgs::srv::GetPositionIK_Event & msg)
  : msg_(msg)
  {}
  ::moveit_msgs::srv::GetPositionIK_Event response(::moveit_msgs::srv::GetPositionIK_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::moveit_msgs::srv::GetPositionIK_Event msg_;
};

class Init_GetPositionIK_Event_request
{
public:
  explicit Init_GetPositionIK_Event_request(::moveit_msgs::srv::GetPositionIK_Event & msg)
  : msg_(msg)
  {}
  Init_GetPositionIK_Event_response request(::moveit_msgs::srv::GetPositionIK_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetPositionIK_Event_response(msg_);
  }

private:
  ::moveit_msgs::srv::GetPositionIK_Event msg_;
};

class Init_GetPositionIK_Event_info
{
public:
  Init_GetPositionIK_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetPositionIK_Event_request info(::moveit_msgs::srv::GetPositionIK_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetPositionIK_Event_request(msg_);
  }

private:
  ::moveit_msgs::srv::GetPositionIK_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::moveit_msgs::srv::GetPositionIK_Event>()
{
  return moveit_msgs::srv::builder::Init_GetPositionIK_Event_info();
}

}  // namespace moveit_msgs

#endif  // MOVEIT_MSGS__SRV__DETAIL__GET_POSITION_IK__BUILDER_HPP_
