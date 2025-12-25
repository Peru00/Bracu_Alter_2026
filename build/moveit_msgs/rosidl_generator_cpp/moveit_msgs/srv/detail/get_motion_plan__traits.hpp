// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from moveit_msgs:srv/GetMotionPlan.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "moveit_msgs/srv/get_motion_plan.hpp"


#ifndef MOVEIT_MSGS__SRV__DETAIL__GET_MOTION_PLAN__TRAITS_HPP_
#define MOVEIT_MSGS__SRV__DETAIL__GET_MOTION_PLAN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "moveit_msgs/srv/detail/get_motion_plan__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'motion_plan_request'
#include "moveit_msgs/msg/detail/motion_plan_request__traits.hpp"

namespace moveit_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetMotionPlan_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: motion_plan_request
  {
    out << "motion_plan_request: ";
    to_flow_style_yaml(msg.motion_plan_request, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetMotionPlan_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: motion_plan_request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motion_plan_request:\n";
    to_block_style_yaml(msg.motion_plan_request, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetMotionPlan_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace moveit_msgs

namespace rosidl_generator_traits
{

[[deprecated("use moveit_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const moveit_msgs::srv::GetMotionPlan_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  moveit_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use moveit_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const moveit_msgs::srv::GetMotionPlan_Request & msg)
{
  return moveit_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<moveit_msgs::srv::GetMotionPlan_Request>()
{
  return "moveit_msgs::srv::GetMotionPlan_Request";
}

template<>
inline const char * name<moveit_msgs::srv::GetMotionPlan_Request>()
{
  return "moveit_msgs/srv/GetMotionPlan_Request";
}

template<>
struct has_fixed_size<moveit_msgs::srv::GetMotionPlan_Request>
  : std::integral_constant<bool, has_fixed_size<moveit_msgs::msg::MotionPlanRequest>::value> {};

template<>
struct has_bounded_size<moveit_msgs::srv::GetMotionPlan_Request>
  : std::integral_constant<bool, has_bounded_size<moveit_msgs::msg::MotionPlanRequest>::value> {};

template<>
struct is_message<moveit_msgs::srv::GetMotionPlan_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'motion_plan_response'
#include "moveit_msgs/msg/detail/motion_plan_response__traits.hpp"

namespace moveit_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetMotionPlan_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: motion_plan_response
  {
    out << "motion_plan_response: ";
    to_flow_style_yaml(msg.motion_plan_response, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetMotionPlan_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: motion_plan_response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motion_plan_response:\n";
    to_block_style_yaml(msg.motion_plan_response, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetMotionPlan_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace moveit_msgs

namespace rosidl_generator_traits
{

[[deprecated("use moveit_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const moveit_msgs::srv::GetMotionPlan_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  moveit_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use moveit_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const moveit_msgs::srv::GetMotionPlan_Response & msg)
{
  return moveit_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<moveit_msgs::srv::GetMotionPlan_Response>()
{
  return "moveit_msgs::srv::GetMotionPlan_Response";
}

template<>
inline const char * name<moveit_msgs::srv::GetMotionPlan_Response>()
{
  return "moveit_msgs/srv/GetMotionPlan_Response";
}

template<>
struct has_fixed_size<moveit_msgs::srv::GetMotionPlan_Response>
  : std::integral_constant<bool, has_fixed_size<moveit_msgs::msg::MotionPlanResponse>::value> {};

template<>
struct has_bounded_size<moveit_msgs::srv::GetMotionPlan_Response>
  : std::integral_constant<bool, has_bounded_size<moveit_msgs::msg::MotionPlanResponse>::value> {};

template<>
struct is_message<moveit_msgs::srv::GetMotionPlan_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace moveit_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetMotionPlan_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetMotionPlan_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetMotionPlan_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace moveit_msgs

namespace rosidl_generator_traits
{

[[deprecated("use moveit_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const moveit_msgs::srv::GetMotionPlan_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  moveit_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use moveit_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const moveit_msgs::srv::GetMotionPlan_Event & msg)
{
  return moveit_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<moveit_msgs::srv::GetMotionPlan_Event>()
{
  return "moveit_msgs::srv::GetMotionPlan_Event";
}

template<>
inline const char * name<moveit_msgs::srv::GetMotionPlan_Event>()
{
  return "moveit_msgs/srv/GetMotionPlan_Event";
}

template<>
struct has_fixed_size<moveit_msgs::srv::GetMotionPlan_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<moveit_msgs::srv::GetMotionPlan_Event>
  : std::integral_constant<bool, has_bounded_size<moveit_msgs::srv::GetMotionPlan_Request>::value && has_bounded_size<moveit_msgs::srv::GetMotionPlan_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<moveit_msgs::srv::GetMotionPlan_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<moveit_msgs::srv::GetMotionPlan>()
{
  return "moveit_msgs::srv::GetMotionPlan";
}

template<>
inline const char * name<moveit_msgs::srv::GetMotionPlan>()
{
  return "moveit_msgs/srv/GetMotionPlan";
}

template<>
struct has_fixed_size<moveit_msgs::srv::GetMotionPlan>
  : std::integral_constant<
    bool,
    has_fixed_size<moveit_msgs::srv::GetMotionPlan_Request>::value &&
    has_fixed_size<moveit_msgs::srv::GetMotionPlan_Response>::value
  >
{
};

template<>
struct has_bounded_size<moveit_msgs::srv::GetMotionPlan>
  : std::integral_constant<
    bool,
    has_bounded_size<moveit_msgs::srv::GetMotionPlan_Request>::value &&
    has_bounded_size<moveit_msgs::srv::GetMotionPlan_Response>::value
  >
{
};

template<>
struct is_service<moveit_msgs::srv::GetMotionPlan>
  : std::true_type
{
};

template<>
struct is_service_request<moveit_msgs::srv::GetMotionPlan_Request>
  : std::true_type
{
};

template<>
struct is_service_response<moveit_msgs::srv::GetMotionPlan_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MOVEIT_MSGS__SRV__DETAIL__GET_MOTION_PLAN__TRAITS_HPP_
