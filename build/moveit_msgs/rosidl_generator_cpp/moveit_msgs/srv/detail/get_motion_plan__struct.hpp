// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from moveit_msgs:srv/GetMotionPlan.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "moveit_msgs/srv/get_motion_plan.hpp"


#ifndef MOVEIT_MSGS__SRV__DETAIL__GET_MOTION_PLAN__STRUCT_HPP_
#define MOVEIT_MSGS__SRV__DETAIL__GET_MOTION_PLAN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'motion_plan_request'
#include "moveit_msgs/msg/detail/motion_plan_request__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__moveit_msgs__srv__GetMotionPlan_Request __attribute__((deprecated))
#else
# define DEPRECATED__moveit_msgs__srv__GetMotionPlan_Request __declspec(deprecated)
#endif

namespace moveit_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetMotionPlan_Request_
{
  using Type = GetMotionPlan_Request_<ContainerAllocator>;

  explicit GetMotionPlan_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : motion_plan_request(_init)
  {
    (void)_init;
  }

  explicit GetMotionPlan_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : motion_plan_request(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _motion_plan_request_type =
    moveit_msgs::msg::MotionPlanRequest_<ContainerAllocator>;
  _motion_plan_request_type motion_plan_request;

  // setters for named parameter idiom
  Type & set__motion_plan_request(
    const moveit_msgs::msg::MotionPlanRequest_<ContainerAllocator> & _arg)
  {
    this->motion_plan_request = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    moveit_msgs::srv::GetMotionPlan_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const moveit_msgs::srv::GetMotionPlan_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<moveit_msgs::srv::GetMotionPlan_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<moveit_msgs::srv::GetMotionPlan_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      moveit_msgs::srv::GetMotionPlan_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<moveit_msgs::srv::GetMotionPlan_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      moveit_msgs::srv::GetMotionPlan_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<moveit_msgs::srv::GetMotionPlan_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<moveit_msgs::srv::GetMotionPlan_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<moveit_msgs::srv::GetMotionPlan_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__moveit_msgs__srv__GetMotionPlan_Request
    std::shared_ptr<moveit_msgs::srv::GetMotionPlan_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__moveit_msgs__srv__GetMotionPlan_Request
    std::shared_ptr<moveit_msgs::srv::GetMotionPlan_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetMotionPlan_Request_ & other) const
  {
    if (this->motion_plan_request != other.motion_plan_request) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetMotionPlan_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetMotionPlan_Request_

// alias to use template instance with default allocator
using GetMotionPlan_Request =
  moveit_msgs::srv::GetMotionPlan_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace moveit_msgs


// Include directives for member types
// Member 'motion_plan_response'
#include "moveit_msgs/msg/detail/motion_plan_response__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__moveit_msgs__srv__GetMotionPlan_Response __attribute__((deprecated))
#else
# define DEPRECATED__moveit_msgs__srv__GetMotionPlan_Response __declspec(deprecated)
#endif

namespace moveit_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetMotionPlan_Response_
{
  using Type = GetMotionPlan_Response_<ContainerAllocator>;

  explicit GetMotionPlan_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : motion_plan_response(_init)
  {
    (void)_init;
  }

  explicit GetMotionPlan_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : motion_plan_response(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _motion_plan_response_type =
    moveit_msgs::msg::MotionPlanResponse_<ContainerAllocator>;
  _motion_plan_response_type motion_plan_response;

  // setters for named parameter idiom
  Type & set__motion_plan_response(
    const moveit_msgs::msg::MotionPlanResponse_<ContainerAllocator> & _arg)
  {
    this->motion_plan_response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    moveit_msgs::srv::GetMotionPlan_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const moveit_msgs::srv::GetMotionPlan_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<moveit_msgs::srv::GetMotionPlan_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<moveit_msgs::srv::GetMotionPlan_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      moveit_msgs::srv::GetMotionPlan_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<moveit_msgs::srv::GetMotionPlan_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      moveit_msgs::srv::GetMotionPlan_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<moveit_msgs::srv::GetMotionPlan_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<moveit_msgs::srv::GetMotionPlan_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<moveit_msgs::srv::GetMotionPlan_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__moveit_msgs__srv__GetMotionPlan_Response
    std::shared_ptr<moveit_msgs::srv::GetMotionPlan_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__moveit_msgs__srv__GetMotionPlan_Response
    std::shared_ptr<moveit_msgs::srv::GetMotionPlan_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetMotionPlan_Response_ & other) const
  {
    if (this->motion_plan_response != other.motion_plan_response) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetMotionPlan_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetMotionPlan_Response_

// alias to use template instance with default allocator
using GetMotionPlan_Response =
  moveit_msgs::srv::GetMotionPlan_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace moveit_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__moveit_msgs__srv__GetMotionPlan_Event __attribute__((deprecated))
#else
# define DEPRECATED__moveit_msgs__srv__GetMotionPlan_Event __declspec(deprecated)
#endif

namespace moveit_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetMotionPlan_Event_
{
  using Type = GetMotionPlan_Event_<ContainerAllocator>;

  explicit GetMotionPlan_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit GetMotionPlan_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<moveit_msgs::srv::GetMotionPlan_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<moveit_msgs::srv::GetMotionPlan_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<moveit_msgs::srv::GetMotionPlan_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<moveit_msgs::srv::GetMotionPlan_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<moveit_msgs::srv::GetMotionPlan_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<moveit_msgs::srv::GetMotionPlan_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<moveit_msgs::srv::GetMotionPlan_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<moveit_msgs::srv::GetMotionPlan_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    moveit_msgs::srv::GetMotionPlan_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const moveit_msgs::srv::GetMotionPlan_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<moveit_msgs::srv::GetMotionPlan_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<moveit_msgs::srv::GetMotionPlan_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      moveit_msgs::srv::GetMotionPlan_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<moveit_msgs::srv::GetMotionPlan_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      moveit_msgs::srv::GetMotionPlan_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<moveit_msgs::srv::GetMotionPlan_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<moveit_msgs::srv::GetMotionPlan_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<moveit_msgs::srv::GetMotionPlan_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__moveit_msgs__srv__GetMotionPlan_Event
    std::shared_ptr<moveit_msgs::srv::GetMotionPlan_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__moveit_msgs__srv__GetMotionPlan_Event
    std::shared_ptr<moveit_msgs::srv::GetMotionPlan_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetMotionPlan_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetMotionPlan_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetMotionPlan_Event_

// alias to use template instance with default allocator
using GetMotionPlan_Event =
  moveit_msgs::srv::GetMotionPlan_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace moveit_msgs

namespace moveit_msgs
{

namespace srv
{

struct GetMotionPlan
{
  using Request = moveit_msgs::srv::GetMotionPlan_Request;
  using Response = moveit_msgs::srv::GetMotionPlan_Response;
  using Event = moveit_msgs::srv::GetMotionPlan_Event;
};

}  // namespace srv

}  // namespace moveit_msgs

#endif  // MOVEIT_MSGS__SRV__DETAIL__GET_MOTION_PLAN__STRUCT_HPP_
