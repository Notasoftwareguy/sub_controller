// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:srv/CommandResponse.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_interfaces/srv/command_response.hpp"


#ifndef CUSTOM_INTERFACES__SRV__DETAIL__COMMAND_RESPONSE__BUILDER_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__COMMAND_RESPONSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/srv/detail/command_response__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_CommandResponse_Request_expect_response
{
public:
  explicit Init_CommandResponse_Request_expect_response(::custom_interfaces::srv::CommandResponse_Request & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::srv::CommandResponse_Request expect_response(::custom_interfaces::srv::CommandResponse_Request::_expect_response_type arg)
  {
    msg_.expect_response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::CommandResponse_Request msg_;
};

class Init_CommandResponse_Request_command
{
public:
  Init_CommandResponse_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandResponse_Request_expect_response command(::custom_interfaces::srv::CommandResponse_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_CommandResponse_Request_expect_response(msg_);
  }

private:
  ::custom_interfaces::srv::CommandResponse_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::CommandResponse_Request>()
{
  return custom_interfaces::srv::builder::Init_CommandResponse_Request_command();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_CommandResponse_Response_response
{
public:
  Init_CommandResponse_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::srv::CommandResponse_Response response(::custom_interfaces::srv::CommandResponse_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::CommandResponse_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::CommandResponse_Response>()
{
  return custom_interfaces::srv::builder::Init_CommandResponse_Response_response();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_CommandResponse_Event_response
{
public:
  explicit Init_CommandResponse_Event_response(::custom_interfaces::srv::CommandResponse_Event & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::srv::CommandResponse_Event response(::custom_interfaces::srv::CommandResponse_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::CommandResponse_Event msg_;
};

class Init_CommandResponse_Event_request
{
public:
  explicit Init_CommandResponse_Event_request(::custom_interfaces::srv::CommandResponse_Event & msg)
  : msg_(msg)
  {}
  Init_CommandResponse_Event_response request(::custom_interfaces::srv::CommandResponse_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_CommandResponse_Event_response(msg_);
  }

private:
  ::custom_interfaces::srv::CommandResponse_Event msg_;
};

class Init_CommandResponse_Event_info
{
public:
  Init_CommandResponse_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandResponse_Event_request info(::custom_interfaces::srv::CommandResponse_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_CommandResponse_Event_request(msg_);
  }

private:
  ::custom_interfaces::srv::CommandResponse_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::CommandResponse_Event>()
{
  return custom_interfaces::srv::builder::Init_CommandResponse_Event_info();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__COMMAND_RESPONSE__BUILDER_HPP_
