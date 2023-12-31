// Generated by gencpp from file op3_walking_module_msgs/SetWalkingParam.msg
// DO NOT EDIT!


#ifndef OP3_WALKING_MODULE_MSGS_MESSAGE_SETWALKINGPARAM_H
#define OP3_WALKING_MODULE_MSGS_MESSAGE_SETWALKINGPARAM_H

#include <ros/service_traits.h>


#include <op3_walking_module_msgs/SetWalkingParamRequest.h>
#include <op3_walking_module_msgs/SetWalkingParamResponse.h>


namespace op3_walking_module_msgs
{

struct SetWalkingParam
{

typedef SetWalkingParamRequest Request;
typedef SetWalkingParamResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetWalkingParam
} // namespace op3_walking_module_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::op3_walking_module_msgs::SetWalkingParam > {
  static const char* value()
  {
    return "5bdce659174df89e66795c32f4aa3cfb";
  }

  static const char* value(const ::op3_walking_module_msgs::SetWalkingParam&) { return value(); }
};

template<>
struct DataType< ::op3_walking_module_msgs::SetWalkingParam > {
  static const char* value()
  {
    return "op3_walking_module_msgs/SetWalkingParam";
  }

  static const char* value(const ::op3_walking_module_msgs::SetWalkingParam&) { return value(); }
};


// service_traits::MD5Sum< ::op3_walking_module_msgs::SetWalkingParamRequest> should match
// service_traits::MD5Sum< ::op3_walking_module_msgs::SetWalkingParam >
template<>
struct MD5Sum< ::op3_walking_module_msgs::SetWalkingParamRequest>
{
  static const char* value()
  {
    return MD5Sum< ::op3_walking_module_msgs::SetWalkingParam >::value();
  }
  static const char* value(const ::op3_walking_module_msgs::SetWalkingParamRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::op3_walking_module_msgs::SetWalkingParamRequest> should match
// service_traits::DataType< ::op3_walking_module_msgs::SetWalkingParam >
template<>
struct DataType< ::op3_walking_module_msgs::SetWalkingParamRequest>
{
  static const char* value()
  {
    return DataType< ::op3_walking_module_msgs::SetWalkingParam >::value();
  }
  static const char* value(const ::op3_walking_module_msgs::SetWalkingParamRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::op3_walking_module_msgs::SetWalkingParamResponse> should match
// service_traits::MD5Sum< ::op3_walking_module_msgs::SetWalkingParam >
template<>
struct MD5Sum< ::op3_walking_module_msgs::SetWalkingParamResponse>
{
  static const char* value()
  {
    return MD5Sum< ::op3_walking_module_msgs::SetWalkingParam >::value();
  }
  static const char* value(const ::op3_walking_module_msgs::SetWalkingParamResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::op3_walking_module_msgs::SetWalkingParamResponse> should match
// service_traits::DataType< ::op3_walking_module_msgs::SetWalkingParam >
template<>
struct DataType< ::op3_walking_module_msgs::SetWalkingParamResponse>
{
  static const char* value()
  {
    return DataType< ::op3_walking_module_msgs::SetWalkingParam >::value();
  }
  static const char* value(const ::op3_walking_module_msgs::SetWalkingParamResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // OP3_WALKING_MODULE_MSGS_MESSAGE_SETWALKINGPARAM_H
