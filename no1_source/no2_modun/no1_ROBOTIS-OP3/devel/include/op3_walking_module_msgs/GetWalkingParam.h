// Generated by gencpp from file op3_walking_module_msgs/GetWalkingParam.msg
// DO NOT EDIT!


#ifndef OP3_WALKING_MODULE_MSGS_MESSAGE_GETWALKINGPARAM_H
#define OP3_WALKING_MODULE_MSGS_MESSAGE_GETWALKINGPARAM_H

#include <ros/service_traits.h>


#include <op3_walking_module_msgs/GetWalkingParamRequest.h>
#include <op3_walking_module_msgs/GetWalkingParamResponse.h>


namespace op3_walking_module_msgs
{

struct GetWalkingParam
{

typedef GetWalkingParamRequest Request;
typedef GetWalkingParamResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetWalkingParam
} // namespace op3_walking_module_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::op3_walking_module_msgs::GetWalkingParam > {
  static const char* value()
  {
    return "fa49c3d94c3b9543346b4158f6b0a3fd";
  }

  static const char* value(const ::op3_walking_module_msgs::GetWalkingParam&) { return value(); }
};

template<>
struct DataType< ::op3_walking_module_msgs::GetWalkingParam > {
  static const char* value()
  {
    return "op3_walking_module_msgs/GetWalkingParam";
  }

  static const char* value(const ::op3_walking_module_msgs::GetWalkingParam&) { return value(); }
};


// service_traits::MD5Sum< ::op3_walking_module_msgs::GetWalkingParamRequest> should match
// service_traits::MD5Sum< ::op3_walking_module_msgs::GetWalkingParam >
template<>
struct MD5Sum< ::op3_walking_module_msgs::GetWalkingParamRequest>
{
  static const char* value()
  {
    return MD5Sum< ::op3_walking_module_msgs::GetWalkingParam >::value();
  }
  static const char* value(const ::op3_walking_module_msgs::GetWalkingParamRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::op3_walking_module_msgs::GetWalkingParamRequest> should match
// service_traits::DataType< ::op3_walking_module_msgs::GetWalkingParam >
template<>
struct DataType< ::op3_walking_module_msgs::GetWalkingParamRequest>
{
  static const char* value()
  {
    return DataType< ::op3_walking_module_msgs::GetWalkingParam >::value();
  }
  static const char* value(const ::op3_walking_module_msgs::GetWalkingParamRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::op3_walking_module_msgs::GetWalkingParamResponse> should match
// service_traits::MD5Sum< ::op3_walking_module_msgs::GetWalkingParam >
template<>
struct MD5Sum< ::op3_walking_module_msgs::GetWalkingParamResponse>
{
  static const char* value()
  {
    return MD5Sum< ::op3_walking_module_msgs::GetWalkingParam >::value();
  }
  static const char* value(const ::op3_walking_module_msgs::GetWalkingParamResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::op3_walking_module_msgs::GetWalkingParamResponse> should match
// service_traits::DataType< ::op3_walking_module_msgs::GetWalkingParam >
template<>
struct DataType< ::op3_walking_module_msgs::GetWalkingParamResponse>
{
  static const char* value()
  {
    return DataType< ::op3_walking_module_msgs::GetWalkingParam >::value();
  }
  static const char* value(const ::op3_walking_module_msgs::GetWalkingParamResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // OP3_WALKING_MODULE_MSGS_MESSAGE_GETWALKINGPARAM_H