// Generated by gencpp from file op3_action_module_msgs/IsRunningRequest.msg
// DO NOT EDIT!


#ifndef OP3_ACTION_MODULE_MSGS_MESSAGE_ISRUNNINGREQUEST_H
#define OP3_ACTION_MODULE_MSGS_MESSAGE_ISRUNNINGREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace op3_action_module_msgs
{
template <class ContainerAllocator>
struct IsRunningRequest_
{
  typedef IsRunningRequest_<ContainerAllocator> Type;

  IsRunningRequest_()
    {
    }
  IsRunningRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator> const> ConstPtr;

}; // struct IsRunningRequest_

typedef ::op3_action_module_msgs::IsRunningRequest_<std::allocator<void> > IsRunningRequest;

typedef boost::shared_ptr< ::op3_action_module_msgs::IsRunningRequest > IsRunningRequestPtr;
typedef boost::shared_ptr< ::op3_action_module_msgs::IsRunningRequest const> IsRunningRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace op3_action_module_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "op3_action_module_msgs/IsRunningRequest";
  }

  static const char* value(const ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct IsRunningRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::op3_action_module_msgs::IsRunningRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // OP3_ACTION_MODULE_MSGS_MESSAGE_ISRUNNINGREQUEST_H