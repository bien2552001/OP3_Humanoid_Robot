// Generated by gencpp from file op3_online_walking_module_msgs/GetJointPoseRequest.msg
// DO NOT EDIT!


#ifndef OP3_ONLINE_WALKING_MODULE_MSGS_MESSAGE_GETJOINTPOSEREQUEST_H
#define OP3_ONLINE_WALKING_MODULE_MSGS_MESSAGE_GETJOINTPOSEREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace op3_online_walking_module_msgs
{
template <class ContainerAllocator>
struct GetJointPoseRequest_
{
  typedef GetJointPoseRequest_<ContainerAllocator> Type;

  GetJointPoseRequest_()
    {
    }
  GetJointPoseRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetJointPoseRequest_

typedef ::op3_online_walking_module_msgs::GetJointPoseRequest_<std::allocator<void> > GetJointPoseRequest;

typedef boost::shared_ptr< ::op3_online_walking_module_msgs::GetJointPoseRequest > GetJointPoseRequestPtr;
typedef boost::shared_ptr< ::op3_online_walking_module_msgs::GetJointPoseRequest const> GetJointPoseRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace op3_online_walking_module_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "op3_online_walking_module_msgs/GetJointPoseRequest";
  }

  static const char* value(const ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetJointPoseRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::op3_online_walking_module_msgs::GetJointPoseRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // OP3_ONLINE_WALKING_MODULE_MSGS_MESSAGE_GETJOINTPOSEREQUEST_H
