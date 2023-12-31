// Generated by gencpp from file op3_online_walking_module_msgs/GetKinematicsPoseRequest.msg
// DO NOT EDIT!


#ifndef OP3_ONLINE_WALKING_MODULE_MSGS_MESSAGE_GETKINEMATICSPOSEREQUEST_H
#define OP3_ONLINE_WALKING_MODULE_MSGS_MESSAGE_GETKINEMATICSPOSEREQUEST_H


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
struct GetKinematicsPoseRequest_
{
  typedef GetKinematicsPoseRequest_<ContainerAllocator> Type;

  GetKinematicsPoseRequest_()
    : name()  {
    }
  GetKinematicsPoseRequest_(const ContainerAllocator& _alloc)
    : name(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _name_type;
  _name_type name;





  typedef boost::shared_ptr< ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetKinematicsPoseRequest_

typedef ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<std::allocator<void> > GetKinematicsPoseRequest;

typedef boost::shared_ptr< ::op3_online_walking_module_msgs::GetKinematicsPoseRequest > GetKinematicsPoseRequestPtr;
typedef boost::shared_ptr< ::op3_online_walking_module_msgs::GetKinematicsPoseRequest const> GetKinematicsPoseRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator1> & lhs, const ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator2> & rhs)
{
  return lhs.name == rhs.name;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator1> & lhs, const ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace op3_online_walking_module_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c1f3d28f1b044c871e6eff2e9fc3c667";
  }

  static const char* value(const ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc1f3d28f1b044c87ULL;
  static const uint64_t static_value2 = 0x1e6eff2e9fc3c667ULL;
};

template<class ContainerAllocator>
struct DataType< ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "op3_online_walking_module_msgs/GetKinematicsPoseRequest";
  }

  static const char* value(const ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string    name\n"
;
  }

  static const char* value(const ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetKinematicsPoseRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::op3_online_walking_module_msgs::GetKinematicsPoseRequest_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.name);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OP3_ONLINE_WALKING_MODULE_MSGS_MESSAGE_GETKINEMATICSPOSEREQUEST_H
