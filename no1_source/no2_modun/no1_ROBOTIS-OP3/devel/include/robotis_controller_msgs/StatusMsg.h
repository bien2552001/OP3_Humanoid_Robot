// Generated by gencpp from file robotis_controller_msgs/StatusMsg.msg
// DO NOT EDIT!


#ifndef ROBOTIS_CONTROLLER_MSGS_MESSAGE_STATUSMSG_H
#define ROBOTIS_CONTROLLER_MSGS_MESSAGE_STATUSMSG_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace robotis_controller_msgs
{
template <class ContainerAllocator>
struct StatusMsg_
{
  typedef StatusMsg_<ContainerAllocator> Type;

  StatusMsg_()
    : header()
    , type(0)
    , module_name()
    , status_msg()  {
    }
  StatusMsg_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , type(0)
    , module_name(_alloc)
    , status_msg(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _type_type;
  _type_type type;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _module_name_type;
  _module_name_type module_name;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _status_msg_type;
  _status_msg_type status_msg;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(STATUS_UNKNOWN)
  #undef STATUS_UNKNOWN
#endif
#if defined(_WIN32) && defined(STATUS_INFO)
  #undef STATUS_INFO
#endif
#if defined(_WIN32) && defined(STATUS_WARN)
  #undef STATUS_WARN
#endif
#if defined(_WIN32) && defined(STATUS_ERROR)
  #undef STATUS_ERROR
#endif

  enum {
    STATUS_UNKNOWN = 0u,
    STATUS_INFO = 1u,
    STATUS_WARN = 2u,
    STATUS_ERROR = 3u,
  };


  typedef boost::shared_ptr< ::robotis_controller_msgs::StatusMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotis_controller_msgs::StatusMsg_<ContainerAllocator> const> ConstPtr;

}; // struct StatusMsg_

typedef ::robotis_controller_msgs::StatusMsg_<std::allocator<void> > StatusMsg;

typedef boost::shared_ptr< ::robotis_controller_msgs::StatusMsg > StatusMsgPtr;
typedef boost::shared_ptr< ::robotis_controller_msgs::StatusMsg const> StatusMsgConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotis_controller_msgs::StatusMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotis_controller_msgs::StatusMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robotis_controller_msgs::StatusMsg_<ContainerAllocator1> & lhs, const ::robotis_controller_msgs::StatusMsg_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.type == rhs.type &&
    lhs.module_name == rhs.module_name &&
    lhs.status_msg == rhs.status_msg;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robotis_controller_msgs::StatusMsg_<ContainerAllocator1> & lhs, const ::robotis_controller_msgs::StatusMsg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robotis_controller_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::robotis_controller_msgs::StatusMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotis_controller_msgs::StatusMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotis_controller_msgs::StatusMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotis_controller_msgs::StatusMsg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotis_controller_msgs::StatusMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotis_controller_msgs::StatusMsg_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotis_controller_msgs::StatusMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8d546af394a35cb47516d4d064603220";
  }

  static const char* value(const ::robotis_controller_msgs::StatusMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8d546af394a35cb4ULL;
  static const uint64_t static_value2 = 0x7516d4d064603220ULL;
};

template<class ContainerAllocator>
struct DataType< ::robotis_controller_msgs::StatusMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotis_controller_msgs/StatusMsg";
  }

  static const char* value(const ::robotis_controller_msgs::StatusMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotis_controller_msgs::StatusMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Status Constants\n"
"uint8 STATUS_UNKNOWN = 0\n"
"uint8 STATUS_INFO = 1\n"
"uint8 STATUS_WARN = 2\n"
"uint8 STATUS_ERROR = 3\n"
"\n"
"std_msgs/Header header\n"
"uint8 type\n"
"string module_name\n"
"string status_msg\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::robotis_controller_msgs::StatusMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotis_controller_msgs::StatusMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.type);
      stream.next(m.module_name);
      stream.next(m.status_msg);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StatusMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotis_controller_msgs::StatusMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotis_controller_msgs::StatusMsg_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
    s << indent << "module_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.module_name);
    s << indent << "status_msg: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.status_msg);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTIS_CONTROLLER_MSGS_MESSAGE_STATUSMSG_H
