// Generated by gencpp from file op3_online_walking_module_msgs/FootStepCommand.msg
// DO NOT EDIT!


#ifndef OP3_ONLINE_WALKING_MODULE_MSGS_MESSAGE_FOOTSTEPCOMMAND_H
#define OP3_ONLINE_WALKING_MODULE_MSGS_MESSAGE_FOOTSTEPCOMMAND_H


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
struct FootStepCommand_
{
  typedef FootStepCommand_<ContainerAllocator> Type;

  FootStepCommand_()
    : command()
    , start_leg()
    , step_num(0)
    , step_time(0.0)
    , step_length(0.0)
    , side_length(0.0)
    , step_angle(0.0)  {
    }
  FootStepCommand_(const ContainerAllocator& _alloc)
    : command(_alloc)
    , start_leg(_alloc)
    , step_num(0)
    , step_time(0.0)
    , step_length(0.0)
    , side_length(0.0)
    , step_angle(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _command_type;
  _command_type command;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _start_leg_type;
  _start_leg_type start_leg;

   typedef int32_t _step_num_type;
  _step_num_type step_num;

   typedef double _step_time_type;
  _step_time_type step_time;

   typedef double _step_length_type;
  _step_length_type step_length;

   typedef double _side_length_type;
  _side_length_type side_length;

   typedef double _step_angle_type;
  _step_angle_type step_angle;





  typedef boost::shared_ptr< ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator> const> ConstPtr;

}; // struct FootStepCommand_

typedef ::op3_online_walking_module_msgs::FootStepCommand_<std::allocator<void> > FootStepCommand;

typedef boost::shared_ptr< ::op3_online_walking_module_msgs::FootStepCommand > FootStepCommandPtr;
typedef boost::shared_ptr< ::op3_online_walking_module_msgs::FootStepCommand const> FootStepCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator1> & lhs, const ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator2> & rhs)
{
  return lhs.command == rhs.command &&
    lhs.start_leg == rhs.start_leg &&
    lhs.step_num == rhs.step_num &&
    lhs.step_time == rhs.step_time &&
    lhs.step_length == rhs.step_length &&
    lhs.side_length == rhs.side_length &&
    lhs.step_angle == rhs.step_angle;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator1> & lhs, const ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace op3_online_walking_module_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6b1e994ec1a40ca11db9d34d679a3f8c";
  }

  static const char* value(const ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6b1e994ec1a40ca1ULL;
  static const uint64_t static_value2 = 0x1db9d34d679a3f8cULL;
};

template<class ContainerAllocator>
struct DataType< ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "op3_online_walking_module_msgs/FootStepCommand";
  }

  static const char* value(const ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string  command\n"
"string  start_leg\n"
"int32   step_num\n"
"float64 step_time\n"
"float64 step_length\n"
"float64 side_length\n"
"float64 step_angle\n"
;
  }

  static const char* value(const ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.command);
      stream.next(m.start_leg);
      stream.next(m.step_num);
      stream.next(m.step_time);
      stream.next(m.step_length);
      stream.next(m.side_length);
      stream.next(m.step_angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FootStepCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::op3_online_walking_module_msgs::FootStepCommand_<ContainerAllocator>& v)
  {
    s << indent << "command: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.command);
    s << indent << "start_leg: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.start_leg);
    s << indent << "step_num: ";
    Printer<int32_t>::stream(s, indent + "  ", v.step_num);
    s << indent << "step_time: ";
    Printer<double>::stream(s, indent + "  ", v.step_time);
    s << indent << "step_length: ";
    Printer<double>::stream(s, indent + "  ", v.step_length);
    s << indent << "side_length: ";
    Printer<double>::stream(s, indent + "  ", v.side_length);
    s << indent << "step_angle: ";
    Printer<double>::stream(s, indent + "  ", v.step_angle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OP3_ONLINE_WALKING_MODULE_MSGS_MESSAGE_FOOTSTEPCOMMAND_H
