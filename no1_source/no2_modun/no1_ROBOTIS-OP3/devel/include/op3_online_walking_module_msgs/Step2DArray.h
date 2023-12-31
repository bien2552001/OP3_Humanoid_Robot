// Generated by gencpp from file op3_online_walking_module_msgs/Step2DArray.msg
// DO NOT EDIT!


#ifndef OP3_ONLINE_WALKING_MODULE_MSGS_MESSAGE_STEP2DARRAY_H
#define OP3_ONLINE_WALKING_MODULE_MSGS_MESSAGE_STEP2DARRAY_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <op3_online_walking_module_msgs/Step2D.h>

namespace op3_online_walking_module_msgs
{
template <class ContainerAllocator>
struct Step2DArray_
{
  typedef Step2DArray_<ContainerAllocator> Type;

  Step2DArray_()
    : step_time(0.0)
    , footsteps_2d()  {
    }
  Step2DArray_(const ContainerAllocator& _alloc)
    : step_time(0.0)
    , footsteps_2d(_alloc)  {
  (void)_alloc;
    }



   typedef double _step_time_type;
  _step_time_type step_time;

   typedef std::vector< ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator> >> _footsteps_2d_type;
  _footsteps_2d_type footsteps_2d;





  typedef boost::shared_ptr< ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator> const> ConstPtr;

}; // struct Step2DArray_

typedef ::op3_online_walking_module_msgs::Step2DArray_<std::allocator<void> > Step2DArray;

typedef boost::shared_ptr< ::op3_online_walking_module_msgs::Step2DArray > Step2DArrayPtr;
typedef boost::shared_ptr< ::op3_online_walking_module_msgs::Step2DArray const> Step2DArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator1> & lhs, const ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator2> & rhs)
{
  return lhs.step_time == rhs.step_time &&
    lhs.footsteps_2d == rhs.footsteps_2d;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator1> & lhs, const ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace op3_online_walking_module_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "598423594f1c61377b299ae8d55d0f04";
  }

  static const char* value(const ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x598423594f1c6137ULL;
  static const uint64_t static_value2 = 0x7b299ae8d55d0f04ULL;
};

template<class ContainerAllocator>
struct DataType< ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "op3_online_walking_module_msgs/Step2DArray";
  }

  static const char* value(const ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 step_time\n"
"Step2D[] footsteps_2d\n"
"\n"
"================================================================================\n"
"MSG: op3_online_walking_module_msgs/Step2D\n"
"#2D StepData\n"
"\n"
"geometry_msgs/Pose2D step2d   # step pose as relative offset to last leg\n"
"\n"
"\n"
"# which leg to be used (left/right/no, see below)\n"
"uint8 moving_foot   \n"
"\n"
"uint8 LEFT_FOOT_SWING  = 1 # Left foot constant\n"
"uint8 RIGHT_FOOT_SWING = 2 # Right foot constant\n"
"uint8 STANDING         = 3 # Standing constant\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose2D\n"
"# Deprecated\n"
"# Please use the full 3D pose.\n"
"\n"
"# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.\n"
"\n"
"# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.\n"
"\n"
"\n"
"# This expresses a position and orientation on a 2D manifold.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 theta\n"
;
  }

  static const char* value(const ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.step_time);
      stream.next(m.footsteps_2d);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Step2DArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::op3_online_walking_module_msgs::Step2DArray_<ContainerAllocator>& v)
  {
    s << indent << "step_time: ";
    Printer<double>::stream(s, indent + "  ", v.step_time);
    s << indent << "footsteps_2d[]" << std::endl;
    for (size_t i = 0; i < v.footsteps_2d.size(); ++i)
    {
      s << indent << "  footsteps_2d[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator> >::stream(s, indent + "    ", v.footsteps_2d[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // OP3_ONLINE_WALKING_MODULE_MSGS_MESSAGE_STEP2DARRAY_H
