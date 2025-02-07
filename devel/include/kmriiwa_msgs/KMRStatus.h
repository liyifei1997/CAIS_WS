// Generated by gencpp from file kmriiwa_msgs/KMRStatus.msg
// DO NOT EDIT!


#ifndef KMRIIWA_MSGS_MESSAGE_KMRSTATUS_H
#define KMRIIWA_MSGS_MESSAGE_KMRSTATUS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace kmriiwa_msgs
{
template <class ContainerAllocator>
struct KMRStatus_
{
  typedef KMRStatus_<ContainerAllocator> Type;

  KMRStatus_()
    : header()
    , charge_state_percentage(0)
    , motion_enabled(false)
    , warning_field_clear(false)
    , safety_field_clear(false)
    , safety_state_enabled(false)  {
    }
  KMRStatus_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , charge_state_percentage(0)
    , motion_enabled(false)
    , warning_field_clear(false)
    , safety_field_clear(false)
    , safety_state_enabled(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _charge_state_percentage_type;
  _charge_state_percentage_type charge_state_percentage;

   typedef uint8_t _motion_enabled_type;
  _motion_enabled_type motion_enabled;

   typedef uint8_t _warning_field_clear_type;
  _warning_field_clear_type warning_field_clear;

   typedef uint8_t _safety_field_clear_type;
  _safety_field_clear_type safety_field_clear;

   typedef uint8_t _safety_state_enabled_type;
  _safety_state_enabled_type safety_state_enabled;





  typedef boost::shared_ptr< ::kmriiwa_msgs::KMRStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kmriiwa_msgs::KMRStatus_<ContainerAllocator> const> ConstPtr;

}; // struct KMRStatus_

typedef ::kmriiwa_msgs::KMRStatus_<std::allocator<void> > KMRStatus;

typedef boost::shared_ptr< ::kmriiwa_msgs::KMRStatus > KMRStatusPtr;
typedef boost::shared_ptr< ::kmriiwa_msgs::KMRStatus const> KMRStatusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kmriiwa_msgs::KMRStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kmriiwa_msgs::KMRStatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kmriiwa_msgs::KMRStatus_<ContainerAllocator1> & lhs, const ::kmriiwa_msgs::KMRStatus_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.charge_state_percentage == rhs.charge_state_percentage &&
    lhs.motion_enabled == rhs.motion_enabled &&
    lhs.warning_field_clear == rhs.warning_field_clear &&
    lhs.safety_field_clear == rhs.safety_field_clear &&
    lhs.safety_state_enabled == rhs.safety_state_enabled;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kmriiwa_msgs::KMRStatus_<ContainerAllocator1> & lhs, const ::kmriiwa_msgs::KMRStatus_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kmriiwa_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kmriiwa_msgs::KMRStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kmriiwa_msgs::KMRStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kmriiwa_msgs::KMRStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kmriiwa_msgs::KMRStatus_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kmriiwa_msgs::KMRStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kmriiwa_msgs::KMRStatus_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kmriiwa_msgs::KMRStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4a74515beaa6408bf61b8361cab81069";
  }

  static const char* value(const ::kmriiwa_msgs::KMRStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4a74515beaa6408bULL;
  static const uint64_t static_value2 = 0xf61b8361cab81069ULL;
};

template<class ContainerAllocator>
struct DataType< ::kmriiwa_msgs::KMRStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kmriiwa_msgs/KMRStatus";
  }

  static const char* value(const ::kmriiwa_msgs::KMRStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kmriiwa_msgs::KMRStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"int32 charge_state_percentage\n"
"bool motion_enabled\n"
"bool warning_field_clear\n"
"bool safety_field_clear\n"
"bool safety_state_enabled\n"
"\n"
"\n"
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

  static const char* value(const ::kmriiwa_msgs::KMRStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kmriiwa_msgs::KMRStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.charge_state_percentage);
      stream.next(m.motion_enabled);
      stream.next(m.warning_field_clear);
      stream.next(m.safety_field_clear);
      stream.next(m.safety_state_enabled);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct KMRStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kmriiwa_msgs::KMRStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kmriiwa_msgs::KMRStatus_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "charge_state_percentage: ";
    Printer<int32_t>::stream(s, indent + "  ", v.charge_state_percentage);
    s << indent << "motion_enabled: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.motion_enabled);
    s << indent << "warning_field_clear: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.warning_field_clear);
    s << indent << "safety_field_clear: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.safety_field_clear);
    s << indent << "safety_state_enabled: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.safety_state_enabled);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KMRIIWA_MSGS_MESSAGE_KMRSTATUS_H
