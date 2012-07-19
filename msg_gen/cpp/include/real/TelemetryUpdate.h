/* Auto-generated by genmsg_cpp for file /home/eric/ros_workspace/real/msg/TelemetryUpdate.msg */
#ifndef REAL_MESSAGE_TELEMETRYUPDATE_H
#define REAL_MESSAGE_TELEMETRYUPDATE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"

namespace real
{
template <class ContainerAllocator>
struct TelemetryUpdate_ {
  typedef TelemetryUpdate_<ContainerAllocator> Type;

  TelemetryUpdate_()
  : telemetryHeader()
  , planeID(0)
  , currentLatitude(0.0)
  , currentLongitude(0.0)
  , currentAltitude(0.0)
  , destLatitude(0.0)
  , destLongitude(0.0)
  , destAltitude(0.0)
  , groundSpeed(0.0)
  , targetBearing(0.0)
  , currentWaypointIndex(0)
  , distanceToDestination(0.0)
  {
  }

  TelemetryUpdate_(const ContainerAllocator& _alloc)
  : telemetryHeader(_alloc)
  , planeID(0)
  , currentLatitude(0.0)
  , currentLongitude(0.0)
  , currentAltitude(0.0)
  , destLatitude(0.0)
  , destLongitude(0.0)
  , destAltitude(0.0)
  , groundSpeed(0.0)
  , targetBearing(0.0)
  , currentWaypointIndex(0)
  , distanceToDestination(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _telemetryHeader_type;
   ::std_msgs::Header_<ContainerAllocator>  telemetryHeader;

  typedef int32_t _planeID_type;
  int32_t planeID;

  typedef double _currentLatitude_type;
  double currentLatitude;

  typedef double _currentLongitude_type;
  double currentLongitude;

  typedef double _currentAltitude_type;
  double currentAltitude;

  typedef double _destLatitude_type;
  double destLatitude;

  typedef double _destLongitude_type;
  double destLongitude;

  typedef double _destAltitude_type;
  double destAltitude;

  typedef double _groundSpeed_type;
  double groundSpeed;

  typedef double _targetBearing_type;
  double targetBearing;

  typedef int64_t _currentWaypointIndex_type;
  int64_t currentWaypointIndex;

  typedef double _distanceToDestination_type;
  double distanceToDestination;


  typedef boost::shared_ptr< ::real::TelemetryUpdate_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::real::TelemetryUpdate_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct TelemetryUpdate
typedef  ::real::TelemetryUpdate_<std::allocator<void> > TelemetryUpdate;

typedef boost::shared_ptr< ::real::TelemetryUpdate> TelemetryUpdatePtr;
typedef boost::shared_ptr< ::real::TelemetryUpdate const> TelemetryUpdateConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::real::TelemetryUpdate_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::real::TelemetryUpdate_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace real

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::real::TelemetryUpdate_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::real::TelemetryUpdate_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::real::TelemetryUpdate_<ContainerAllocator> > {
  static const char* value() 
  {
    return "53cd950963d7a5c403c785f8c0a2ffa7";
  }

  static const char* value(const  ::real::TelemetryUpdate_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x53cd950963d7a5c4ULL;
  static const uint64_t static_value2 = 0x03c785f8c0a2ffa7ULL;
};

template<class ContainerAllocator>
struct DataType< ::real::TelemetryUpdate_<ContainerAllocator> > {
  static const char* value() 
  {
    return "real/TelemetryUpdate";
  }

  static const char* value(const  ::real::TelemetryUpdate_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::real::TelemetryUpdate_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header telemetryHeader\n\
int32 planeID\n\
float64 currentLatitude\n\
float64 currentLongitude\n\
float64 currentAltitude\n\
float64 destLatitude\n\
float64 destLongitude\n\
float64 destAltitude\n\
float64 groundSpeed\n\
float64 targetBearing\n\
int64 currentWaypointIndex\n\
float64 distanceToDestination\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::real::TelemetryUpdate_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::real::TelemetryUpdate_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.telemetryHeader);
    stream.next(m.planeID);
    stream.next(m.currentLatitude);
    stream.next(m.currentLongitude);
    stream.next(m.currentAltitude);
    stream.next(m.destLatitude);
    stream.next(m.destLongitude);
    stream.next(m.destAltitude);
    stream.next(m.groundSpeed);
    stream.next(m.targetBearing);
    stream.next(m.currentWaypointIndex);
    stream.next(m.distanceToDestination);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct TelemetryUpdate_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::real::TelemetryUpdate_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::real::TelemetryUpdate_<ContainerAllocator> & v) 
  {
    s << indent << "telemetryHeader: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.telemetryHeader);
    s << indent << "planeID: ";
    Printer<int32_t>::stream(s, indent + "  ", v.planeID);
    s << indent << "currentLatitude: ";
    Printer<double>::stream(s, indent + "  ", v.currentLatitude);
    s << indent << "currentLongitude: ";
    Printer<double>::stream(s, indent + "  ", v.currentLongitude);
    s << indent << "currentAltitude: ";
    Printer<double>::stream(s, indent + "  ", v.currentAltitude);
    s << indent << "destLatitude: ";
    Printer<double>::stream(s, indent + "  ", v.destLatitude);
    s << indent << "destLongitude: ";
    Printer<double>::stream(s, indent + "  ", v.destLongitude);
    s << indent << "destAltitude: ";
    Printer<double>::stream(s, indent + "  ", v.destAltitude);
    s << indent << "groundSpeed: ";
    Printer<double>::stream(s, indent + "  ", v.groundSpeed);
    s << indent << "targetBearing: ";
    Printer<double>::stream(s, indent + "  ", v.targetBearing);
    s << indent << "currentWaypointIndex: ";
    Printer<int64_t>::stream(s, indent + "  ", v.currentWaypointIndex);
    s << indent << "distanceToDestination: ";
    Printer<double>::stream(s, indent + "  ", v.distanceToDestination);
  }
};


} // namespace message_operations
} // namespace ros

#endif // REAL_MESSAGE_TELEMETRYUPDATE_H
