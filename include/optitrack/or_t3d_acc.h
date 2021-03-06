/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/gmilliez/robotpkg/localization/optitrack-genom3/work.elea/templates/ros/client/ros/optitrack/msg/or_t3d_acc.msg
 *
 */


#ifndef OPTITRACK_MESSAGE_OR_T3D_ACC_H
#define OPTITRACK_MESSAGE_OR_T3D_ACC_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace optitrack
{
template <class ContainerAllocator>
struct or_t3d_acc_
{
  typedef or_t3d_acc_<ContainerAllocator> Type;

  or_t3d_acc_()
    : ax(0.0)
    , ay(0.0)
    , az(0.0)  {
    }
  or_t3d_acc_(const ContainerAllocator& _alloc)
    : ax(0.0)
    , ay(0.0)
    , az(0.0)  {
    }



   typedef double _ax_type;
  _ax_type ax;

   typedef double _ay_type;
  _ay_type ay;

   typedef double _az_type;
  _az_type az;




  typedef boost::shared_ptr< ::optitrack::or_t3d_acc_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::optitrack::or_t3d_acc_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct or_t3d_acc_

typedef ::optitrack::or_t3d_acc_<std::allocator<void> > or_t3d_acc;

typedef boost::shared_ptr< ::optitrack::or_t3d_acc > or_t3d_accPtr;
typedef boost::shared_ptr< ::optitrack::or_t3d_acc const> or_t3d_accConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::optitrack::or_t3d_acc_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::optitrack::or_t3d_acc_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace optitrack

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/hydro/share/std_msgs/msg'], 'actionlib_msgs': ['/opt/ros/hydro/share/actionlib_msgs/msg'], 'optitrack': ['optitrack/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::optitrack::or_t3d_acc_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::optitrack::or_t3d_acc_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optitrack::or_t3d_acc_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optitrack::or_t3d_acc_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optitrack::or_t3d_acc_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optitrack::or_t3d_acc_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::optitrack::or_t3d_acc_<ContainerAllocator> >
{
  static const char* value()
  {
    return "271a8351f08a1993852d2e5e55f1efa1";
  }

  static const char* value(const ::optitrack::or_t3d_acc_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x271a8351f08a1993ULL;
  static const uint64_t static_value2 = 0x852d2e5e55f1efa1ULL;
};

template<class ContainerAllocator>
struct DataType< ::optitrack::or_t3d_acc_<ContainerAllocator> >
{
  static const char* value()
  {
    return "optitrack/or_t3d_acc";
  }

  static const char* value(const ::optitrack::or_t3d_acc_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::optitrack::or_t3d_acc_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# IDL struct ::or::t3d::acc\n\
float64 ax\n\
float64 ay\n\
float64 az\n\
";
  }

  static const char* value(const ::optitrack::or_t3d_acc_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::optitrack::or_t3d_acc_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ax);
      stream.next(m.ay);
      stream.next(m.az);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct or_t3d_acc_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::optitrack::or_t3d_acc_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::optitrack::or_t3d_acc_<ContainerAllocator>& v)
  {
    s << indent << "ax: ";
    Printer<double>::stream(s, indent + "  ", v.ax);
    s << indent << "ay: ";
    Printer<double>::stream(s, indent + "  ", v.ay);
    s << indent << "az: ";
    Printer<double>::stream(s, indent + "  ", v.az);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPTITRACK_MESSAGE_OR_T3D_ACC_H
