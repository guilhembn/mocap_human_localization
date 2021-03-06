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
 * Auto-generated by genmsg_cpp from file /home/gmilliez/robotpkg/localization/optitrack-genom3/work.elea/templates/ros/client/ros/optitrack/msg/optitrack_ids_pubdata_s.msg
 *
 */


#ifndef OPTITRACK_MESSAGE_OPTITRACK_IDS_PUBDATA_S_H
#define OPTITRACK_MESSAGE_OPTITRACK_IDS_PUBDATA_S_H


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
struct optitrack_ids_pubdata_s_
{
  typedef optitrack_ids_pubdata_s_<ContainerAllocator> Type;

  optitrack_ids_pubdata_s_()
    : bodies(false)
    , markers(false)
    , anon(false)  {
    }
  optitrack_ids_pubdata_s_(const ContainerAllocator& _alloc)
    : bodies(false)
    , markers(false)
    , anon(false)  {
    }



   typedef uint8_t _bodies_type;
  _bodies_type bodies;

   typedef uint8_t _markers_type;
  _markers_type markers;

   typedef uint8_t _anon_type;
  _anon_type anon;




  typedef boost::shared_ptr< ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct optitrack_ids_pubdata_s_

typedef ::optitrack::optitrack_ids_pubdata_s_<std::allocator<void> > optitrack_ids_pubdata_s;

typedef boost::shared_ptr< ::optitrack::optitrack_ids_pubdata_s > optitrack_ids_pubdata_sPtr;
typedef boost::shared_ptr< ::optitrack::optitrack_ids_pubdata_s const> optitrack_ids_pubdata_sConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator> >
{
  static const char* value()
  {
    return "80666125f0a7f664885bcd6b164b3fa7";
  }

  static const char* value(const ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x80666125f0a7f664ULL;
  static const uint64_t static_value2 = 0x885bcd6b164b3fa7ULL;
};

template<class ContainerAllocator>
struct DataType< ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator> >
{
  static const char* value()
  {
    return "optitrack/optitrack_ids_pubdata_s";
  }

  static const char* value(const ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# IDL struct ::optitrack::ids::pubdata_s\n\
bool bodies\n\
bool markers\n\
bool anon\n\
";
  }

  static const char* value(const ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.bodies);
      stream.next(m.markers);
      stream.next(m.anon);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct optitrack_ids_pubdata_s_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::optitrack::optitrack_ids_pubdata_s_<ContainerAllocator>& v)
  {
    s << indent << "bodies: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.bodies);
    s << indent << "markers: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.markers);
    s << indent << "anon: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.anon);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPTITRACK_MESSAGE_OPTITRACK_IDS_PUBDATA_S_H
