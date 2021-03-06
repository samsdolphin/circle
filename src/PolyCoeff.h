// Generated by gencpp from file traj_gen/PolyCoeff.msg
// DO NOT EDIT!


#ifndef TRAJ_GEN_MESSAGE_POLYCOEFF_H
#define TRAJ_GEN_MESSAGE_POLYCOEFF_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace traj_gen
{
template <class ContainerAllocator>
struct PolyCoeff_
{
  typedef PolyCoeff_<ContainerAllocator> Type;

  PolyCoeff_()
    : coeff()
    , poly_order(0)  {
    }
  PolyCoeff_(const ContainerAllocator& _alloc)
    : coeff(_alloc)
    , poly_order(0)  {
  (void)_alloc;
    }



   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _coeff_type;
  _coeff_type coeff;

   typedef int8_t _poly_order_type;
  _poly_order_type poly_order;





  typedef boost::shared_ptr< ::traj_gen::PolyCoeff_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::traj_gen::PolyCoeff_<ContainerAllocator> const> ConstPtr;

}; // struct PolyCoeff_

typedef ::traj_gen::PolyCoeff_<std::allocator<void> > PolyCoeff;

typedef boost::shared_ptr< ::traj_gen::PolyCoeff > PolyCoeffPtr;
typedef boost::shared_ptr< ::traj_gen::PolyCoeff const> PolyCoeffConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::traj_gen::PolyCoeff_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::traj_gen::PolyCoeff_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace traj_gen

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'traj_gen': ['/home/sam/catkin_ws/src/traj_gen/msg', '/home/sam/catkin_ws/src/traj_gen/msg'], 'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::traj_gen::PolyCoeff_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::traj_gen::PolyCoeff_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::traj_gen::PolyCoeff_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::traj_gen::PolyCoeff_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::traj_gen::PolyCoeff_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::traj_gen::PolyCoeff_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::traj_gen::PolyCoeff_<ContainerAllocator> >
{
  static const char* value()
  {
    return "eced8596b6a1e1e44efd431704fe5562";
  }

  static const char* value(const ::traj_gen::PolyCoeff_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xeced8596b6a1e1e4ULL;
  static const uint64_t static_value2 = 0x4efd431704fe5562ULL;
};

template<class ContainerAllocator>
struct DataType< ::traj_gen::PolyCoeff_<ContainerAllocator> >
{
  static const char* value()
  {
    return "traj_gen/PolyCoeff";
  }

  static const char* value(const ::traj_gen::PolyCoeff_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::traj_gen::PolyCoeff_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[] coeff\n\
int8 poly_order\n\
\n\
\n\
";
  }

  static const char* value(const ::traj_gen::PolyCoeff_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::traj_gen::PolyCoeff_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.coeff);
      stream.next(m.poly_order);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PolyCoeff_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::traj_gen::PolyCoeff_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::traj_gen::PolyCoeff_<ContainerAllocator>& v)
  {
    s << indent << "coeff[]" << std::endl;
    for (size_t i = 0; i < v.coeff.size(); ++i)
    {
      s << indent << "  coeff[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.coeff[i]);
    }
    s << indent << "poly_order: ";
    Printer<int8_t>::stream(s, indent + "  ", v.poly_order);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TRAJ_GEN_MESSAGE_POLYCOEFF_H
