// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cloud_msgs:msg/CloudInfo.idl
// generated code does not contain a copyright notice

#ifndef CLOUD_MSGS__MSG__DETAIL__CLOUD_INFO__TRAITS_HPP_
#define CLOUD_MSGS__MSG__DETAIL__CLOUD_INFO__TRAITS_HPP_

#include "cloud_msgs/msg/detail/cloud_info__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const cloud_msgs::msg::CloudInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_yaml(msg.header, out, indentation + 2);
  }

  // member: start_ring_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.start_ring_index.size() == 0) {
      out << "start_ring_index: []\n";
    } else {
      out << "start_ring_index:\n";
      for (auto item : msg.start_ring_index) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: end_ring_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.end_ring_index.size() == 0) {
      out << "end_ring_index: []\n";
    } else {
      out << "end_ring_index:\n";
      for (auto item : msg.end_ring_index) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: start_orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_orientation: ";
    value_to_yaml(msg.start_orientation, out);
    out << "\n";
  }

  // member: end_orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "end_orientation: ";
    value_to_yaml(msg.end_orientation, out);
    out << "\n";
  }

  // member: orientation_diff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orientation_diff: ";
    value_to_yaml(msg.orientation_diff, out);
    out << "\n";
  }

  // member: segmented_cloud_ground_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.segmented_cloud_ground_flag.size() == 0) {
      out << "segmented_cloud_ground_flag: []\n";
    } else {
      out << "segmented_cloud_ground_flag:\n";
      for (auto item : msg.segmented_cloud_ground_flag) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: segmented_cloud_col_ind
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.segmented_cloud_col_ind.size() == 0) {
      out << "segmented_cloud_col_ind: []\n";
    } else {
      out << "segmented_cloud_col_ind:\n";
      for (auto item : msg.segmented_cloud_col_ind) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: segmented_cloud_range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.segmented_cloud_range.size() == 0) {
      out << "segmented_cloud_range: []\n";
    } else {
      out << "segmented_cloud_range:\n";
      for (auto item : msg.segmented_cloud_range) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const cloud_msgs::msg::CloudInfo & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<cloud_msgs::msg::CloudInfo>()
{
  return "cloud_msgs::msg::CloudInfo";
}

template<>
inline const char * name<cloud_msgs::msg::CloudInfo>()
{
  return "cloud_msgs/msg/CloudInfo";
}

template<>
struct has_fixed_size<cloud_msgs::msg::CloudInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cloud_msgs::msg::CloudInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<cloud_msgs::msg::CloudInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CLOUD_MSGS__MSG__DETAIL__CLOUD_INFO__TRAITS_HPP_
