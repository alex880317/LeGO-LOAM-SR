// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from cloud_msgs:msg/CloudInfo.idl
// generated code does not contain a copyright notice
#include "cloud_msgs/msg/detail/cloud_info__rosidl_typesupport_fastrtps_cpp.hpp"
#include "cloud_msgs/msg/detail/cloud_info__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace cloud_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cloud_msgs
cdr_serialize(
  const cloud_msgs::msg::CloudInfo & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: start_ring_index
  {
    cdr << ros_message.start_ring_index;
  }
  // Member: end_ring_index
  {
    cdr << ros_message.end_ring_index;
  }
  // Member: start_orientation
  cdr << ros_message.start_orientation;
  // Member: end_orientation
  cdr << ros_message.end_orientation;
  // Member: orientation_diff
  cdr << ros_message.orientation_diff;
  // Member: segmented_cloud_ground_flag
  {
    cdr << ros_message.segmented_cloud_ground_flag;
  }
  // Member: segmented_cloud_col_ind
  {
    cdr << ros_message.segmented_cloud_col_ind;
  }
  // Member: segmented_cloud_range
  {
    cdr << ros_message.segmented_cloud_range;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cloud_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  cloud_msgs::msg::CloudInfo & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: start_ring_index
  {
    cdr >> ros_message.start_ring_index;
  }

  // Member: end_ring_index
  {
    cdr >> ros_message.end_ring_index;
  }

  // Member: start_orientation
  cdr >> ros_message.start_orientation;

  // Member: end_orientation
  cdr >> ros_message.end_orientation;

  // Member: orientation_diff
  cdr >> ros_message.orientation_diff;

  // Member: segmented_cloud_ground_flag
  {
    cdr >> ros_message.segmented_cloud_ground_flag;
  }

  // Member: segmented_cloud_col_ind
  {
    cdr >> ros_message.segmented_cloud_col_ind;
  }

  // Member: segmented_cloud_range
  {
    cdr >> ros_message.segmented_cloud_range;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cloud_msgs
get_serialized_size(
  const cloud_msgs::msg::CloudInfo & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: start_ring_index
  {
    size_t array_size = ros_message.start_ring_index.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.start_ring_index[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: end_ring_index
  {
    size_t array_size = ros_message.end_ring_index.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.end_ring_index[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: start_orientation
  {
    size_t item_size = sizeof(ros_message.start_orientation);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: end_orientation
  {
    size_t item_size = sizeof(ros_message.end_orientation);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: orientation_diff
  {
    size_t item_size = sizeof(ros_message.orientation_diff);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: segmented_cloud_ground_flag
  {
    size_t array_size = ros_message.segmented_cloud_ground_flag.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.segmented_cloud_ground_flag[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: segmented_cloud_col_ind
  {
    size_t array_size = ros_message.segmented_cloud_col_ind.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.segmented_cloud_col_ind[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: segmented_cloud_range
  {
    size_t array_size = ros_message.segmented_cloud_range.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.segmented_cloud_range[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cloud_msgs
max_serialized_size_CloudInfo(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        full_bounded, current_alignment);
    }
  }

  // Member: start_ring_index
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: end_ring_index
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: start_orientation
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: end_orientation
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: orientation_diff
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: segmented_cloud_ground_flag
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: segmented_cloud_col_ind
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: segmented_cloud_range
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _CloudInfo__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const cloud_msgs::msg::CloudInfo *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _CloudInfo__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<cloud_msgs::msg::CloudInfo *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _CloudInfo__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const cloud_msgs::msg::CloudInfo *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _CloudInfo__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_CloudInfo(full_bounded, 0);
}

static message_type_support_callbacks_t _CloudInfo__callbacks = {
  "cloud_msgs::msg",
  "CloudInfo",
  _CloudInfo__cdr_serialize,
  _CloudInfo__cdr_deserialize,
  _CloudInfo__get_serialized_size,
  _CloudInfo__max_serialized_size
};

static rosidl_message_type_support_t _CloudInfo__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_CloudInfo__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace cloud_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_cloud_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<cloud_msgs::msg::CloudInfo>()
{
  return &cloud_msgs::msg::typesupport_fastrtps_cpp::_CloudInfo__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, cloud_msgs, msg, CloudInfo)() {
  return &cloud_msgs::msg::typesupport_fastrtps_cpp::_CloudInfo__handle;
}

#ifdef __cplusplus
}
#endif
