// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from cloud_msgs:msg/CloudInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "cloud_msgs/msg/detail/cloud_info__rosidl_typesupport_introspection_c.h"
#include "cloud_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "cloud_msgs/msg/detail/cloud_info__functions.h"
#include "cloud_msgs/msg/detail/cloud_info__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `start_ring_index`
// Member `end_ring_index`
// Member `segmented_cloud_ground_flag`
// Member `segmented_cloud_col_ind`
// Member `segmented_cloud_range`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cloud_msgs__msg__CloudInfo__init(message_memory);
}

void CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_fini_function(void * message_memory)
{
  cloud_msgs__msg__CloudInfo__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_member_array[9] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cloud_msgs__msg__CloudInfo, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "start_ring_index",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cloud_msgs__msg__CloudInfo, start_ring_index),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "end_ring_index",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cloud_msgs__msg__CloudInfo, end_ring_index),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "start_orientation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cloud_msgs__msg__CloudInfo, start_orientation),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "end_orientation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cloud_msgs__msg__CloudInfo, end_orientation),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "orientation_diff",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cloud_msgs__msg__CloudInfo, orientation_diff),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "segmented_cloud_ground_flag",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cloud_msgs__msg__CloudInfo, segmented_cloud_ground_flag),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "segmented_cloud_col_ind",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cloud_msgs__msg__CloudInfo, segmented_cloud_col_ind),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "segmented_cloud_range",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cloud_msgs__msg__CloudInfo, segmented_cloud_range),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_members = {
  "cloud_msgs__msg",  // message namespace
  "CloudInfo",  // message name
  9,  // number of fields
  sizeof(cloud_msgs__msg__CloudInfo),
  CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_member_array,  // message members
  CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_type_support_handle = {
  0,
  &CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cloud_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cloud_msgs, msg, CloudInfo)() {
  CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_type_support_handle.typesupport_identifier) {
    CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
