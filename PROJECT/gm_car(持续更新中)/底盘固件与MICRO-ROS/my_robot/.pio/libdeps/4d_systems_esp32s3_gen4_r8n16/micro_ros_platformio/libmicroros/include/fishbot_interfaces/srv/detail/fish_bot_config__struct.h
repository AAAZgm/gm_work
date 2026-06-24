// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from fishbot_interfaces:srv/FishBotConfig.idl
// generated code does not contain a copyright notice

#ifndef FISHBOT_INTERFACES__SRV__DETAIL__FISH_BOT_CONFIG__STRUCT_H_
#define FISHBOT_INTERFACES__SRV__DETAIL__FISH_BOT_CONFIG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'key'
// Member 'value'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/FishBotConfig in the package fishbot_interfaces.
typedef struct fishbot_interfaces__srv__FishBotConfig_Request
{
  rosidl_runtime_c__String key;
  rosidl_runtime_c__String value;
} fishbot_interfaces__srv__FishBotConfig_Request;

// Struct for a sequence of fishbot_interfaces__srv__FishBotConfig_Request.
typedef struct fishbot_interfaces__srv__FishBotConfig_Request__Sequence
{
  fishbot_interfaces__srv__FishBotConfig_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} fishbot_interfaces__srv__FishBotConfig_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'key'
// Member 'value'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/FishBotConfig in the package fishbot_interfaces.
typedef struct fishbot_interfaces__srv__FishBotConfig_Response
{
  rosidl_runtime_c__String key;
  rosidl_runtime_c__String value;
} fishbot_interfaces__srv__FishBotConfig_Response;

// Struct for a sequence of fishbot_interfaces__srv__FishBotConfig_Response.
typedef struct fishbot_interfaces__srv__FishBotConfig_Response__Sequence
{
  fishbot_interfaces__srv__FishBotConfig_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} fishbot_interfaces__srv__FishBotConfig_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FISHBOT_INTERFACES__SRV__DETAIL__FISH_BOT_CONFIG__STRUCT_H_
