// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from fishbot_interfaces:srv/FishBotConfig.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "fishbot_interfaces/srv/detail/fish_bot_config__rosidl_typesupport_introspection_c.h"
#include "fishbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "fishbot_interfaces/srv/detail/fish_bot_config__functions.h"
#include "fishbot_interfaces/srv/detail/fish_bot_config__struct.h"


// Include directives for member types
// Member `key`
// Member `value`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void fishbot_interfaces__srv__FishBotConfig_Request__rosidl_typesupport_introspection_c__FishBotConfig_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  fishbot_interfaces__srv__FishBotConfig_Request__init(message_memory);
}

void fishbot_interfaces__srv__FishBotConfig_Request__rosidl_typesupport_introspection_c__FishBotConfig_Request_fini_function(void * message_memory)
{
  fishbot_interfaces__srv__FishBotConfig_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember fishbot_interfaces__srv__FishBotConfig_Request__rosidl_typesupport_introspection_c__FishBotConfig_Request_message_member_array[2] = {
  {
    "key",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fishbot_interfaces__srv__FishBotConfig_Request, key),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fishbot_interfaces__srv__FishBotConfig_Request, value),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers fishbot_interfaces__srv__FishBotConfig_Request__rosidl_typesupport_introspection_c__FishBotConfig_Request_message_members = {
  "fishbot_interfaces__srv",  // message namespace
  "FishBotConfig_Request",  // message name
  2,  // number of fields
  sizeof(fishbot_interfaces__srv__FishBotConfig_Request),
  fishbot_interfaces__srv__FishBotConfig_Request__rosidl_typesupport_introspection_c__FishBotConfig_Request_message_member_array,  // message members
  fishbot_interfaces__srv__FishBotConfig_Request__rosidl_typesupport_introspection_c__FishBotConfig_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  fishbot_interfaces__srv__FishBotConfig_Request__rosidl_typesupport_introspection_c__FishBotConfig_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t fishbot_interfaces__srv__FishBotConfig_Request__rosidl_typesupport_introspection_c__FishBotConfig_Request_message_type_support_handle = {
  0,
  &fishbot_interfaces__srv__FishBotConfig_Request__rosidl_typesupport_introspection_c__FishBotConfig_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_fishbot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, fishbot_interfaces, srv, FishBotConfig_Request)() {
  if (!fishbot_interfaces__srv__FishBotConfig_Request__rosidl_typesupport_introspection_c__FishBotConfig_Request_message_type_support_handle.typesupport_identifier) {
    fishbot_interfaces__srv__FishBotConfig_Request__rosidl_typesupport_introspection_c__FishBotConfig_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &fishbot_interfaces__srv__FishBotConfig_Request__rosidl_typesupport_introspection_c__FishBotConfig_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "fishbot_interfaces/srv/detail/fish_bot_config__rosidl_typesupport_introspection_c.h"
// already included above
// #include "fishbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "fishbot_interfaces/srv/detail/fish_bot_config__functions.h"
// already included above
// #include "fishbot_interfaces/srv/detail/fish_bot_config__struct.h"


// Include directives for member types
// Member `key`
// Member `value`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void fishbot_interfaces__srv__FishBotConfig_Response__rosidl_typesupport_introspection_c__FishBotConfig_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  fishbot_interfaces__srv__FishBotConfig_Response__init(message_memory);
}

void fishbot_interfaces__srv__FishBotConfig_Response__rosidl_typesupport_introspection_c__FishBotConfig_Response_fini_function(void * message_memory)
{
  fishbot_interfaces__srv__FishBotConfig_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember fishbot_interfaces__srv__FishBotConfig_Response__rosidl_typesupport_introspection_c__FishBotConfig_Response_message_member_array[2] = {
  {
    "key",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fishbot_interfaces__srv__FishBotConfig_Response, key),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fishbot_interfaces__srv__FishBotConfig_Response, value),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers fishbot_interfaces__srv__FishBotConfig_Response__rosidl_typesupport_introspection_c__FishBotConfig_Response_message_members = {
  "fishbot_interfaces__srv",  // message namespace
  "FishBotConfig_Response",  // message name
  2,  // number of fields
  sizeof(fishbot_interfaces__srv__FishBotConfig_Response),
  fishbot_interfaces__srv__FishBotConfig_Response__rosidl_typesupport_introspection_c__FishBotConfig_Response_message_member_array,  // message members
  fishbot_interfaces__srv__FishBotConfig_Response__rosidl_typesupport_introspection_c__FishBotConfig_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  fishbot_interfaces__srv__FishBotConfig_Response__rosidl_typesupport_introspection_c__FishBotConfig_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t fishbot_interfaces__srv__FishBotConfig_Response__rosidl_typesupport_introspection_c__FishBotConfig_Response_message_type_support_handle = {
  0,
  &fishbot_interfaces__srv__FishBotConfig_Response__rosidl_typesupport_introspection_c__FishBotConfig_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_fishbot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, fishbot_interfaces, srv, FishBotConfig_Response)() {
  if (!fishbot_interfaces__srv__FishBotConfig_Response__rosidl_typesupport_introspection_c__FishBotConfig_Response_message_type_support_handle.typesupport_identifier) {
    fishbot_interfaces__srv__FishBotConfig_Response__rosidl_typesupport_introspection_c__FishBotConfig_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &fishbot_interfaces__srv__FishBotConfig_Response__rosidl_typesupport_introspection_c__FishBotConfig_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "fishbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "fishbot_interfaces/srv/detail/fish_bot_config__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers fishbot_interfaces__srv__detail__fish_bot_config__rosidl_typesupport_introspection_c__FishBotConfig_service_members = {
  "fishbot_interfaces__srv",  // service namespace
  "FishBotConfig",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // fishbot_interfaces__srv__detail__fish_bot_config__rosidl_typesupport_introspection_c__FishBotConfig_Request_message_type_support_handle,
  NULL  // response message
  // fishbot_interfaces__srv__detail__fish_bot_config__rosidl_typesupport_introspection_c__FishBotConfig_Response_message_type_support_handle
};

static rosidl_service_type_support_t fishbot_interfaces__srv__detail__fish_bot_config__rosidl_typesupport_introspection_c__FishBotConfig_service_type_support_handle = {
  0,
  &fishbot_interfaces__srv__detail__fish_bot_config__rosidl_typesupport_introspection_c__FishBotConfig_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, fishbot_interfaces, srv, FishBotConfig_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, fishbot_interfaces, srv, FishBotConfig_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_fishbot_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, fishbot_interfaces, srv, FishBotConfig)() {
  if (!fishbot_interfaces__srv__detail__fish_bot_config__rosidl_typesupport_introspection_c__FishBotConfig_service_type_support_handle.typesupport_identifier) {
    fishbot_interfaces__srv__detail__fish_bot_config__rosidl_typesupport_introspection_c__FishBotConfig_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)fishbot_interfaces__srv__detail__fish_bot_config__rosidl_typesupport_introspection_c__FishBotConfig_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, fishbot_interfaces, srv, FishBotConfig_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, fishbot_interfaces, srv, FishBotConfig_Response)()->data;
  }

  return &fishbot_interfaces__srv__detail__fish_bot_config__rosidl_typesupport_introspection_c__FishBotConfig_service_type_support_handle;
}
