@# Included from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
@{
TEMPLATE(
    'msg__type_support.c.em',
    package_name=package_name, interface_path=interface_path, message=service.request_message,
    include_directives=include_directives)
}@

@{
TEMPLATE(
    'msg__type_support.c.em',
    package_name=package_name, interface_path=interface_path, message=service.response_message,
    include_directives=include_directives)
}@

@{
TEMPLATE(
    'msg__type_support.c.em',
    package_name=package_name, interface_path=interface_path, message=service.event_message,
    include_directives=include_directives)
}@

@{
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore
from rosidl_parser.definition import SERVICE_REQUEST_MESSAGE_SUFFIX
from rosidl_parser.definition import SERVICE_RESPONSE_MESSAGE_SUFFIX
from rosidl_parser.definition import SERVICE_EVENT_MESSAGE_SUFFIX
include_parts = [package_name] + list(interface_path.parents[0].parts) + [
    'detail', convert_camel_case_to_lower_case_underscore(interface_path.stem)]
include_base = '/'.join(include_parts)

header_files = [
    'rosidl_runtime_c/service_type_support_struct.h',
    package_name + '/msg/rosidl_typesupport_introspection_c__visibility_control.h',
    include_base + '__rosidl_typesupport_introspection_c.h',
    'rosidl_typesupport_introspection_c/identifier.h',
    'rosidl_typesupport_introspection_c/service_introspection.h',
]

message_function_prefix = '__'.join([package_name] + list(interface_path.parents[0].parts) + [service.namespaced_type.name]) + f'{SERVICE_EVENT_MESSAGE_SUFFIX}__rosidl_typesupport_introspection_c'
function_prefix = '__'.join(include_parts) + '__rosidl_typesupport_introspection_c'
}@
@[for header_file in header_files]@
@[    if header_file in include_directives]@
// already included above
// @
@[    else]@
@{include_directives.add(header_file)}@
@[    end if]@
#include "@(header_file)"
@[end for]@

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers @(function_prefix)__@(service.namespaced_type.name)_service_members = {
  "@('__'.join([package_name] + list(interface_path.parents[0].parts)))",  // service namespace
  "@(service.namespaced_type.name)",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // @(function_prefix)__@(service.request_message.structure.namespaced_type.name)_message_type_support_handle,
  NULL,  // response message
  // @(function_prefix)__@(service.response_message.structure.namespaced_type.name)_message_type_support_handle
  NULL  // event_message
  // @(function_prefix)__@(service.response_message.structure.namespaced_type.name)_message_type_support_handle
};

@{event_type = '__'.join([package_name, *interface_path.parents[0].parts, service.namespaced_type.name]) + SERVICE_EVENT_MESSAGE_SUFFIX}@
@{request_type = '__'.join([package_name, *interface_path.parents[0].parts, service.namespaced_type.name]) + SERVICE_REQUEST_MESSAGE_SUFFIX}@
@{response_type = '__'.join([package_name, *interface_path.parents[0].parts, service.namespaced_type.name]) + SERVICE_RESPONSE_MESSAGE_SUFFIX}@

void *
rosidl_typesupport_introspection_c_@('__'.join([package_name, *interface_path.parents[0].parts, service.namespaced_type.name]))__event_message__create(
  const rosidl_service_introspection_info_t * info,
  rcutils_allocator_t * allocator,
  const void * request_message,
  const void * response_message,
  bool enable_message_payload)
{
  @event_type * event_msg = (@event_type *)(allocator->allocate(sizeof(@event_type), allocator->state));
  if (!@(event_type)__init(event_msg)) {
    allocator->deallocate(event_msg, allocator->state);
    return NULL;
  }

  event_msg->info.event_type = info->event_type;
  event_msg->info.sequence_number = info->sequence_number;
  event_msg->info.stamp.sec = info->stamp_sec;
  event_msg->info.stamp.nanosec = info->stamp_nanosec;
  for (size_t i = 0; i < 16; ++i) {
    event_msg->info.client_id.uuid[i] = info->client_id[i];
  }

  if (!enable_message_payload) {
    return event_msg;
  }

  if (request_message) {
    event_msg->response.capacity = 1;
    event_msg->response.size = 1;
    event_msg->response.data = (@response_type *)(allocator->allocate(sizeof(@response_type), allocator->state));
    if (!@(response_type)__copy((const @response_type *)(response_message), &event_msg->response.data[0])) {
      allocator->deallocate(event_msg, allocator->state);
      return NULL;
    }
  }
  if (response_message) {
    event_msg->request.capacity = 1;
    event_msg->request.size = 1;
    event_msg->request.data = (@request_type *)(allocator->allocate(sizeof(@request_type), allocator->state));
    if (!@(request_type)__copy((const @request_type *)(request_message), &event_msg->request.data[0])) {
      allocator->deallocate(event_msg, allocator->state);
      return NULL;
    }
  }
  return event_msg;
}

bool
rosidl_typesupport_introspection_c_@('__'.join([package_name, *interface_path.parents[0].parts, service.namespaced_type.name]))__event_message__destroy(
  void * event_msg,
  rcutils_allocator_t * allocator)
{
  if (NULL == event_msg) {
    return false;
  }
  @event_type * _event_msg = (@event_type *)(event_msg);

  @(event_type)__fini((@event_type *)(_event_msg));
  if (_event_msg->request.data) {
    allocator->deallocate(_event_msg->request.data, allocator->state);
  }
  if (_event_msg->response.data) {
    allocator->deallocate(_event_msg->response.data, allocator->state);
  }
  allocator->deallocate(_event_msg, allocator->state);
  return true;
}

static rosidl_service_type_support_t @(function_prefix)__@(service.namespaced_type.name)_service_type_support_handle = {
  0,
  &@(function_prefix)__@(service.namespaced_type.name)_service_members,
  get_service_typesupport_handle_function,
  rosidl_typesupport_introspection_c_@('__'.join([package_name, *interface_path.parents[0].parts, service.namespaced_type.name]))__event_message__create,
  rosidl_typesupport_introspection_c_@('__'.join([package_name, *interface_path.parents[0].parts, service.namespaced_type.name]))__event_message__destroy,
  &@(message_function_prefix)__@(service.namespaced_type.name)@(SERVICE_EVENT_MESSAGE_SUFFIX)_message_type_support_handle,
};
// rcl_interfaces__srv__GetParameters_Event__rosidl_typesupport_introspection_c__GetParameters_Event_message_type_support_handle
// rcl_interfaces__srv__detail__get_parameters__rosidl_typesupport_introspection_c__GetParameters_Event_message_type_support_handle,

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, @(', '.join([package_name] + list(interface_path.parents[0].parts))), @(service.namespaced_type.name + SERVICE_REQUEST_MESSAGE_SUFFIX))();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, @(', '.join([package_name] + list(interface_path.parents[0].parts))), @(service.namespaced_type.name + SERVICE_RESPONSE_MESSAGE_SUFFIX))();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, @(', '.join([package_name] + list(interface_path.parents[0].parts))), @(service.namespaced_type.name + SERVICE_EVENT_MESSAGE_SUFFIX))();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_@(package_name)
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, @(', '.join([package_name] + list(interface_path.parents[0].parts))), @(service.namespaced_type.name))() {
  if (!@(function_prefix)__@(service.namespaced_type.name)_service_type_support_handle.typesupport_identifier) {
    @(function_prefix)__@(service.namespaced_type.name)_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)@(function_prefix)__@(service.namespaced_type.name)_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, @(', '.join([package_name] + list(interface_path.parents[0].parts))), @(service.namespaced_type.name + SERVICE_REQUEST_MESSAGE_SUFFIX))()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, @(', '.join([package_name] + list(interface_path.parents[0].parts))), @(service.namespaced_type.name + SERVICE_RESPONSE_MESSAGE_SUFFIX))()->data;
  }

  return &@(function_prefix)__@(service.namespaced_type.name)_service_type_support_handle;
}
