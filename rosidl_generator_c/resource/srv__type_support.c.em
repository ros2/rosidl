@# Included from rosidl_generator_c/resource/srv__type_support.c.em
@{
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore
from rosidl_parser.definition import SERVICE_REQUEST_MESSAGE_SUFFIX
from rosidl_parser.definition import SERVICE_RESPONSE_MESSAGE_SUFFIX
from rosidl_parser.definition import SERVICE_EVENT_MESSAGE_SUFFIX
event_type = '__'.join([package_name, *interface_path.parents[0].parts, service.namespaced_type.name]) + SERVICE_EVENT_MESSAGE_SUFFIX
request_type = '__'.join([package_name, *interface_path.parents[0].parts, service.namespaced_type.name]) + SERVICE_REQUEST_MESSAGE_SUFFIX
response_type = '__'.join([package_name, *interface_path.parents[0].parts, service.namespaced_type.name]) + SERVICE_RESPONSE_MESSAGE_SUFFIX
}
void *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  @(',\n  '.join(service.namespaced_type.namespaced_name()))
)(
  const rosidl_service_introspection_info_t * info,
  rcutils_allocator_t * allocator,
  const void * request_message,
  const void * response_message)
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

// Forward declare the get type support functions for this type.
bool
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  @(',\n  '.join(service.namespaced_type.namespaced_name()))
)(
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
