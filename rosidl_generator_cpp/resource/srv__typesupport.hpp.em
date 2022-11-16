@# Included from rosidl_generator_c/resource/srv__type_support.hpp.em

template<typename T>
void * service_create_event_message(
  const rosidl_service_introspection_info_t * info,
  rcutils_allocator_t * allocator,
  const void * request_message,
  const void * response_message)
{
  if (nullptr == info) {
    throw std::invalid_argument("service introspection info struct cannot be null");
  }
  if (nullptr == allocator) {
    throw std::invalid_argument("allocator cannot be null");
  }
  auto * event_msg = static_cast<typename T::Event *>(allocator->allocate(sizeof(typename T::Event), allocator->state));
  if (nullptr == event_msg) {
    throw std::invalid_argument("allocation failed for service event message");
  }
  event_msg = new(event_msg) typename T::Event();

  event_msg->info.set__event_type(info->event_type);
  event_msg->info.set__sequence_number(info->sequence_number);
  event_msg->info.stamp.set__sec(info->stamp_sec);
  event_msg->info.stamp.set__nanosec(info->stamp_nanosec);

  std::array<uint8_t, 16> client_id;
  std::move(std::begin(info->client_id), std::end(info->client_id), client_id.begin());
  event_msg->info.client_id.set__uuid(client_id);

  if (nullptr != request_message) {
    event_msg->request.push_back(*static_cast<const typename T::Request *>(request_message));
  }
  if (nullptr != response_message) {
    event_msg->response.push_back(*static_cast<const typename T::Response *>(response_message));
  }
  return event_msg;
}

template<typename T>
bool service_destroy_event_message(
  void * event_msg,
  rcutils_allocator_t * allocator)
{
  auto * event_msg_ = static_cast<typename T::Event *>(event_msg);
  using EventT = typename T::Event;
  event_msg_->~EventT();
  allocator->deallocate(event_msg, allocator->state);
  return true;
}
