#ifndef __rosidl_generator_dds_cpp__Resolver__h__
#define __rosidl_generator_dds_cpp__Resolver__h__

namespace dds_impl
{

template<typename T>
struct DDSTypeResolver
{
  typedef void* DDSMsgType;
  typedef void* DDSMsgSeqType;
  typedef void* DDSMsgSeqType_var;
  typedef void* DDSMsgTypeSupportType;
  typedef void* DDSMsgTypeSupportType_var;
  typedef void* DDSMsgDataWriter;
  typedef void* DDSMsgDataWriter_var;
  typedef void* DDSMsgDataReader;
  typedef void* DDSMsgDataReader_var;

  static void convert_ros_message_to_dds(void*, void*);

  static void convert_dds_message_to_ros(void*, void*);
};

}  // namespace dds_impl

#endif  // __rosidl_generator_dds_cpp__Resolver__h__
