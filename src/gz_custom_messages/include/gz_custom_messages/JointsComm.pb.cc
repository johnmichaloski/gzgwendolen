// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: JointsComm.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "JointsComm.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace message {

namespace {

const ::google::protobuf::Descriptor* JointsComm_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  JointsComm_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_JointsComm_2eproto() {
  protobuf_AddDesc_JointsComm_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "JointsComm.proto");
  GOOGLE_CHECK(file != NULL);
  JointsComm_descriptor_ = file->message_type(0);
  static const int JointsComm_offsets_[4] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(JointsComm, name_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(JointsComm, position_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(JointsComm, velocity_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(JointsComm, effort_),
  };
  JointsComm_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      JointsComm_descriptor_,
      JointsComm::default_instance_,
      JointsComm_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(JointsComm, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(JointsComm, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(JointsComm));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_JointsComm_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    JointsComm_descriptor_, &JointsComm::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_JointsComm_2eproto() {
  delete JointsComm::default_instance_;
  delete JointsComm_reflection_;
}

void protobuf_AddDesc_JointsComm_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\020JointsComm.proto\022\007message\"N\n\nJointsCom"
    "m\022\014\n\004name\030\001 \003(\t\022\020\n\010position\030\002 \003(\001\022\020\n\010vel"
    "ocity\030\003 \003(\001\022\016\n\006effort\030\004 \003(\001", 107);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "JointsComm.proto", &protobuf_RegisterTypes);
  JointsComm::default_instance_ = new JointsComm();
  JointsComm::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_JointsComm_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_JointsComm_2eproto {
  StaticDescriptorInitializer_JointsComm_2eproto() {
    protobuf_AddDesc_JointsComm_2eproto();
  }
} static_descriptor_initializer_JointsComm_2eproto_;

// ===================================================================

#ifndef _MSC_VER
const int JointsComm::kNameFieldNumber;
const int JointsComm::kPositionFieldNumber;
const int JointsComm::kVelocityFieldNumber;
const int JointsComm::kEffortFieldNumber;
#endif  // !_MSC_VER

JointsComm::JointsComm()
  : ::google::protobuf::Message() {
  SharedCtor();
  // @@protoc_insertion_point(constructor:message.JointsComm)
}

void JointsComm::InitAsDefaultInstance() {
}

JointsComm::JointsComm(const JointsComm& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:message.JointsComm)
}

void JointsComm::SharedCtor() {
  ::google::protobuf::internal::GetEmptyString();
  _cached_size_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

JointsComm::~JointsComm() {
  // @@protoc_insertion_point(destructor:message.JointsComm)
  SharedDtor();
}

void JointsComm::SharedDtor() {
  if (this != default_instance_) {
  }
}

void JointsComm::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* JointsComm::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return JointsComm_descriptor_;
}

const JointsComm& JointsComm::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_JointsComm_2eproto();
  return *default_instance_;
}

JointsComm* JointsComm::default_instance_ = NULL;

JointsComm* JointsComm::New() const {
  return new JointsComm;
}

void JointsComm::Clear() {
  name_.Clear();
  position_.Clear();
  velocity_.Clear();
  effort_.Clear();
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool JointsComm::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:message.JointsComm)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated string name = 1;
      case 1: {
        if (tag == 10) {
         parse_name:
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->add_name()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->name(this->name_size() - 1).data(),
            this->name(this->name_size() - 1).length(),
            ::google::protobuf::internal::WireFormat::PARSE,
            "name");
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(10)) goto parse_name;
        if (input->ExpectTag(17)) goto parse_position;
        break;
      }

      // repeated double position = 2;
      case 2: {
        if (tag == 17) {
         parse_position:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 17, input, this->mutable_position())));
        } else if (tag == 18) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_position())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(17)) goto parse_position;
        if (input->ExpectTag(25)) goto parse_velocity;
        break;
      }

      // repeated double velocity = 3;
      case 3: {
        if (tag == 25) {
         parse_velocity:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 25, input, this->mutable_velocity())));
        } else if (tag == 26) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_velocity())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(25)) goto parse_velocity;
        if (input->ExpectTag(33)) goto parse_effort;
        break;
      }

      // repeated double effort = 4;
      case 4: {
        if (tag == 33) {
         parse_effort:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 33, input, this->mutable_effort())));
        } else if (tag == 34) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_effort())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(33)) goto parse_effort;
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:message.JointsComm)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:message.JointsComm)
  return false;
#undef DO_
}

void JointsComm::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:message.JointsComm)
  // repeated string name = 1;
  for (int i = 0; i < this->name_size(); i++) {
  ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
    this->name(i).data(), this->name(i).length(),
    ::google::protobuf::internal::WireFormat::SERIALIZE,
    "name");
    ::google::protobuf::internal::WireFormatLite::WriteString(
      1, this->name(i), output);
  }

  // repeated double position = 2;
  for (int i = 0; i < this->position_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      2, this->position(i), output);
  }

  // repeated double velocity = 3;
  for (int i = 0; i < this->velocity_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      3, this->velocity(i), output);
  }

  // repeated double effort = 4;
  for (int i = 0; i < this->effort_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      4, this->effort(i), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:message.JointsComm)
}

::google::protobuf::uint8* JointsComm::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:message.JointsComm)
  // repeated string name = 1;
  for (int i = 0; i < this->name_size(); i++) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->name(i).data(), this->name(i).length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "name");
    target = ::google::protobuf::internal::WireFormatLite::
      WriteStringToArray(1, this->name(i), target);
  }

  // repeated double position = 2;
  for (int i = 0; i < this->position_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleToArray(2, this->position(i), target);
  }

  // repeated double velocity = 3;
  for (int i = 0; i < this->velocity_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleToArray(3, this->velocity(i), target);
  }

  // repeated double effort = 4;
  for (int i = 0; i < this->effort_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleToArray(4, this->effort(i), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:message.JointsComm)
  return target;
}

int JointsComm::ByteSize() const {
  int total_size = 0;

  // repeated string name = 1;
  total_size += 1 * this->name_size();
  for (int i = 0; i < this->name_size(); i++) {
    total_size += ::google::protobuf::internal::WireFormatLite::StringSize(
      this->name(i));
  }

  // repeated double position = 2;
  {
    int data_size = 0;
    data_size = 8 * this->position_size();
    total_size += 1 * this->position_size() + data_size;
  }

  // repeated double velocity = 3;
  {
    int data_size = 0;
    data_size = 8 * this->velocity_size();
    total_size += 1 * this->velocity_size() + data_size;
  }

  // repeated double effort = 4;
  {
    int data_size = 0;
    data_size = 8 * this->effort_size();
    total_size += 1 * this->effort_size() + data_size;
  }

  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void JointsComm::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const JointsComm* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const JointsComm*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void JointsComm::MergeFrom(const JointsComm& from) {
  GOOGLE_CHECK_NE(&from, this);
  name_.MergeFrom(from.name_);
  position_.MergeFrom(from.position_);
  velocity_.MergeFrom(from.velocity_);
  effort_.MergeFrom(from.effort_);
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void JointsComm::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void JointsComm::CopyFrom(const JointsComm& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool JointsComm::IsInitialized() const {

  return true;
}

void JointsComm::Swap(JointsComm* other) {
  if (other != this) {
    name_.Swap(&other->name_);
    position_.Swap(&other->position_);
    velocity_.Swap(&other->velocity_);
    effort_.Swap(&other->effort_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata JointsComm::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = JointsComm_descriptor_;
  metadata.reflection = JointsComm_reflection_;
  return metadata;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace message

// @@protoc_insertion_point(global_scope)
