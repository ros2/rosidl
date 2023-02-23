// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from @(package_name):@(interface_path)
// generated code does not contain a copyright notice
@
@#######################################################################
@# EmPy template for generating <idl>__description.c files
@#
@# Context:
@#  - package_name (string)
@#  - interface_path (Path relative to the directory named after the package)
@#  - content (IdlContent, list of elements, e.g. Messages or Services)
@#######################################################################
@{
from rosidl_generator_c import escape_string
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore

subinterfaces = type_description_info['subinterfaces']

type_description_struct = type_description_info['type_description_msg']
toplevel_type_description = type_description_struct['type_description']
referenced_type_descriptions = type_description_struct['referenced_type_descriptions']
all_type_descriptions = [toplevel_type_description] + referenced_type_descriptions

message_typename = toplevel_type_description['type_name'].replace('/', '__')
include_parts = [package_name] + list(interface_path.parents[0].parts) + [
    'detail', convert_camel_case_to_lower_case_underscore(interface_path.stem)]
include_base = '/'.join(include_parts)
}@

#include "@(include_base)__struct.h"

#define STATIC_SEQ(x) {x, sizeof(x), sizeof(x)}
#define NULL_SEQ {NULL, 0, 0}

// Declare and define all type names, field names, and default values
@[for itype_description in all_type_descriptions]@
@{  td_typename = itype_description['type_name'].replace('/', '__') }@
static char @(td_typename)__TYPE_NAME[] = "@(itype_description['type_name'])";
@[  for field in itype_description['fields']]@
static char @(td_typename)__FIELD_NAME__@(field['name'])[] = "@(field['name'])";
@[    if field['default_value']]@
static char @(td_typename)__DEFAULT_VALUE__@(field['name'])[] = "@(escape_string(field['default_value']))";
@[    end if]@
@[  end for]@

@[end for]@

// Define all arrays of fields
@[for itype_description in all_type_descriptions]@
@{  td_typename = itype_description['type_name'].replace('/', '__') }@
@[  if itype_description['fields']]@
static rosidl_runtime_c__type_description__Field @(td_typename)__FIELDS[] = {
@[    for field in itype_description['fields']]@
  {
    STATIC_SEQ(@(td_typename)__FIELD_NAME__@(field['name'])),
    {
      @(field['type']['type_id']),  // TODO(ek) should this be named to FieldType const, or is value ok?
      @(field['type']['capacity']),
      @(field['type']['string_capacity']),
@[      if field['type']['nested_type_name']]@
      STATIC_SEQ(@(field['type']['nested_type_name'].replace('/', '__'))__TYPE_NAME),
@[      else]@
      NULL_SEQ,
@[      end if]@
    },
@[      if field['default_value']]@
    STATIC_SEQ(@(td_typename)__DEFAULT_VALUE__@(field['name'])),
@[      else]@
    NULL_SEQ,
@[      end if]@
  },
@[    end for]@
};
@[  end if]@

@[end for]@

// Define all IndividualTypeDescriptions
@[for itype_description in all_type_descriptions]@
@{ td_typename = itype_description['type_name'].replace('/', '__') }@
static const rosidl_runtime_c__type_description__IndividualTypeDescription @(td_typename)__INDIVIDUAL_TYPE_DESCRIPTION = {
  STATIC_SEQ(@(td_typename)__TYPE_NAME),
@[  if itype_description['fields']]@
  STATIC_SEQ(@(td_typename)__FIELDS),
@[  else]@
  NULL_SEQ,
@[  end if]@
};

@[end for]@

// Define all TypeDescriptions
@[for subinterface_typename, ref_tds in subinterfaces.items()]@
@{  subinterface_symbol = subinterface_typename.replace('/', '__') }@
@[  if ref_tds]@
static rosidl_runtime_c__type_description__IndividualTypeDescription @(subinterface_symbol)__REFERENCED_TYPE_DESCRIPTIONS[] = {
@[    for ref_td_typename in ref_tds]@
  @(ref_td_typename.replace('/', '__'))__INDIVIDUAL_TYPE_DESCRIPTION,
@[    end for]@
};
@[  end if]@

const rosidl_runtime_c__type_description__TypeDescription @(subinterface_symbol)__TYPE_DESCRIPTION = {
  @(subinterface_symbol)__INDIVIDUAL_TYPE_DESCRIPTION,
@[  if ref_tds]@
  STATIC_SEQ(@(subinterface_symbol)__REFERENCED_TYPE_DESCRIPTIONS),
@[  else]@
  NULL_SEQ,
@[  end if]@
};

@[end for]@
