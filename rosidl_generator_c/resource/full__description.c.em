@# Included from rosidl_generator_c/resource/idl__description.c.em
@{
from rosidl_generator_c import escape_string
from rosidl_generator_c import idl_structure_type_to_c_include_prefix
from rosidl_parser.definition import NamespacedType
from rosidl_generator_type_description import FIELD_TYPE_ID_TO_NAME
from rosidl_generator_type_description import GET_DESCRIPTION_FUNC
from rosidl_generator_type_description import GET_SOURCES_FUNC

def typename_to_c(typename):
  return typename.replace('/', '__')

def static_seq(varname, values):
  """Statically define a runtime Sequence or String type."""
  if values:
    return f'{{{varname}, {len(values)}, {len(values)}}}'
  return '{NULL, 0, 0}'

def represent_default(default_value):
  # Encode to UTF-8 in case of WStrings, then remove the b'' from the representation.
  return repr(default_value.encode('utf-8'))[2:-1]

implicit_type_names = set(td['type_description']['type_name'] for td, _ in implicit_type_descriptions)
includes = set()
toplevel_msg, _ = toplevel_type_description

for referenced_td in toplevel_msg['referenced_type_descriptions']:
    if referenced_td['type_name'] in implicit_type_names:
        continue
    names = referenced_td['type_name'].split('/')
    _type = NamespacedType(names[:-1], names[-1])
    include_prefix = idl_structure_type_to_c_include_prefix(_type, 'detail')
    includes.add(include_prefix + '__functions.h')

full_type_descriptions = [toplevel_type_description] + implicit_type_descriptions
all_type_descriptions = [toplevel_msg['type_description']] + toplevel_msg['referenced_type_descriptions']
}@

// Include directives for referenced types
@[for header_file in includes]@
#include "@(header_file)"
@[end for]@

// Names for all types
@[for itype_description in all_type_descriptions]@
static char @(typename_to_c(itype_description['type_name']))__TYPE_NAME[] = "@(itype_description['type_name'])";
@[end for]@

@[for msg, interface_type in full_type_descriptions]@
@{
itype_description = msg['type_description']
td_typename = itype_description['type_name']
td_c_typename = typename_to_c(td_typename)
ref_tds = msg['referenced_type_descriptions']
}@
@
// Define type names, field names, and default values
@[  for field in itype_description['fields']]@
static char @(td_c_typename)__FIELD_NAME__@(field['name'])[] = "@(field['name'])";
@[    if field['default_value']]@
static char @(td_c_typename)__DEFAULT_VALUE__@(field['name'])[] = "@(escape_string(represent_default(field['default_value'])))";
@[    end if]@
@[  end for]@

/// Define arrays of Fields
@
@[  if itype_description['fields']]@
static rosidl_runtime_c__type_description__Field @(td_c_typename)__FIELDS[] = {
@[    for field in itype_description['fields']]@
  {
    @(static_seq(f"{td_c_typename}__FIELD_NAME__{field['name']}", field['name'])),
    {
      rosidl_runtime_c__type_description__FieldType__@(FIELD_TYPE_ID_TO_NAME[field['type']['type_id']]),
      @(field['type']['capacity']),
      @(field['type']['string_capacity']),
      @(static_seq(f"{typename_to_c(field['type']['nested_type_name'])}__TYPE_NAME", field['type']['nested_type_name'])),
    },
    @(static_seq(f"{td_c_typename}__DEFAULT_VALUE__{field['name']}", field['default_value'])),
  },
@[    end for]@
};
@[  end if]@

/// Define exported TypeDescription and TypeSources
@[  if ref_tds]@
static rosidl_runtime_c__type_description__IndividualTypeDescription @(td_c_typename)__REFERENCED_TYPE_DESCRIPTIONS[] = {
@[    for ref_td in ref_tds]@
  {
    @(static_seq(f"{typename_to_c(ref_td['type_name'])}__TYPE_NAME", ref_td['type_name'])),
    {NULL, 0, 0},
  },
@[    end for]@
};
@[  end if]@

const rosidl_runtime_c__type_description__TypeDescription *
@(td_c_typename)__@(GET_DESCRIPTION_FUNC)(const rosidl_@(interface_type)_type_support_t *)
{
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      @(static_seq(f'{td_c_typename}__TYPE_NAME', td_typename)),
      @(static_seq(f'{td_c_typename}__FIELDS', msg['type_description']['fields'])),
    },
    @(static_seq(f'{td_c_typename}__REFERENCED_TYPE_DESCRIPTIONS', ref_tds)),
  };
  if (!constructed) {
    // TODO(ek) check hashes for consistency
@[  for idx, ref_td in enumerate(ref_tds)]@
    {
      const rosidl_runtime_c__type_description__TypeDescription * ref_desc = @(typename_to_c(ref_td['type_name']))__@(GET_DESCRIPTION_FUNC)(NULL);
      description.referenced_type_descriptions.data[@(idx)].fields.data = ref_desc->type_description.fields.data;
      description.referenced_type_descriptions.data[@(idx)].fields.size = ref_desc->type_description.fields.size;
      description.referenced_type_descriptions.data[@(idx)].fields.capacity = ref_desc->type_description.fields.capacity;
    }
@[  end for]@
    constructed = true;
  }
  return &description;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
@(td_c_typename)__@(GET_SOURCES_FUNC)(const rosidl_@(interface_type)_type_support_t *)
{
@# TODO(ek) Implement raw source code embedding/generation. This sequence is left empty for now.
  static const rosidl_runtime_c__type_description__TypeSource__Sequence sources = @(static_seq(None, ''));
  return &sources;
}
@[end for]@
