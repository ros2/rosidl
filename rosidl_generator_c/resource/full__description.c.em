@# Included from rosidl_generator_c/resource/idl__description.c.em
@{
from rosidl_generator_c import escape_string
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
}@

@[for itype_description in all_type_descriptions]@
static char @(typename_to_c(itype_description['type_name']))__TYPE_NAME[] = "@(itype_description['type_name'])";
@[end for]@

@[for msg in full_type_descriptions]@
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
  *@(typename_to_c(ref_td['type_name']))__@(GET_DESCRIPTION_FUNC)(),
@[    end for]@
};
@[  end if]@

const rosidl_runtime_c__type_description__TypeDescription *
@(td_c_typename)__@(GET_DESCRIPTION_FUNC)()
{
  // static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      @(static_seq(f'{td_c_typename}__TYPE_NAME', td_typename)),
      @(static_seq(f'{td_c_typename}__FIELDS', msg['type_description']['fields'])),
    },
    @(static_seq(f'{td_c_typename}__REFERENCED_TYPE_DESCRIPTIONS', ref_tds)),
  };
  return &description;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
@(td_c_typename)__@(GET_SOURCES_FUNC)()
{
@# TODO(emersonknapp) Implement raw source code embedding/generation. This sequence is left empty for now.
  static const rosidl_runtime_c__type_description__TypeSource__Sequence sources = @(static_seq(None, ''));
  return &sources;
}
@[end for]@
