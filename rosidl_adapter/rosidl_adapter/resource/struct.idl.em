@#typedefs for arrays need to be defined outside of the struct
@{
from collections import OrderedDict
typedefs = OrderedDict()
def get_idl_type_identifier(idl_type):
    return idl_type.replace('::', '__').replace('[', '__').replace(']', '')
}@
@[for field in msg.fields]@
@{
idl_type = get_idl_type(field.type)
}@
@[  if field.type.is_fixed_size_array()]@
@{
idl_base_type = idl_type.split('[', 1)[0]
idl_base_type_identifier = idl_base_type.replace('::', '__')
# only necessary for complex types
if idl_base_type_identifier != idl_base_type:
    if idl_base_type_identifier not in typedefs:
        typedefs[idl_base_type_identifier] = idl_base_type
    else:
        assert typedefs[idl_base_type_identifier] == idl_base_type
idl_type_identifier = get_idl_type_identifier(idl_type) + '[' + str(field.type.array_size) + ']'
if idl_type_identifier not in typedefs:
    typedefs[idl_type_identifier] = idl_base_type_identifier
else:
    assert typedefs[idl_type_identifier] == idl_base_type_identifier
}@
@[  end if]@
@[end for]@
@[for k, v in typedefs.items()]@
    typedef @(v) @(k);
@[end for]@
@[if msg.constants]@
    module @(msg.msg_name)_Constants {
@[  for constant in msg.constants]@
      const @(get_idl_type(constant.type)) @(constant.name) = @(to_literal(get_idl_type(constant.type), constant.value));
@[  end for]@
    };
@[end if]@
@#
    struct @(msg.msg_name) {
@# use comments as docblocks once they are available
@[if msg.fields]@
@[  for field in msg.fields]@
@[    if field.default_value is not None]@
      @@default (value=@(to_literal(get_idl_type(field.type), field.default_value)))
@[    end if]@
@{
idl_type = get_idl_type(field.type)
}@
@[    if field.type.is_fixed_size_array()]@
@{
idl_type = get_idl_type_identifier(idl_type)
}@
@[    end if]@
      @(idl_type) @(field.name);
@[  end for]@
@[else]@
      boolean structure_needs_at_least_one_member;
@[end if]@
    };
