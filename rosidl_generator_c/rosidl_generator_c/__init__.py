import em
import os

from rosidl_parser import parse_message_file


def generate_c(pkg_name, ros_interface_files, deps, output_dir, template_dir):
    mapping = {
        os.path.join(template_dir, 'msg-c.h.template'): '%s-c.h',
        os.path.join(template_dir, 'msg_Struct-c.h.template'): '%s_Struct-c.h',
    }
    for template_file in mapping.keys():
        assert(os.path.exists(template_file))

    try:
        os.makedirs(output_dir)
    except FileExistsError:
        pass

    for ros_interface_file in ros_interface_files:
        spec = parse_message_file(pkg_name, ros_interface_file)
        for template_file, generated_filename in mapping.items():
            generated_file = os.path.join(
                output_dir, generated_filename % spec.base_type.type)
            print('Generating (C): %s' % generated_file)

            try:
                # TODO only write generated file if its different
                ofile = open(generated_file, 'w')
                # TODO reuse interpreter
                interpreter = em.Interpreter(
                    output=ofile,
                    options={
                        em.RAW_OPT: True,
                        em.BUFFERED_OPT: True,
                    },
                    globals={'spec': spec},
                )
                interpreter.file(open(template_file))
                interpreter.shutdown()
            except Exception:
                os.remove(generated_file)
                raise

    return 0


MSG_TYPE_TO_C = {
    'bool': 'bool',
    'byte': 'uint8_t',
    'char': 'char',
    'float32': 'float',
    'float64': 'double',
    'uint8': 'uint8_t',
    'int8': 'int8_t',
    'uint16': 'uint16_t',
    'int16': 'int16_t',
    'uint32': 'uint32_t',
    'int32': 'int32_t',
    'uint64': 'uint64_t',
    'int64': 'int64_t',
    'string': "char *",
}


def msg_type_to_c(type_, name_, containing_msg_name='notset'):
    """
    Convert a message type into the C declaration.

    Example input: uint32, std_msgs/String
    Example output: uint32_t, char *

    @param type_: The message type
    @type type_: rosidl_parser.Type
    @param type_: The field name
    @type type_: str
    """
    c_type = None
    if type_.is_primitive_type():
        c_type = MSG_TYPE_TO_C[type_.type]
    else:
        c_type = '%s__%s' % (type_.pkg_name, type_.type)

    if type_.is_array:
        if type_.array_size is None:
            # Dynamic sized array
            return 'ROSIDL_Array__%s\n  %s' % (type_.type, name_)
        else:
            # Static sized array (field specific)
            return 'ROSIDL_Array__%s__%s\n  %s' % \
                (containing_msg_name, name_, name_)
    else:
        return '%s %s' % (c_type, name_)
