import em
import os

from rosidl_parser import parse_message_file


def generate_rb(pkg_name, ros_interface_files, deps, output_dir, template_dir):
    mapping = {
        os.path.join(template_dir, 'msg-rb.c.template'): '%s-rb.c',
        os.path.join(template_dir, 'msg-rb.rb.template'): '%s.rb',
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
            print('Generating (Python): %s' % generated_file)

            try:
                # TODO only touch generated file if its content actually changes
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


MSG_TYPE_TO_RUBY = {
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

MSG_TYPE_TO_RUBY_CONVERT = {
    'bool': 'INT2BOOL',
    'byte': 'rb_str_new2',
    'char': 'rb_str_new2',
    'float32': 'rb_num2dbl',
    'float64': 'rb_num2dbl',
    'uint8': 'UINT2NUM',
    'int8': 'INT2NUM',
    'uint16': 'ULONG2NUM',
    'int16': 'LONG2NUM',
    'uint32': 'ULONG2NUM',
    'int32': 'LONG2NUM',
    'uint64': 'ULONG2NUM',
    'int64': 'LONG2NUM',
    'string': "rb_str_new2",
}

MSG_TYPE_TO_C_RUBY = {
    'bool': 'RTEST',
    'byte': 'NUM2CHAR',
    'char': 'NUM2CHAR',
    'float32': 'NUM2DBL',
    'float64': 'NUM2DBL',
    'uint8': 'NUM2CHR',
    'int8': 'NUM2CHR',
    'uint16': 'NUM2UINT',
    'int16': 'NUM2INT',
    'uint32': 'NUM2ULONG',
    'int32': 'NUM2LONG',
    'uint64': 'NUM2ULL',
    'int64': 'NUM2LL',
    'string': "rb_str_new_cstr",
}

def rb_to_c(type_):
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
        c_type = MSG_TYPE_TO_C_RUBY[type_.type]
    else:
        c_type = '%s_%s' % (type_.pkg_name, type_.type)
    return '%s' % (c_type)

def msg_type_to_rb_convert(type_):
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
        c_type = MSG_TYPE_TO_RUBY_CONVERT[type_.type]
    else:
        c_type = '%s_%s' % (type_.pkg_name, type_.type)
    return '%s' % (c_type)

def msg_type_to_rb(type_, name_):
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
        c_type = MSG_TYPE_TO_RUBY[type_.type]
    else:
        c_type = '%s_%s' % (type_.pkg_name, type_.type)

    if type_.is_array:
        if type_.array_size is None:
            return '%s * %s' % (c_type, name_)
        else:
            return '%s %s[%s]' % (c_type, name_, type_.array_size)
    else:
        return '%s %s' % (c_type, name_)
