import em
import os

from rosidl_parser import parse_message_file


def generate_dds_idl(
        pkg_name, interface_files, deps, output_dir, template_dir):
    template_file = os.path.join(template_dir, 'msg.idl.template')
    assert(os.path.exists(template_file))

    try:
        os.makedirs(output_dir)
    except FileExistsError:
        pass

    for idl_file in interface_files:
        spec = parse_message_file(pkg_name, idl_file)
        generated_file = os.path.join(output_dir, '%s_.idl' % spec.base_type.type)
        print('Generating: %s' % generated_file)

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


# used by the template
MSG_TYPE_TO_IDL = {
    'bool': 'boolean',
    'byte': 'octet',
    'char': 'char',
    'int8': 'octet',
    'uint8': 'char',
    'int16': 'short',
    'uint16': 'unsigned short',
    'int32': 'long',
    'uint32': 'unsigned long',
    'int64': 'long long',
    'uint64': 'unsigned long long',
    'float32': 'float',
    'float64': 'double',
    'string': 'string',
    #'time': 'DDS::Time_t',
    #'duration': 'DDS::Duration_t'
}


# used by the template
def msg_type_to_idl(type_):
    """
    Convert a message type into the DDS declaration.

    Example input: uint32, std_msgs/String
    Example output: uint32_t, std_msgs::String_<ContainerAllocator>

    @param type: The message type
    @type type: rosidl_parser.Type
    """
    idl_type = None
    if type_.is_primitive_type():
        idl_type = MSG_TYPE_TO_IDL[type_.type]
    else:
        idl_type = '%s::dds_idl::%s_' % (type_.pkg_name, type_.type)

    if type_.is_array:
        if type_.array_size is None:
            return ['', '', 'sequence<%s>' % idl_type]
        else:
            typename = '%s_array_%s' % \
                (idl_type.replace(' ', '_'), type_.array_size)
            return [
                'typedef %s' % idl_type,
                '%s[%s];' % (typename, type_.array_size),
                '%s' % typename
            ]
    else:
        return ['', '', idl_type]
