import em
import os

from rosidl_parser import parse_message_file


def generate_cpp(pkg_name, ros_interface_files, deps, output_dir, template_dir):
    mapping = {
        os.path.join(template_dir, 'msg_TypeSupport_Introspection.cpp.template'): '%s_TypeSupport_Introspection.cpp',
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
            generated_file = os.path.join(output_dir, generated_filename % spec.base_type.type)
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
