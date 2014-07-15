import em
import os

from rosidl_parser import parse_message_file


def generate_dds_cpp(
        pkg_name, interface_files, deps, output_dir, template_dir):
    template_file = os.path.join(template_dir, 'msg__conversion.h.template')
    assert(os.path.exists(template_file))

    try:
        os.makedirs(output_dir)
    except FileExistsError:
        pass

    for idl_file in interface_files:
        spec = parse_message_file(pkg_name, idl_file)
        generated_file = os.path.join(
            output_dir, '%s__conversion.h' % spec.base_type.type)
        print('Generating: %s' % generated_file)

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

    return 0
