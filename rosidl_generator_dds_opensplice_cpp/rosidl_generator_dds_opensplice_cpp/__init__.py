import copy
import em
import os
import subprocess

from rosidl_parser import parse_message_file


def generate_dds_opensplice_cpp(
        pkg_name, interface_files, deps, output_dir, ospl_home,
        ospl_tmpl_path, idl_pp):
    try:
        os.makedirs(output_dir)
    except FileExistsError:
        pass

    include_dirs = []
    for dep in deps:
        dep_parts = dep.split(':')
        assert(len(dep_parts) == 2)
        idl_path = dep_parts[1]
        idl_base_path = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.normpath(idl_path))))
        if idl_base_path not in include_dirs:
            include_dirs.append(idl_base_path)

    for idl_file in interface_files:
        generated_file = os.path.join(
            output_dir,
            os.path.splitext(os.path.basename(idl_file))[0] + '.h/cpp')
        print('Generating: %s' % generated_file)

        cmd = [idl_pp]
        for include_dir in include_dirs:
            cmd += ['-I', include_dir]
        cmd += [
            '-S',
            '-l', 'cpp',
            '-o', 'dds-types',
            '-d', output_dir,
            idl_file
        ]
        env = copy.copy(os.environ)
        env['OSPL_HOME'] = ospl_home
        env['OSPL_TMPL_PATH'] = ospl_tmpl_path

        subprocess.check_call(cmd, env=env)

    # for idl_file in interface_files:
    #     spec = parse_message_file(pkg_name, idl_file)
    #     generated_file = os.path.join(output_dir, spec.base_type.type + '.idl')
    #     print('Generating: %s' % generated_file)

    #     # TODO only touch generated file if its content actually changes
    #     ofile = open(generated_file, 'w')
    #     # TODO reuse interpreter
    #     interpreter = em.Interpreter(
    #         output=ofile,
    #         options={
    #             em.RAW_OPT: True,
    #             em.BUFFERED_OPT: True,
    #         },
    #         globals={'spec': spec},
    #     )
    #     interpreter.file(open(template_file))
    #     interpreter.shutdown()

    return 0
