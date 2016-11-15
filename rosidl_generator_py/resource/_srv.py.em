# generated from rosidl_generator_py/resource/_srv.py.em
# generated code does not contain a copyright notice

import logging
import traceback


class Metaclass(type):
    """Metaclass of message '@(spec.srv_name)'."""

    _TYPE_SUPPORT = None

    @@classmethod
    def __import_type_support__(cls):
        try:
            import rclpy
            from rosidl_generator_py import import_type_support
            rclpy_implementation = rclpy._rclpy.rclpy_get_rmw_implementation_identifier()
            module = import_type_support(
                '@(package_name)', rclpy_implementation)
        except ImportError:
            logger = logging.getLogger('rosidl_generator_py.@(spec.srv_name)')
            logger.debug(
                'Failed to import needed modules for type support:\n' + traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv_@(module_name)
@{
srv_name = '_' + convert_camel_case_to_lower_case_underscore(spec.srv_name)
for field_name in [srv_name + '__request', srv_name + '__response']:
    print('%sfrom %s.srv import %s' % (' ' * 4 * 3, package_name, field_name))
    print('%sif %s.Metaclass._TYPE_SUPPORT is None:' % (' ' * 4 * 3, field_name))
    print('%s%s.Metaclass.__import_type_support__()' % (' ' * 4 * 4, field_name))
}@


class @(spec.srv_name)(metaclass=Metaclass):
    from @(package_name).srv._@convert_camel_case_to_lower_case_underscore(spec.srv_name)__request import @(spec.srv_name)_Request as Request
    from @(package_name).srv._@convert_camel_case_to_lower_case_underscore(spec.srv_name)__response import @(spec.srv_name)_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instanciated')
