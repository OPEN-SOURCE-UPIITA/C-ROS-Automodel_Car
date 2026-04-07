# generated from rosidl_generator_py/resource/_idl.py.em
# with input from motor_msgs:msg/EncoderData.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_EncoderData(type):
    """Metaclass of message 'EncoderData'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('motor_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'motor_msgs.msg.EncoderData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__encoder_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__encoder_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__encoder_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__encoder_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__encoder_data

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class EncoderData(metaclass=Metaclass_EncoderData):
    """Message class 'EncoderData'."""

    __slots__ = [
        '_vel_m1',
        '_dir_m1',
        '_vel_m2',
        '_dir_m2',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'vel_m1': 'int32',
        'dir_m1': 'int8',
        'vel_m2': 'int32',
        'dir_m2': 'int8',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.vel_m1 = kwargs.get('vel_m1', int())
        self.dir_m1 = kwargs.get('dir_m1', int())
        self.vel_m2 = kwargs.get('vel_m2', int())
        self.dir_m2 = kwargs.get('dir_m2', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.vel_m1 != other.vel_m1:
            return False
        if self.dir_m1 != other.dir_m1:
            return False
        if self.vel_m2 != other.vel_m2:
            return False
        if self.dir_m2 != other.dir_m2:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def vel_m1(self):
        """Message field 'vel_m1'."""
        return self._vel_m1

    @vel_m1.setter
    def vel_m1(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'vel_m1' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'vel_m1' field must be an integer in [-2147483648, 2147483647]"
        self._vel_m1 = value

    @builtins.property
    def dir_m1(self):
        """Message field 'dir_m1'."""
        return self._dir_m1

    @dir_m1.setter
    def dir_m1(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'dir_m1' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'dir_m1' field must be an integer in [-128, 127]"
        self._dir_m1 = value

    @builtins.property
    def vel_m2(self):
        """Message field 'vel_m2'."""
        return self._vel_m2

    @vel_m2.setter
    def vel_m2(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'vel_m2' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'vel_m2' field must be an integer in [-2147483648, 2147483647]"
        self._vel_m2 = value

    @builtins.property
    def dir_m2(self):
        """Message field 'dir_m2'."""
        return self._dir_m2

    @dir_m2.setter
    def dir_m2(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'dir_m2' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'dir_m2' field must be an integer in [-128, 127]"
        self._dir_m2 = value
