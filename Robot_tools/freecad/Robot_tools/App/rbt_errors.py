"""
rbt_errors.py - error exception types for robot wb
"""


class RbtError(Exception):
    """
    base class for robot wb errors
    """


class RbtInputError(RbtError, ValueError):
    """
    bad input value to the caller
    """


class RbtDocError(RbtError):
    """
    the FC Document or robot model is not usable
    """
