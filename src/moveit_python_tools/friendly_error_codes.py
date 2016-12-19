#!/usr/bin/env python

from moveit_msgs.msg import MoveItErrorCodes

"""
To ease use of the MoveItErrorCodes
we have them in a dictionary to easily translate
from their code to their string representation.

print moveit_error_dict[-31]
Would give 'NO_IK_SOLUTION'
"""

# Build a useful mapping from MoveIt error codes to error names
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name
