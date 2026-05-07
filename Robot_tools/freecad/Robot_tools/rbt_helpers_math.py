"""Robot math helper functions.

Name: rbt_helpers_math.py

See Changelog below.

Author: Carlo Dormeletti and Nishendra Singh
Copyright: 2026
Licence:  LGPL 2.1
"""
__version__ = "0.01"
__build__ = "20260507_1255"

# ------------------------------------------------
#               Service functions
# ------------------------------------------------


def roundvec(r_vec, prec=6):
    """Round value in vectors."""
    vlist = [round(r_vec.x, prec), round(r_vec.y, prec), round(r_vec.z, prec)]
    return vlist


def roundrot(r_rot, prec=6):
    """Round value in vectors."""
    vlist = [round(r_rot[0], prec), round(r_rot[1], prec), round(r_rot[2], prec)]
    return vlist
