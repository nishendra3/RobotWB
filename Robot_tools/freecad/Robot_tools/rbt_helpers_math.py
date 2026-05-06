"""Robot math helper functions.

Name: rbt_helpers_math.py

See Changelog below.

Author: Nishendra Singh
Copyright: 2026
Licence: All right reserved
"""
__version__ = "0.01"
__build__ = "20260505_0828"

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