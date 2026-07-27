""" Robot_tools Custom TB

Author: Carlo Dormeletti and Nishendra Singh
Copyright: 2026
Licence: LGPL 2.1
"""

# tb_commands imported for side effect:
# registers the RBT_* commands
from freecad.Robot_tools.Gui import rbt_commands  # noqa

import FreeCAD as App  # type: ignore
from freecad.Robot_tools.App.rbt_helpers_log import fcl_msg

tb_pstr = "User parameter:BaseApp/Workbench/Global/Toolbar"
tb_vers = 1.09
tb_name = "Robot Tools Toolbar"
pg_name = "Robot_tools"

tb_cmds = [("RBT_defrob", "RBT"),
           ("RBT_anrob", "RBT"),
           ("RBT_deftool", "RBT"),
           ("RBT_multictrl", "RBT")]

gtb_grp = App.ParamGet(tb_pstr)


def check_tb(tb_grp):
    """Check the toolbar existence."""
    grp_nm = tb_grp.GetGroupName()
    if tb_grp.GetString('Name') == tb_name:
        fcl_msg(f"RBT group: {grp_nm}\n")
        if check_vers(tb_grp) is True:
            return True
        else:
            create_tb(tb_grp)
    else:
        return False


def check_vers(tb_grp):
    """Check toolbar version."""
    vers = tb_grp.GetFloat('Vers')
    fcl_msg(f"RBT version: {vers}\n")
    if vers < tb_vers:
        fcl_msg("RBT version outdated\n")
        fcl_msg(f"new version {tb_vers}\n Recreating the ToolBar")
        tb_grp.Clear()
        return False
    fcl_msg("RBT version OK")
    return True


def create_tb(tb_grp):
    """Create the Toolbar."""
    tb_grp.SetString("Name", tb_name)
    tb_grp.SetBool("Active", True)
    tb_grp.SetFloat("Vers", tb_vers)
    # add commands
    for tb_cmd in tb_cmds:
        tb_grp.SetString(tb_cmd[0], tb_cmd[1])


# ------------------------------------------------
#                   init_toolbar
# ------------------------------------------------


tb_is_cust1 = False

if gtb_grp.HasGroup("Custom_1"):
    # print("Custom_1 group exists!")
    cust_grp = App.ParamGet(tb_pstr + "/Custom_1")
    # print(dir(cust_grp))
    if check_tb(cust_grp) is True:
        tb_is_cust1 = True
    else:
        print("Custom_1 is not our tb")

if tb_is_cust1 is False:
    if gtb_grp.HasGroup(pg_name):
        tb_grp = App.ParamGet(tb_pstr + "/" + pg_name)
        if check_tb(tb_grp) is True:
            pass
    else:
        tb_grp = App.ParamGet(tb_pstr + "/" + pg_name)
        create_tb(tb_grp)
