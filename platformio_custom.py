#
# This works around a bug in the print_cmd_line shipped with platformio/scons 4.11.1
import sys
from SCons.Script import Import

# Access the PlatformIO/SCons construction environment
Import("env")

def my_print_cmd_line(s, target, source, env):
    """
    Custom print function to override noisy compiler commands.
    """
    sys.stdout.write(f"{s}\n")

env['PRINT_CMD_LINE_FUNC'] = my_print_cmd_line
