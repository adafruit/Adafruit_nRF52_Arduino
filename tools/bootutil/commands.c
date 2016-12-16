#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>
#include "commands.h"
#include "serialport.h"
#include "infohelper.h"

extern int g_bootutil_gui_mode; /* Defined in main.c */

/*
static int
commands_stol(char *param_val, long min, long max, long *output, uint8_t b)
{
    char *endptr;
    long lval;

    if ((b != 10) && (b != 16)) {
        return EINVAL;
    }

    lval = strtol(param_val, &endptr, b);
    if (param_val != '\0' && *endptr == '\0' &&
        lval >= min && lval <= max) {
            *output = lval;
    } else {
        return EINVAL;
    }

    return 0;
}
*/

static int
commands_err_unknown_arg(char *arg_name)
{
    LOGERR("unknown argument \"%s\"\n",
            arg_name);
    return EINVAL;
}

int
commands_cmd_reset(void)
{
    /* Can't log data with -g option since stdout use kills ncurses  */
    if (g_bootutil_gui_mode == 0) {
        LOGINFO("attempting to reset (toggling DTR)\n");
    }

    /* Toggle the DTR line to provoke a reset */
    serialport_set_dtr(1);
    serialport_set_dtr(0);

    return 0;
}

int
commands_parse(char *argv)
{
    int i = 0;

    while(argv[i]) {
        argv[i] = tolower(argv[i]);
        i++;
    }

    if (strcmp(argv, "reset") == 0) {
        return commands_cmd_reset();
    } else {
        return commands_err_unknown_arg(argv);
    }
}
