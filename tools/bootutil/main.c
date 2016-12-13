#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "serialport.h"

volatile int g_bootutil_serport_open = 0;

static int
bootutil_stol(char *param_val, long min, long max, long *output, uint8_t b)
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

static int
bootutil_err_too_few_args(char *cmd_name)
{
    printf("Error: too few arguments for command \"%s\"\n",
                   cmd_name);
    return EINVAL;
}

static int
bootutil_err_too_many_args(char *cmd_name)
{
    printf("Error: too many arguments for command \"%s\"\n",
                   cmd_name);
    return EINVAL;
}

static int
bootutil_err_unknown_arg(char *cmd_name)
{
    printf("Error: unknown argument \"%s\"\n",
                   cmd_name);
    return EINVAL;
}

static int
bootutil_err_invalid_arg(char *cmd_name)
{
    printf("Error: invalid argument \"%s\"\n",
                   cmd_name);
    return EINVAL;
}

static int
bootutil_help(void)
{
    printf("bootutil port cmd [flags...]\n");
    printf("cmd:\n");
    printf("  init\n");
    printf("  debug [verbose]\n");
    printf("  fill <color:0xFFFF>\n");
    printf("  p <x> <y> <color:0xFFFF>\n");

    return 0;
}

static int
bootutil_cmd_reset(int argc, char **argv)
{
    int rc;

    if (argc > 3) {
        return bootutil_err_too_many_args(argv[1]);
    }

    printf("Resetting `%s` (Toggling DTR)\n", argv[1]);

    /* Toggle the DTR line to provoke a reset */
    serialport_set_dtr(1);
    serialport_set_dtr(0);

    rc = 0;

error:
    return rc;
}

static int
bootutil_serport_open(int argc, char **argv)
{
    int serport;

    serport = serialport_open(argv[1], 115200);
    if (serport != 0) {
        g_bootutil_serport_open = 1;
    }

    return 0;
}

static int
bootutil_serport_close(void)
{
    int rc;

    rc = serialport_close();
    g_bootutil_serport_open = 0;

    return rc;
}

int
main(int argc, char **argv)
{
    int rc;

    if (argc == 1) {
        return bootutil_help();
    }

    /* Attempt to open the serial port */
    rc = bootutil_serport_open(argc, argv);
    if (g_bootutil_serport_open != 1) {
        goto error;
    }

    /* Command handlers */
    rc = 0;
    if (argc == 3 && strcmp(argv[2], "reset") == 0) {
        rc = bootutil_cmd_reset(argc, argv);
    } else {
        return bootutil_err_unknown_arg(argv[1]);
    }

    /* Giving an error warning if relevant */
    if (rc != 0) {
        printf("RC = 0x%04X (%u): %s\n", rc, rc, strerror(rc));
    }

error:
    bootutil_serport_close();
    return rc;
}
