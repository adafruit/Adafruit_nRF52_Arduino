#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <limits.h>
#include <ctype.h>
#include "serialport.h"

#ifdef _WIN32
char *progname = "bootutil.exe";
#else
char *progname = "bootutil";
#endif

volatile int g_bootutil_serport_open = 0;

/*
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
*/

static int
bootutil_err_too_many_instances_arg(char *arg_name)
{
    printf("Error: multiple instances of \"%s\"\n",
                   arg_name);
    return EINVAL;
}

static int
bootutil_err_missing_arg(char *arg_name)
{
    printf("Error: missing argument \"%s\"\n",
                   arg_name);
    return EINVAL;
}

static int
bootutil_err_unknown_arg(char *arg_name)
{
    printf("Error: unknown argument \"%s\"\n",
                   arg_name);
    return EINVAL;
}

static int
bootutil_err_invalid_arg(char *arg_name)
{
    printf("Error: invalid argument \"%s\"\n",
                   arg_name);
    return EINVAL;
}

static void
bootutil_help(int rc)
{
    printf("Bootloader utility for the Adafruit nRF52 board family.\n");
    printf("\n");
    printf("Usage: %s -d [TTY.DEVICE] -chr?\n", progname);
    printf("\n");
    printf("  -d [TTY.DEVICE] : serial/tty port name (mandatory)\n");
    printf("  -c [command]    : execute the specified command\n");
    printf("     reset        : perform a system reset (toggle DTR)\n");
    printf("  -h              : show help\n");
    printf("  -r              : perform a system reset (toggle DTR)\n");
    printf("  -?              : show help\n");
    printf("\n");
    printf("Ex: $ %s -d /dev/tty.SLAB_USBtoUART -r\n", progname);

    exit(rc);
}

static int
bootutil_cmd_reset(void)
{
    printf("Attempting to reset (toggling DTR)\n");

    /* Toggle the DTR line to provoke a reset */
    serialport_set_dtr(1);
    serialport_set_dtr(0);

    return 0;
}

static int
bootutil_cmd_parse(char *argv)
{
    int i = 0;

    while(argv[i]) {
        argv[i] = tolower(argv[i]);
        i++;
    }

    if (strcmp(argv, "reset") == 0) {
        return bootutil_cmd_reset();
    } else {
        return bootutil_err_unknown_arg(argv);
    }
}

static int
bootutil_serport_open(char *argv)
{
    int serport;

    serport = serialport_open(argv, 115200);
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
    int ch;
    int rc = -1;
    int rstflag = 0;
    int cmdflag = 0;
    char cmd[256];
    char tty[PATH_MAX + NAME_MAX];

    memset(cmd, '\0', sizeof(cmd));
    memset(tty, '\0', sizeof(cmd));

    /* Display help menu on zero params */
    if (argc == 1) {
        bootutil_help(0);
    }

    /* Parse input params */
    while ((ch = getopt(argc, argv, "d:c:rh")) != -1) {
        switch (ch) {
        case 'd':
            strncpy(tty, optarg, sizeof(tty)-1);
            break;
        case 'c':
            /* Check if we've already received a command */
            if (cmdflag == 1) {
                exit(bootutil_err_too_many_instances_arg("-c"));
            }
            cmdflag = 1;
            strncpy(cmd, optarg, sizeof(cmd)-1);
            break;
        case 'r':
            rstflag = 1;
            break;
        case 'h':
        case '?':
            bootutil_help(0);
            break;
        }
    }

    /* Make sure a serial port was specified */
    if (tty[0] == '\0') {
        exit(bootutil_err_missing_arg("-d"));
    } else {
        /* Try to open the serial port */
        printf("Opening %s\n", tty);
        bootutil_serport_open(tty);
        /* Make sure the serial port was actually opened */
        if (g_bootutil_serport_open != 1) {
            exit(bootutil_err_invalid_arg(tty));
        }
    }

    /* Check if a system reset was requested (perform this first) */
    if (rstflag) {
        rc = bootutil_cmd_reset();
        if (rc != 0) {
            goto error;
        }
    }

    /* Command handler */
    if (cmdflag) {
        /* Make sure we have a valid command */
        if (cmd[0] == '\0') {
            rc = bootutil_err_missing_arg("-c");
            goto error;
        }
        /* Try to parse the supplied command */
        rc = bootutil_cmd_parse(cmd);
        if (rc != 0) {
            goto error;
        }
    }

    /* Give an error warning for any unhandled errors */
    if (rc != 0) {
        printf("RC = 0x%04X (%u): %s\n", rc, rc, strerror(rc));
    }

error:
    bootutil_serport_close();
    exit(rc);
}
