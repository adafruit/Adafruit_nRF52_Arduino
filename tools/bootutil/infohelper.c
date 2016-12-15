/**********************************************************************************
 **********************************************************************************
 ***
 ***    infohelper.c
 ***    - implementation of a crude verbosity-controllabe info system
 ***
 ***    Copyright (C) 2014 Christian Klippel <ck@atelier-klippel.de>
 ***
 ***    This program is free software; you can redistribute it and/or modify
 ***    it under the terms of the GNU General Public License as published by
 ***    the Free Software Foundation; either version 2 of the License, or
 ***    (at your option) any later version.
 ***
 ***    This program is distributed in the hope that it will be useful,
 ***    but WITHOUT ANY WARRANTY; without even the implied warranty of
 ***    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 ***    GNU General Public License for more details.
 ***
 ***    You should have received a copy of the GNU General Public License along
 ***    with this program; if not, write to the Free Software Foundation, Inc.,
 ***    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 ***
 **/

#include <stdio.h>
#include <stdarg.h>
#include "infohelper.h"

static char infolevel = 0;

void infohelper_set_infolevel(char lvl)
{
    infolevel = lvl;
}

void infohelper_increase_infolevel(void)
{
    if(infolevel < 20)
    {
        infolevel++;
    }
}

void infohelper_set_argverbosity(int num_args, char **arg_ptr)
{
    char *cur_arg;

    while(num_args--)
    {
        cur_arg = arg_ptr[num_args];

        if(*cur_arg++ == '-')
        {
            if(*cur_arg == 'q')
            {
                infolevel = 0;
                return;
            }
            else while(*cur_arg++ == 'v')
            {
                infolevel++;
            }
        }
    }
}

void infohelper_output(int loglevel, const char* format, ...)
{
    if (infolevel < loglevel)
        return;
    const char* log_level_names[] = {"Error: ", "Warning: ", "", "\t", "\t\t"};
    const int log_level_names_count = sizeof(log_level_names) / sizeof(const char*);
    if (loglevel >= log_level_names_count)
        loglevel = log_level_names_count - 1;

    va_list v;
    printf("%s", log_level_names[loglevel]);
    va_start(v, format);
    vprintf(format, v);
    va_end(v);
    printf("\n");
}

void infohelper_output_plain(int loglevel, const char* format, ...)
{
    if (infolevel < loglevel)
        return;
    va_list v;
    va_start(v, format);
    vprintf(format, v);
    va_end(v);
}
