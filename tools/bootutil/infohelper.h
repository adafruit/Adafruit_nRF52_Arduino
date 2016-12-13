/**********************************************************************************
 **********************************************************************************
 ***
 ***    infohelper.h
 ***    - defines and prototypes for a crude verbositiy-controllabe info output
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

#ifndef INFOHELPER_H
#define INFOHELPER_H


/*
** set verbositiy level
** 0 = only error messages
** 1 = warnings
** 2 = information messages (default)
** 3 = debugging info
** 4 = extra debugging info
*/
void infohelper_set_infolevel(char lvl);

/*
** increase verbositylevel by 1
*/
void infohelper_increase_infolevel(void);

/*
** set verbosity level according to arguments given
*/
void infohelper_set_argverbosity(int num_args, char **arg_ptr);

void infohelper_output(int loglevel, const char* format, ...);
void infohelper_output_plain(int loglevel, const char* format, ...);

#define LOGERR(...) infohelper_output(0, __VA_ARGS__)
#define LOGWARN(...) infohelper_output(1, __VA_ARGS__)
#define LOGINFO(...) infohelper_output(2, __VA_ARGS__)
#define LOGDEBUG(...) infohelper_output(3, __VA_ARGS__)
#define LOGVERBOSE(...) infohelper_output(4, __VA_ARGS__)

#define INFO(...) infohelper_output_plain(0, __VA_ARGS__)

#endif
