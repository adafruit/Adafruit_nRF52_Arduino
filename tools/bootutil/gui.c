#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <curses.h>
#include "infohelper.h"

static void
gui_config(void)
{
    /* Disable line buffering (to avoid waiting for enter key) */
	raw();
    /* Enable color functionality */
	start_color();
    /* Enable F1, F2, etc. */
    keypad(stdscr, TRUE);
    /* Dont echo in getch (avoids garbage output on special keys) */
	noecho();
    /* Setup color pairs */
	init_pair(1, COLOR_CYAN, COLOR_BLACK);
}

/*
void gui_menu_print(WINDOW *win, int highlight)
{
	int x, y, i;

	x = 2;
	y = 2;
	box(win, 0, 0);
	for(i = 0; i < 10; ++i) {
        if(highlight == i + 1) {
            wattron(win, A_REVERSE);
			mvwprintw(win, y, x, "%s", "string");
			wattroff(win, A_REVERSE);
		} else {
			mvwprintw(win, y, x, "%s", "string");
        }
		++y;
	}
	wrefresh(win);
}
*/

int
gui_init(void)
{
    int ch;
    int row, col;
    WINDOW * mainwin;

    /* Init ncurses window */
    if ( (mainwin = initscr()) == NULL ) {
        LOGERR("error initialising ncurses.\n");
        exit(EXIT_FAILURE);
    }

    /* Setup the core window properties */
    gui_config();

    /* Get the window dimenstions */
    getmaxyx(stdscr,row,col);

    /* Draw some borders (hack, remove) */
	box(mainwin, 0, 0);

    /* Add the title */
    char *title = "Adafruit nRF52 Bootloader Utility";
    attron(COLOR_PAIR(1));
	attron(A_BOLD);
    mvprintw(row/2,(col-strlen(title))/2,"%s",title);
	attroff(A_BOLD);
    attroff(COLOR_PAIR(1));

    /* Wait for some feedback */
    mvprintw(1, 2, "Type any character to see it in bold and color");
	ch = getch();

    /* Display the key value */
    mvprintw(2, 2, "The pressed key is ");
    attron(COLOR_PAIR(1));
	attron(A_BOLD);

    switch(ch) {
        case KEY_F(1):
	        printw("F1");
            break;
        case KEY_LEFT:
	        printw("LEFT");
			break;
		case KEY_RIGHT:
	        printw("RIGHT");
			break;
		case KEY_UP:
	        printw("UP");
			break;
		case KEY_DOWN:
	        printw("DOWN");
			break;
        default:
	        printw("%c", ch);
            break;
    }

	attroff(A_BOLD);
    attroff(COLOR_PAIR(1));
    refresh();

    /* Wait a bit before closing */
    sleep(2);

    /*  Clean up after ourselves  */
    delwin(mainwin);
    endwin();
    refresh();

    return EXIT_SUCCESS;
}
