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

int
gui_init(void)
{
    int ch;
    WINDOW * mainwin;

    /* Init ncurses window */
    if ( (mainwin = initscr()) == NULL ) {
        LOGERR("error initialising ncurses.\n");
        exit(EXIT_FAILURE);
    }

    /* Setup the core window properties */
    gui_config();

    /* Wait for some feedback */
    printw("Type any character to see it in bold and color\n");
	ch = getch();

    attron(COLOR_PAIR(1));
    printw("The pressed key is ");
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
    sleep(2);

    /*  Clean up after ourselves  */
    delwin(mainwin);
    endwin();
    refresh();

    return EXIT_SUCCESS;
}
