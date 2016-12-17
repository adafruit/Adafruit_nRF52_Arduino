#ifndef _WIN32

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <curses.h>
#include "infohelper.h"
#include "commands.h"

enum gui_action {
    GUI_ACTION_ABOUT = 0,
    GUI_ACTION_RESET,
    GUI_ACTION_EXIT,
    GUI_ACTION_NONE = 0xFF,
};

struct gui_frame_cfg {
    int x;
    int y;
    int w;
    int h;
    int titlebar;
    int colorpair;
};

/* Advanced prototypes for command handler functions */
int gui_handler_about(void);
int gui_handler_reset(void);

static const struct {
    const char *name;
    const char shortcut;
    const enum gui_action action;
    int (*handler)(void);
} gui_menu_items[] = {
    { "About",          '?',  GUI_ACTION_ABOUT, gui_handler_about },
    { "Reset Device",   'r',  GUI_ACTION_RESET, gui_handler_reset },
    { "Exit",           'X',  GUI_ACTION_EXIT,  NULL },
    { NULL,             '\0', GUI_ACTION_NONE, NULL }
};

static volatile int g_gui_menu_item_count = 0;

int
gui_handler_about(void) {
    printw("About Adafruit nRF52 bootutil!\n");
    return 0;
}

int
gui_handler_reset(void) {
    printw("Resetting the nRF52 (toggling DTR)\n");
    commands_cmd_reset();
    return 0;
}

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
	init_pair(2, COLOR_BLACK, COLOR_BLACK);
	init_pair(3, COLOR_WHITE, COLOR_BLUE);
    /* Hide the cursor by default */
    curs_set(0);
}

static void
gui_draw_frame(WINDOW *win, const struct gui_frame_cfg *cfg)
{
    int i, hpad;

    if ((cfg->x < 0) || (cfg->y < 0)) {
        return;
    }

    if ((!cfg->w) || (!cfg->h)) {
        return;
    }

    /* Check if we need hpadding for the titlebar */
    hpad = cfg->titlebar ? 2 : 0;

    /* Set the frame colors */
    if (cfg->colorpair > 0) {
        attron(COLOR_PAIR(cfg->colorpair));
    }

    /* Top bar */
    wmove(win, cfg->y, cfg->x);
    addch(ACS_ULCORNER);
    for (i=0;i<cfg->w;i++) {
        addch(ACS_HLINE);
    }
    addch(ACS_URCORNER);

    /* Title section if requested */
    if (cfg->titlebar) {
        wmove(win, cfg->y+1, cfg->x);
        addch(ACS_VLINE);
        wmove(win, cfg->y+1, cfg->x+cfg->w+1);
        addch(ACS_VLINE);
        wmove(win, cfg->y+2, cfg->x);
        addch(ACS_LTEE);
        for (i=0;i<cfg->w;i++) {
            addch(ACS_HLINE);
        }
        addch(ACS_RTEE);
    }

    /* Middle lines */
    for (i=0;i<cfg->h;i++) {
        wmove(win, cfg->y+i+hpad+1, cfg->x);
        addch(ACS_VLINE);
        wmove(win, cfg->y+i+hpad+1, cfg->x+cfg->w+1);
        addch(ACS_VLINE);
    }

    /* Bottom bar */
    wmove(win,cfg-> y+cfg->h+hpad+1, cfg->x);
    addch(ACS_LLCORNER);
    for (i=0;i<cfg->w;i++) {
        addch(ACS_HLINE);
    }
    addch(ACS_LRCORNER);

    /* Clear the frame colors */
    if (cfg->colorpair > 0) {
        attroff(COLOR_PAIR(cfg->colorpair));
    }

	wrefresh(win);
}

static void
gui_menu_print(WINDOW *win, int highlight)
{
    int rows, cols;
	int x, y, i;

    /* Get the window dimenstions */
    getmaxyx(stdscr, rows, cols);

    /* Draw some borders */
    struct gui_frame_cfg cfg = {
        .x = 0,
        .y = 0,
        .w = cols-2,
        .h = rows-2,
        .titlebar = false,
        .colorpair = 0
    };
    gui_draw_frame(win, &cfg);

	//box(win, 0, 0);

    /* Add the title */
    char *title = "Adafruit nRF52 Bootloader Utility";
    attron(COLOR_PAIR(1));
	attron(A_BOLD);
    mvprintw(2, (cols-strlen(title))/2, "%s", title);
	attroff(A_BOLD);
    attroff(COLOR_PAIR(1));

    /* Set the menu item start position */
	y = 4;
    x = (cols-strlen(title))/2;
    wmove(win, y, x);

    if (highlight > g_gui_menu_item_count) {
        highlight = 0;
    }

	for(i = 0; i < g_gui_menu_item_count; ++i) {
        /* Print the menu item name */
        if(highlight == i + 1) {
            wattron(win, A_REVERSE);
			mvwprintw(win, y, x, "%s", gui_menu_items[i].name);
			wattroff(win, A_REVERSE);
		} else {
			mvwprintw(win, y, x, "%s", gui_menu_items[i].name);
        }
        /* Print the shortcut */
        if (gui_menu_items[i].shortcut != '\0') {
            printw(" [");
            attron(A_BOLD);
            printw("%c", gui_menu_items[i].shortcut);
            attroff(A_BOLD);
            printw("]");
        }
		++y;
	}

    /* Change the cursor to displat any debug output at a safe location*/
    wmove(win, 5 + g_gui_menu_item_count, x);
	wrefresh(win);
}

int
gui_init(const char *ttyname)
{
    int i;
    int ch;
    int selected_menu_item;
    int execute_menu_item;
    int rows, cols;
    WINDOW * mainwin;

    /* Init ncurses window */
    if ( (mainwin = initscr()) == NULL ) {
        LOGERR("error initialising ncurses.\n");
        exit(EXIT_FAILURE);
    }

    /* Setup the core window properties */
    gui_config();

    /* Count the total menu items */
    g_gui_menu_item_count = -1;
    while (gui_menu_items[++g_gui_menu_item_count].action != GUI_ACTION_NONE) {
    }

    if (g_gui_menu_item_count < 1) {
        LOGERR("No menu items defined\n");
        return EXIT_FAILURE;
    }

    /* Get the window dimenstions */
    getmaxyx(stdscr, rows, cols);

    /* Show the TTY/Serial port */
    attron(COLOR_PAIR(2));
	attron(A_BOLD);
    mvprintw(rows-4, (cols-strlen(ttyname))/2, "%s", ttyname);
	attroff(A_BOLD);
    attroff(COLOR_PAIR(2));

    /* Help message on navigation */
    char *dataentry = "Use the arrow or shortcut keys to continue, 'X' to exit";
    attron(COLOR_PAIR(1));
	attron(A_BOLD);
    mvprintw(rows-3, (cols-strlen(dataentry))/2, "%s", dataentry);
	attroff(A_BOLD);
    attroff(COLOR_PAIR(1));

    /* Render the menu */
    selected_menu_item = 1;
    gui_menu_print(mainwin, selected_menu_item);

    /* Check input until we get an exit request */
    while(1) {
        ch = getch();
        execute_menu_item = -1;
        switch(ch) {
            case KEY_F(1):
                break;
            case KEY_LEFT:
    	        selected_menu_item--;
    			break;
    		case KEY_RIGHT:
    	        selected_menu_item++;
    			break;
    		case KEY_UP:
    	        selected_menu_item--;
    			break;
    		case KEY_DOWN:
    	        selected_menu_item++;
    			break;
            case 10:    /* ENTER */
                execute_menu_item = selected_menu_item-1;
                break;
            case 'Z':
                goto cleanup;
                break;
            default:
                /* Check for hotkeys */
                for (i=0;i<g_gui_menu_item_count;i++) {
                    if (ch == gui_menu_items[i].shortcut) {
                        execute_menu_item = i;
                    }
                }
                break;
        }

        /* Check if we should execute an action */
        if (execute_menu_item != -1) {
            if (gui_menu_items[execute_menu_item].action != GUI_ACTION_NONE) {
                if (gui_menu_items[execute_menu_item].handler != NULL) {
                    attron(COLOR_PAIR(2));
                    attron(A_BOLD);
                    gui_menu_items[execute_menu_item].handler();
                    attroff(A_BOLD);
                    attroff(COLOR_PAIR(2));
                    wrefresh(mainwin);
                }
            }
            if (gui_menu_items[execute_menu_item].action == GUI_ACTION_EXIT) {
                goto cleanup;
            }
        }

        /* Check menu boundaries */
        if (selected_menu_item < 1) {
            selected_menu_item = 1;
        }
        if (selected_menu_item > g_gui_menu_item_count) {
            selected_menu_item = g_gui_menu_item_count;
        }

        /* Re-render the menu with the new menu item */
        gui_menu_print(mainwin, selected_menu_item);

        /* Update the display */
        refresh();
    }

cleanup:
    /*  Clean up after ourselves  */
    delwin(mainwin);
    endwin();
    refresh();

    return EXIT_SUCCESS;
}

#endif /* _WIN32 */
