#include <linux/input.h>

 /* ,-----------------------------------------------------------------------------------.
  * |   `  |   1  |   2  |   3  |   4  |   5  |   6  |   7  |   8  |   9  |   0  |      |
  * | Tab  |   Q  |   W  |   F  |   P  |   G  |   J  |   L  |   U  |   Y  |   ;  | Bksp |
  * |   ~  |   !  |   @  |   #  |   $  |   %  |   ^  |   &  |   *  |   (  |   )  |      |
  * |------+------+------+------+------+-------------+------+------+------+------+------|
  * | Del  |  F1  |  F2  |  F3  |  F4  |  F5  |  F6  |   -  |   =  |   [  |   ]  |  \   |
  * | Esc  |   A  |   R  |   S  |   T  |   D  |   H  |   N  |   E  |   I  |   O  |  "   |
  * | Del  |  F1  |  F2  |  F3  |  F4  |  F5  |  F6  |   _  |   +  |   {  |   }  |  |   |
  * |------+------+------+------+------+------|------+------+------+------+------+------|
  * |      |  F7  |  F8  |  F9  |  F10 |  F11 |  F12 |ISO # |ISO / | Vol- | Vol+ |      |
  * | Shift|   Z  |   X  |   C  |   V  |   B  |   K  |   M  |   ,  |   .  |   /  |Enter |
  * |      |  F7  |  F8  |  F9  |  F10 |  F11 |  F12 |ISO ~ |ISO | | Vol- | Vol+ |      |
  * |------+------+------+------+------+------+------+------+------+------+------+------|
  * |      |      |      |      |      | Scr- | Scr+ |      | Home |Pg Up |Pg Dn | End  |
  * | Ctrl | GUI  | Alt  |WrkSp |Lower |Space | Bksp |Raise | Left | Down |  Up  |Right |
  * |      |      |      |      |      | Vol- | Vol+ |      | Home |Pg Up |Pg Dn | End  |
  * `-----------------------------------------------------------------------------------'
  */

static unsigned short planck_keycodes[] = {
  // BASE
  KEY_TAB, KEY_Q, KEY_W, KEY_F, KEY_P, KEY_G, KEY_G, KEY_J, KEY_L, KEY_U, KEY_Y, KEY_SEMICOLON, KEY_DELETE,
  KEY_ESC, KEY_A, KEY_R, KEY_S, KEY_T, KEY_D, KEY_H, KEY_N, KEY_E, KEY_I, KEY_O, KEY_APOSTROPHE, 
  KEY_LEFTSHIFT, KEY_Z, KEY_X, KEY_C, KEY_V, KEY_B, KEY_K, KEY_M, KEY_COMMA, KEY_DOT, KEY_SLASH, KEY_ENTER, 
  KEY_LEFTCTRL, KEY_LEFTMETA, KEY_LEFTALT, 0, 0, KEY_SPACE, KEY_BACKSPACE, 0, KEY_LEFT, KEY_DOWN, KEY_UP, KEY_RIGHT, 

  // LOWER
  KEY_GRAVE, KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9, KEY_0, KEY_DELETE, 
  KEY_DELETE, KEY_F1, KEY_F2, KEY_F3, KEY_F4, KEY_F5, KEY_F6, KEY_MINUS, KEY_EQUAL, KEY_LEFTBRACE, KEY_RIGHTBRACE, KEY_BACKSLASH, 
  KEY_RIGHTSHIFT, KEY_F7, KEY_F8, KEY_F9, KEY_F10, KEY_F11, KEY_F12, KEY_BRIGHTNESSDOWN, KEY_BRIGHTNESSUP, KEY_VOLUMEUP, KEY_VOLUMEDOWN, KEY_ENTER, 
  KEY_RIGHTCTRL, KEY_RIGHTMETA, KEY_LEFTALT, 0, 0, KEY_SPACE, KEY_BACKSPACE, 0, KEY_HOME, KEY_PAGEUP, KEY_PAGEDOWN, KEY_END, 

  // HIGHER
  KEY_GRAVE, KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9, KEY_0, KEY_DELETE, 
  KEY_DELETE, KEY_F1, KEY_F2, KEY_F3, KEY_F4, KEY_F5, KEY_F6, KEY_MINUS, KEY_EQUAL, KEY_LEFTBRACE, KEY_RIGHTBRACE, KEY_BACKSLASH, 
  KEY_RIGHTSHIFT, KEY_F7, KEY_F8, KEY_F9, KEY_F10, KEY_F11, KEY_F12, KEY_BRIGHTNESSDOWN, KEY_BRIGHTNESSUP, KEY_VOLUMEUP, KEY_VOLUMEDOWN, KEY_ENTER, 
  KEY_RIGHTCTRL, KEY_RIGHTMETA, KEY_LEFTALT, 0, 0, KEY_SPACE, KEY_BACKSPACE, 0, KEY_HOME, KEY_PAGEUP, KEY_PAGEDOWN, KEY_END 
};
