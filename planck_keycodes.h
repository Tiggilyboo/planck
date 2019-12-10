#include <linux/input.h>

#define MAX_X 12
#define MAX_Y 4

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

// Internal input device keycode mappings
static unsigned short planck_keycodes[] = {
  // BASE
  KEY_TAB, KEY_Q, KEY_W, KEY_F, KEY_P, KEY_G, KEY_J, KEY_L, KEY_U, KEY_Y, KEY_SEMICOLON, KEY_DELETE,
  KEY_ESC, KEY_A, KEY_R, KEY_S, KEY_T, KEY_D, KEY_H, KEY_N, KEY_E, KEY_I, KEY_O, KEY_APOSTROPHE, 
  KEY_LEFTSHIFT, KEY_Z, KEY_X, KEY_C, KEY_V, KEY_B, KEY_K, KEY_M, KEY_COMMA, KEY_DOT, KEY_SLASH, KEY_ENTER, 
  KEY_LEFTCTRL, KEY_LEFTMETA, KEY_LEFTALT, 0, 0, KEY_SPACE, KEY_BACKSPACE, 0, KEY_LEFT, KEY_DOWN, KEY_UP, KEY_RIGHT, 

  // LOWER
  KEY_GRAVE, KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9, KEY_0, KEY_DELETE, 
  KEY_ESC, KEY_F1, KEY_F2, KEY_F3, KEY_F4, KEY_F5, KEY_F6, KEY_MINUS, KEY_EQUAL, KEY_LEFTBRACE, KEY_RIGHTBRACE, KEY_BACKSLASH, 
  KEY_RIGHTSHIFT, KEY_F7, KEY_F8, KEY_F9, KEY_F10, KEY_F11, KEY_F12, KEY_BRIGHTNESSDOWN, KEY_BRIGHTNESSUP, KEY_VOLUMEUP, KEY_VOLUMEDOWN, KEY_ENTER, 
  KEY_RIGHTCTRL, KEY_RIGHTMETA, KEY_LEFTALT, 0, 0, KEY_SPACE, KEY_BACKSPACE, 0, KEY_HOME, KEY_PAGEDOWN, KEY_PAGEUP, KEY_END, 

  // UPPER
  KEY_GRAVE, KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9, KEY_0, KEY_DELETE, 
  KEY_ESC, KEY_F1, KEY_F2, KEY_F3, KEY_F4, KEY_F5, KEY_F6, KEY_MINUS, KEY_EQUAL, KEY_LEFTBRACE, KEY_RIGHTBRACE, KEY_BACKSLASH, 
  KEY_RIGHTSHIFT, KEY_F7, KEY_F8, KEY_F9, KEY_F10, KEY_F11, KEY_F12, KEY_BRIGHTNESSDOWN, KEY_BRIGHTNESSUP, KEY_VOLUMEUP, KEY_VOLUMEDOWN, KEY_ENTER, 
  KEY_RIGHTCTRL, KEY_RIGHTMETA, KEY_LEFTALT, 0, 0, KEY_SPACE, KEY_BACKSPACE, 0, KEY_HOME, KEY_PAGEDOWN, KEY_PAGEUP, KEY_END, 

  // BOTH
  KEY_CONNECT, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};


// External USB HID gadget 
// Map key position matrix to HID key reports
// https://usb.org/document-library/hid-usage-tables-112
static unsigned char planck_hid_keymap[] = {
  // BASE
  0x2b, 0x14, 0x1A, 0x09, 0x13, 0x0A, 0x0D, 0x0F, 0x18, 0x1C, 0x33, 0x4C,
  0x29, 0x04, 0x15, 0x16, 0x17, 0x07, 0x0B, 0x11, 0x08, 0x0C, 0x12, 0x34,
  0xE1, 0x1D, 0x1B, 0x06, 0x19, 0x05, 0x0E, 0x10, 0x36, 0x37, 0x38, 0x28,
  0xE0, 0xE3, 0xE2, 0x00, 0x00, 0x2C, 0x2A, 0x00, 0x50, 0x51, 0x52, 0x4F,

  // LOWER
  0x35, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x4C,
  0x29, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x2D, 0x2E, 0x2F, 0x30, 0x31,
  0xE5, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x00, 0x80, 0x81, 0x28,
  0xE4, 0xE7, 0xE2, 0x00, 0x00, 0x2C, 0x2A, 0x00, 0x4A, 0x4E, 0x4B, 0x4D,

  // UPPER
  0x35, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x4C,
  0x29, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x2D, 0x2E, 0x2F, 0x30, 0x31,
  0xE5, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x00, 0x80, 0x81, 0x28,
  0xE4, 0xE7, 0xE2, 0x00, 0x00, 0x2C, 0x2A, 0x00, 0x4A, 0x4E, 0x4B, 0x4D,

  // BOTH
  0xDA, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0
};

static unsigned char planck_hid_report[8];
