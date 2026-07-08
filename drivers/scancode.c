/*
 * scancode.c — Shared PS/2 scancode-set-1 → ASCII tables and decode.
 * See scancode.h for who uses this and why.
 */

#include "scancode.h"

/* Unshifted scancode → ASCII (0 = no printable mapping) */
const char scancode_set1_lower[128] = {
/*00*/ 0,  27, '1','2','3','4','5','6','7','8','9','0','-','=',  8, '\t',
/*10*/'q','w','e','r','t','y','u','i','o','p','[',']','\n',  0, 'a', 's',
/*20*/'d','f','g','h','j','k','l',';','\'','`',  0,'\\','z','x', 'c', 'v',
/*30*/'b','n','m',',','.','/',  0, '*',  0, ' ',  0,   0,   0,   0,   0,  0,
/*40*/ 0,  0,  0,  0,  0,  0,  0, '7','8','9','-','4','5','6','+','1',
/*50*/'2','3','0','.', 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*60*/ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*70*/ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
};

/* Shifted scancode → ASCII */
const char scancode_set1_upper[128] = {
/*00*/ 0,  27, '!','@','#','$','%','^','&','*','(',')','_','+',  8, '\t',
/*10*/'Q','W','E','R','T','Y','U','I','O','P','{','}','\n',  0, 'A', 'S',
/*20*/'D','F','G','H','J','K','L',':','"', '~',  0, '|','Z','X', 'C', 'V',
/*30*/'B','N','M','<','>','?',  0, '*',  0, ' ',  0,   0,   0,   0,   0,  0,
/*40*/ 0,  0,  0,  0,  0,  0,  0, '7','8','9','-','4','5','6','+','1',
/*50*/'2','3','0','.', 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*60*/ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
/*70*/ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
};

char scancode1_to_ascii(uint8_t code, int shift, int caps) {
    if (code >= 128) return 0;
    char base = scancode_set1_lower[code];
    int use_shift = shift;
    /* Caps Lock affects letter keys only, where it inverts Shift. */
    if (base >= 'a' && base <= 'z') use_shift = shift ^ caps;
    return use_shift ? scancode_set1_upper[code] : scancode_set1_lower[code];
}
