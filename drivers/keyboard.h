/*
 * keyboard.h - PS/2 keyboard driver public API
 */
#pragma once

/*
 * Poll the PS/2 keyboard.
 * Returns an ASCII byte if a key was pressed, or -1 if the buffer is empty.
 * Modifier keys (Shift, Caps Lock, Alt, Ctrl) are handled internally.
 */
int kb_getchar(void);

/*
 * Returns 1 if the space bar is currently physically held down, 0 otherwise.
 * Actively drains the PS/2 and VirtIO keyboard queues to update state.
 * Safe to call from inside an audio-playback loop between DMA chunks.
 */
int kb_space_held(void);
