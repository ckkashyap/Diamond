/*
 * piano.h — Interactive piano demo public API.
 *
 * Renders an 88-key piano on the framebuffer, accepts mouse input, and
 * plays the correct frequency for each key click.
 *
 * Press ESC or Q to return to the shell.
 */
#pragma once

void piano_run(void);
