/* Stub math.h for freestanding kernel builds.
   stb_truetype includes <math.h>; the functions it actually calls are
   overridden via STBTT_* macros in font.c, so we only need declarations. */
#pragma once

double sqrt(double x);
double fabs(double x);
double floor(double x);
double ceil(double x);
double pow(double base, double exp);
double cos(double x);
double acos(double x);
