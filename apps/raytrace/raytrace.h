/*
 * raytrace.h - SMP ray-tracing demo public API
 */
#pragma once

/*
 * Render the scene onto the active video framebuffer using all CPUs.
 * If slow != 0, also renders a single-core pass first and prints the speedup.
 */
void raytrace_run(int slow);
