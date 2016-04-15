
// Automatically define globs (declared in globs.h) without double maintenance.
#define DEFINE_GLOBS
#include "globs.h"
#undef DEFINE_GLOBS

// Array of metadata. This gets populated in the constructor of the globs.
GlobBase * globs[NUM_GLOBS];
