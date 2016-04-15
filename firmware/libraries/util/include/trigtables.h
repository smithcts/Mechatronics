#ifndef TRIGTABLES_H_INCLUDED
#define TRIGTABLES_H_INCLUDED

#define TABLEN 1000
#define ATANRANGE 22.36068f   // sqrt(TABLELEN/2) makes trunction and delta near zero equal

// Global variables
extern const float COSTABLE[TABLEN];
extern const float ATANTABLE[TABLEN];

// Exported functions -------------------------------------------------------
#ifdef __cplusplus
    extern "C" {
#endif
    extern float atan2tab(float y, float x);
    extern void sincostab(float theta, float *sth, float *cth);
#ifdef __cplusplus
    }
#endif

#endif
