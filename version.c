#include "version.h"

#include "version-string.h" //This will define the version string, and VERSION_STRING macro
#ifndef VERSION_STRING
#error "No version string is defined. This is strictly required to build. Check pre-build python step, version.py ."
#else
static unsigned char version[] = VERSION_STRING ;
#endif


unsigned char* versionGetString(void)
{ return version; }
