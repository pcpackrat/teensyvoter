#ifndef VERSION_H
#define VERSION_H

#define FIRMWARE_VERSION "1.1.7"

// BUILD_TIMESTAMP is injected fresh by build_timestamp.py on every single
// build invocation, regardless of which .cpp files actually changed -
// __DATE__/__TIME__ only refresh when the specific file using them gets
// recompiled, which made the old banner go stale for a whole session's
// worth of fixes that happened to land in other files.
#ifdef BUILD_TIMESTAMP
#define BUILD_DATE BUILD_TIMESTAMP
#else
#define BUILD_DATE __DATE__ " " __TIME__
#endif

#endif
