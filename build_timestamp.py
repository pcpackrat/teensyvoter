Import("env")
import datetime

# __DATE__/__TIME__ only refresh when the specific .cpp using them gets
# recompiled - PlatformIO's incremental build skips files that didn't
# change, so a build-date banner printed from main.cpp went stale for an
# entire session's worth of fixes because those fixes were all in other
# files. This injects a real "when was this actually built" timestamp as
# a compiler define, fresh on every single build invocation regardless of
# which files changed, so the banner is trustworthy.
timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
env.Append(CPPDEFINES=[("BUILD_TIMESTAMP", '\\"%s\\"' % timestamp)])
