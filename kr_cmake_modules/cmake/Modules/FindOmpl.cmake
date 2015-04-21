# FindOmpl.cmake - Find Ompl
# quchao@seas.upenn.edu (Chao Qu)
# Modified from FindGlog.cmake by alexs.mac@gmail.com (Alex Stewart)
#
# FindOmpl.cmake - Find Ompl library.
#
# This module defines the following variables:
#
# Ompl_FOUND:        TRUE iff Ompl is found.
# Ompl_INCLUDE_DIRS: Include directories for Ompl.
# Ompl_LIBRARIES:    Libraries required to link Ompl.
#
# The following variables control the behaviour of this module:
#
# Ompl_INCLUDE_DIR_HINTS: List of additional directories in which to
#                         search for Ompl includes, e.g: /foo/include.
# Ompl_LIBRARY_DIR_HINTS: List of additional directories in which to
#                         search for Ompl libraries, e.g: /bar/lib.
#
# The following variables are also defined by this module, but in line with
# CMake recommended FindPackage() module style should NOT be referenced directly
# by callers (use the plural variables detailed above instead).  These variables
# do however affect the behaviour of the module via FIND_[PATH/LIBRARY]() which
# are NOT re-called (i.e. search for library is not repeated) if these variables
# are set with valid values _in the CMake cache_. This means that if these
# variables are set directly in the cache, either by the user in the CMake GUI,
# or by the user passing -DVAR=VALUE directives to CMake when called (which
# explicitly defines a cache variable), then they will be used verbatim,
# bypassing the HINTS variables and other hard-coded search locations.
#
# Ompl_INCLUDE_DIR: Include directory for Ompl, not including the
#                   include directory of any dependencies.
# Ompl_LIBRARY: Ompl library, not including the libraries of any
#               dependencies.

# Called if we failed to find Ompl or any of it's required dependencies,
# unsets all public (designed to be used externally) variables and reports
# error message at priority depending upon [REQUIRED/QUIET/<NONE>] argument.
macro(Ompl_REPORT_NOT_FOUND REASON_MSG)
    unset(Ompl_FOUND)
    unset(Ompl_INCLUDE_DIRS)
    unset(Ompl_LIBRARIES)
    # Make results of search visible in the CMake GUI if Ompl has not
    # been found so that user does not have to toggle to advanced view.
    mark_as_advanced(CLEAR Ompl_INCLUDE_DIR Ompl_LIBRARY)
    # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
    # use the camelcase library name, not uppercase.
    if(Ompl_FIND_QUIETLY)
        message(STATUS "Failed to find Ompl - " ${REASON_MSG} ${ARGN})
    elseif(Ompl_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find Ompl - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find Ompl - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(Ompl_REPORT_NOT_FOUND)

# Search user-installed locations first, so that we prefer user installs
# to system installs where both exist.
list(APPEND Ompl_CHECK_INCLUDE_DIRS
    /opt/ros/indigo/include
    /usr/local/include
    /usr/local/homebrew/include # Mac OS X
    /opt/local/var/macports/software # Mac OS X.
    /opt/local/include
    /usr/include/x86_64-linux-gnu
    /usr/include
    )
list(APPEND Ompl_CHECK_LIBRARY_DIRS
    /opt/ros/indigo/lib/x86_64-linux-gnu
    /usr/local/lib
    /usr/local/homebrew/lib # Mac OS X.
    /opt/local/lib
    /usr/lib/x86_64-linux-gnu
    /usr/lib
    )

# Check general hints
if(Ompl_HINTS AND EXISTS ${Ompl_HINTS})
    set(Ompl_INCLUDE_DIR_HINTS ${Ompl_HINTS}/include)
    set(Ompl_LIBRARY_DIR_HINTS ${Ompl_HINTS}/lib)
endif()

set(Ompl_INCLUDE_FILE ompl/config.h)
# Search supplied hint directories first if supplied.
find_path(Ompl_INCLUDE_DIR
    NAMES ${Ompl_INCLUDE_FILE}
    PATHS ${Ompl_INCLUDE_DIR_HINTS}
          ${Ompl_CHECK_INCLUDE_DIRS}
    NO_DEFAULT_PATH)
if(NOT Ompl_INCLUDE_DIR OR NOT EXISTS ${Ompl_INCLUDE_DIR})
Ompl_REPORT_NOT_FOUND("Could not find Ompl include directory, "
    "set Ompl_INCLUDE_DIR to directory containing Ompl/config.h")
endif()

find_library(Ompl_LIBRARY
    NAMES ompl
    PATHS ${Ompl_LIBRARY_DIR_HINTS}
          ${Ompl_CHECK_LIBRARY_DIRS}
    NO_DEFAULT_PATH)
if(NOT Ompl_LIBRARY OR NOT EXISTS ${Ompl_LIBRARY})
Ompl_REPORT_NOT_FOUND("Could not find Ompl library, "
    "set Ompl_LIBRARY to full path to libompl.so.")
else()
    string(REGEX MATCH ".*/" Ompl_LIBRARY_DIR ${Ompl_LIBRARY})
endif()

# Mark internally as found, then verify. Ompl_REPORT_NOT_FOUND() unsets
# if called.
set(Ompl_FOUND TRUE)

# Catch case when caller has set Ompl_INCLUDE_DIR in the cache / GUI and
# thus FIND_[PATH/LIBRARY] are not called, but specified locations are
# invalid, otherwise we would report the library as found.
if(Ompl_INCLUDE_DIR AND NOT EXISTS ${Ompl_INCLUDE_DIR}/${Ompl_INCLUDE_FILE})
Ompl_REPORT_NOT_FOUND("Caller defined Ompl_INCLUDE_DIR:"
    " ${Ompl_INCLUDE_DIR} does not contain ompl/config.h header.")
endif()

# TODO: This regex for Ompl library is pretty primitive, we use lowercase
#       for comparison to handle Windows using CamelCase library names, could
#       this check be better?
string(TOLOWER "${Ompl_LIBRARY}" LOWERCASE_Ompl_LIBRARY)
if(Ompl_LIBRARY AND NOT "${LOWERCASE_Ompl_LIBRARY}" MATCHES ".*ompl*")
Ompl_REPORT_NOT_FOUND("Caller defined Ompl_LIBRARY: "
    "${Ompl_LIBRARY} does not match ompl.")
endif()

# Set standard CMake FindPackage variables if found.
if(Ompl_FOUND)
    set(Ompl_INCLUDE_DIRS ${Ompl_INCLUDE_DIR})
    set(Ompl_LIBRARIES ${Ompl_LIBRARY})
endif()

# Handle REQUIRED / QUIET optional arguments.
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Ompl DEFAULT_MSG
    Ompl_INCLUDE_DIRS Ompl_LIBRARIES)

# Only mark internal variables as advanced if we found Ompl, otherwise
# leave them visible in the standard GUI for the user to set manually.
if(Ompl_FOUND)
    mark_as_advanced(FORCE Ompl_INCLUDE_DIR Ompl_LIBRARY)
endif()
