# Findg2o.cmake - Find g2o
# Modified from FindGlog.cmake by alexs.mac@gmail.com (Alex Stewart)
#
# Findg2o.cmake - Find g2o library.
#
# This module defines the following variables:
#
# G2O_FOUND: TRUE iff g2o is found.
# G2O_INCLUDE_DIRS: Include directories for g2o.
# G2O_LIBRARIES: Libraries required to link g2o.
#
# The following variables control the behaviour of this module:
#
# G2O_INCLUDE_DIR_HINTS: List of additional directories in which to
#                        search for g2o includes, e.g: /foo/include.
# G2O_LIBRARY_DIR_HINTS: List of additional directories in which to
#                        search for g2o libraries, e.g: /bar/lib.
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
# G2O_INCLUDE_DIR: Include directory for g2o, not including the
#                  include directory of any dependencies.
# G2O_LIBRARY: g2o library, not including the libraries of any
#              dependencies.

# Called if we failed to find g2o or any of it's required dependencies,
# unsets all public (designed to be used externally) variables and reports
# error message at priority depending upon [REQUIRED/QUIET/<NONE>] argument.
macro(G2O_REPORT_NOT_FOUND REASON_MSG)
    unset(G2O_FOUND)
    unset(G2O_INCLUDE_DIRS)
    unset(G2O_LIBRARIES)
    # Make results of search visible in the CMake GUI if g2o has not
    # been found so that user does not have to toggle to advanced view.
    mark_as_advanced(CLEAR G2O_INCLUDE_DIR G2O_LIBRARY)
    # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
    # use the camelcase library name, not uppercase.
    if(G2o_FIND_QUIETLY)
        message(STATUS "Failed to find g2o - " ${REASON_MSG} ${ARGN})
    elseif(G2o_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find g2o - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find g2o - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(G2O_REPORT_NOT_FOUND)

# Search user-installed locations first, so that we prefer user installs
# to system installs where both exist.
list(APPEND G2O_CHECK_INCLUDE_DIRS
    /usr/local/include
    /usr/local/homebrew/include # Mac OS X
    /opt/local/var/macports/software # Mac OS X.
    /opt/local/include
    /usr/include)
list(APPEND G2O_CHECK_LIBRARY_DIRS
    /usr/local/lib
    /usr/local/homebrew/lib # Mac OS X.
    /opt/local/lib
    /usr/lib)

# Check general hints
if(G2O_HINTS AND EXISTS ${G2O_HINTS})
    set(G2O_INCLUDE_DIR_HINTS ${G2O_HINTS}/include)
    set(G2O_LIBRARY_DIR_HINTS ${G2O_HINTS}/lib)
endif()

set(G2O_INCLUDE_FILE g2o/config.h)
# Search supplied hint directories first if supplied.
find_path(G2O_INCLUDE_DIR
    NAMES ${G2O_INCLUDE_FILE}
    PATHS ${G2O_INCLUDE_DIR_HINTS}
          ${G2O_CHECK_INCLUDE_DIRS}
    NO_DEFAULT_PATH)
if(NOT G2O_INCLUDE_DIR OR NOT EXISTS ${G2O_INCLUDE_DIR})
G2O_REPORT_NOT_FOUND("Could not find g2o include directory, "
    "set G2O_INCLUDE_DIR to directory containing g2o/config.h")
endif()

find_library(G2O_LIBRARY
    NAMES g2o_core
    PATHS ${G2O_LIBRARY_DIR_HINTS}
          ${G2O_CHECK_LIBRARY_DIRS}
    NO_DEFAULT_PATH)
if(NOT G2O_LIBRARY OR NOT EXISTS ${G2O_LIBRARY})
G2O_REPORT_NOT_FOUND("Could not find g2o library, "
    "set G2O_LIBRARY to full path to libg2o_core.")
else()
    string(REGEX MATCH ".*/" G2O_LIBRARY_DIR ${G2O_LIBRARY})
endif()

# Mark internally as found, then verify. G2O_REPORT_NOT_FOUND() unsets
# if called.
set(G2O_FOUND TRUE)

# Catch case when caller has set G2O_INCLUDE_DIR in the cache / GUI and
# thus FIND_[PATH/LIBRARY] are not called, but specified locations are
# invalid, otherwise we would report the library as found.
if(G2O_INCLUDE_DIR AND NOT EXISTS ${G2O_INCLUDE_DIR}/${G2O_INCLUDE_FILE})
G2O_REPORT_NOT_FOUND("Caller defined G2O_INCLUDE_DIR:"
    " ${G2O_INCLUDE_DIR} does not contain g2o/config.h header.")
endif()

# TODO: This regex for g2o library is pretty primitive, we use lowercase
#       for comparison to handle Windows using CamelCase library names, could
#       this check be better?
string(TOLOWER "${G2O_LIBRARY}" LOWERCASE_G2O_LIBRARY)
if(G2O_LIBRARY AND NOT "${LOWERCASE_G2O_LIBRARY}" MATCHES ".*g2o_*")
G2O_REPORT_NOT_FOUND("Caller defined G2O_LIBRARY: "
    "${G2O_LIBRARY} does not match g2o.")
endif()

# Set standard CMake FindPackage variables if found.
if(G2O_FOUND)
    set(G2O_INCLUDE_DIRS ${G2O_INCLUDE_DIR})
    file(GLOB G2O_LIBRARIES ${G2O_LIBRARY_DIR}libg2o*)
endif()

# Handle REQUIRED / QUIET optional arguments.
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(G2o DEFAULT_MSG
    G2O_INCLUDE_DIRS G2O_LIBRARIES)

# Only mark internal variables as advanced if we found g2o, otherwise
# leave them visible in the standard GUI for the user to set manually.
if(G2O_FOUND)
    mark_as_advanced(FORCE G2O_INCLUDE_DIR G2O_LIBRARY)
endif()
