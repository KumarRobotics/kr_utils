# FindGtsam.cmake - Find gtsam
# Modified from FindGlog.cmake by alexs.mac@gmail.com (Alex Stewart)
#
# FindGtsam.cmake - Find gtsam library.
#
# This module defines the following variables:
#
# Gtsam_FOUND: TRUE iff gtsam is found.
# Gtsam_INCLUDE_DIRS: Include directories for gtsam.
# Gtsam_LIBRARIES: Libraries required to link gtsam.
#
# The following variables control the behaviour of this module:
#
# Gtsam_INCLUDE_DIR_HINTS: List of additional directories in which to
#                          search for gtsam includes, e.g: /foo/include.
# Gtsam_LIBRARY_DIR_HINTS: List of additional directories in which to
#                          search for gtsam libraries, e.g: /bar/lib.
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
# Gtsam_INCLUDE_DIR: Include directory for gtsam, not including the
#                    include directory of any dependencies.
# Gtsam_LIBRARY: gtsam library, not including the libraries of any
#                dependencies.

# Called if we failed to find gtsam or any of it's required dependencies,
# unsets all public (designed to be used externally) variables and reports
# error message at priority depending upon [REQUIRED/QUIET/<NONE>] argument.
macro(Gtsam_REPORT_NOT_FOUND REASON_MSG)
    unset(Gtsam_FOUND)
    unset(Gtsam_INCLUDE_DIRS)
    unset(Gtsam_LIBRARIES)
    # Make results of search visible in the CMake GUI if gtsam has not
    # been found so that user does not have to toggle to advanced view.
    mark_as_advanced(CLEAR Gtsam_INCLUDE_DIR Gtsam_LIBRARY)
    # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
    # use the camelcase library name, not uppercase.
    if(Gtsam_FIND_QUIETLY)
        message(STATUS "Failed to find gtsam - " ${REASON_MSG} ${ARGN})
    elseif(Gtsam_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find gtsam - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find gtsam - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(Gtsam_REPORT_NOT_FOUND)

# Search user-installed locations first, so that we prefer user installs
# to system installs where both exist.
list(APPEND Gtsam_CHECK_INCLUDE_DIRS
    /usr/local/include
    /usr/local/homebrew/include # Mac OS X
    /opt/local/var/macports/software # Mac OS X.
    /opt/local/include
    /usr/include)
list(APPEND Gtsam_CHECK_LIBRARY_DIRS
    /usr/local/lib
    /usr/local/homebrew/lib # Mac OS X.
    /opt/local/lib
    /usr/lib)

# Check general hints
if(Gtsam_HINTS AND EXISTS ${Gtsam_HINTS})
    set(Gtsam_INCLUDE_DIR_HINTS ${Gtsam_HINTS}/include)
    set(Gtsam_LIBRARY_DIR_HINTS ${Gtsam_HINTS}/lib)
endif()

# Mark internally as found, then verify. Gtsam_REPORT_NOT_FOUND() unsets
# if called.
set(Gtsam_FOUND TRUE)

set(Gtsam_INCLUDE_FILE gtsam/config.h)
# Search supplied hint directories first if supplied.
find_path(Gtsam_INCLUDE_DIR
    NAMES ${Gtsam_INCLUDE_FILE}
    PATHS ${Gtsam_INCLUDE_DIR_HINTS}
          ${Gtsam_CHECK_INCLUDE_DIRS}
    NO_DEFAULT_PATH)
if(NOT Gtsam_INCLUDE_DIR OR NOT EXISTS ${Gtsam_INCLUDE_DIR})
Gtsam_REPORT_NOT_FOUND("Could not find gtsam include directory, "
    "set Gtsam_INCLUDE_DIR to directory containing gtsam/config.h")
endif()

find_library(Gtsam_LIBRARY
    NAMES gtsam
    PATHS ${Gtsam_LIBRARY_DIR_HINTS}
          ${Gtsam_CHECK_LIBRARY_DIRS}
    NO_DEFAULT_PATH)
if(NOT Gtsam_LIBRARY OR NOT EXISTS ${Gtsam_LIBRARY})
Gtsam_REPORT_NOT_FOUND("Could not find gtsam library, "
    "set Gtsam_LIBRARY to full path to libgtsam.")
endif()

# Catch case when caller has set Gtsam_INCLUDE_DIR in the cache / GUI and
# thus FIND_[PATH/LIBRARY] are not called, but specified locations are
# invalid, otherwise we would report the library as found.
if(Gtsam_INCLUDE_DIR AND NOT EXISTS ${Gtsam_INCLUDE_DIR}/${Gtsam_INCLUDE_FILE})
Gtsam_REPORT_NOT_FOUND("Caller defined Gtsam_INCLUDE_DIR:"
    " ${Gtsam_INCLUDE_DIR} does not contain gtsam/config.h header.")
endif()

# TODO: This regex for gtsam library is pretty primitive, we use lowercase
#       for comparison to handle Windows using CamelCase library names, could
#       this check be better?
string(TOLOWER "${Gtsam_LIBRARY}" LOWERCASE_Gtsam_LIBRARY)
if(Gtsam_LIBRARY AND NOT "${LOWERCASE_Gtsam_LIBRARY}" MATCHES ".*gtsam[^/]*")
Gtsam_REPORT_NOT_FOUND("Caller defined Gtsam_LIBRARY: "
    "${Gtsam_LIBRARY} does not match gtsam.")
endif()

# Set standard CMake FindPackage variables if found.
if(Gtsam_FOUND)
    set(Gtsam_INCLUDE_DIRS ${Gtsam_INCLUDE_DIR})
    list(APPEND Gtsam_LIBRARIES ${Gtsam_LIBRARY} tbb)
endif()

# Handle REQUIRED / QUIET optional arguments.
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Gtsam DEFAULT_MSG
    Gtsam_INCLUDE_DIRS Gtsam_LIBRARIES)

# Only mark internal variables as advanced if we found gtsam, otherwise
# leave them visible in the standard GUI for the user to set manually.
if(Gtsam_FOUND)
    mark_as_advanced(FORCE Gtsam_INCLUDE_DIR Gtsam_LIBRARY)
endif()
