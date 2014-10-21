# FindOgre.cmake - Find Ogre
# Modified from FindGlog.cmake by alexs.mac@gmail.com (Alex Stewart)
#
# FindOgre.cmake - Find Ogre library.
#
# This module defines the following variables:
#
# OGRE_FOUND: TRUE iff Ogre is found.
# OGRE_INCLUDE_DIRS: Include directories for Ogre.
# OGRE_LIBRARIES: Libraries required to link Ogre.
#
# The following variables control the behaviour of this module:
#
# OGRE_INCLUDE_DIR_HINTS: List of additional directories in which to
#                         search for Ogre includes, e.g: /foo/include.
# OGRE_LIBRARY_DIR_HINTS: List of additional directories in which to
#                         search for Ogre libraries, e.g: /bar/lib.
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
# OGRE_INCLUDE_DIR: Include directory for Ogre, not including the
#                   include directory of any dependencies.
# OGRE_LIBRARY: Ogre library, not including the libraries of any
#               dependencies.

# Called if we failed to find Ogre or any of it's required dependencies,
# unsets all public (designed to be used externally) variables and reports
# error message at priority depending upon [REQUIRED/QUIET/<NONE>] argument.
macro(OGRE_REPORT_NOT_FOUND REASON_MSG)
    unset(OGRE_FOUND)
    unset(OGRE_INCLUDE_DIRS)
    unset(OGRE_LIBRARIES)
    # Make results of search visible in the CMake GUI if Ogre has not
    # been found so that user does not have to toggle to advanced view.
    mark_as_advanced(CLEAR OGRE_INCLUDE_DIR OGRE_LIBRARY)
    # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
    # use the camelcase library name, not uppercase.
    if(Ogre_FIND_QUIETLY)
        message(STATUS "Failed to find Ogre - " ${REASON_MSG} ${ARGN})
    elseif(Ogre_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find Ogre - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find Ogre - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(OGRE_REPORT_NOT_FOUND)

# Search user-installed locations first, so that we prefer user installs
# to system installs where both exist.
list(APPEND OGRE_CHECK_INCLUDE_DIRS
    /usr/local/include
    /usr/local/homebrew/include # Mac OS X
    /opt/local/var/macports/software # Mac OS X.
    /opt/local/include
    /usr/include/x86_64-linux-gnu
    /usr/include
    )
list(APPEND OGRE_CHECK_LIBRARY_DIRS
    /usr/local/lib
    /usr/local/homebrew/lib # Mac OS X.
    /opt/local/lib
    /usr/lib/x86_64-linux-gnu
    /usr/lib
    )

# Check general hints
if(OGRE_HINTS AND EXISTS ${OGRE_HINTS})
    set(OGRE_INCLUDE_DIR_HINTS ${OGRE_HINTS}/include)
    set(OGRE_LIBRARY_DIR_HINTS ${OGRE_HINTS}/lib)
endif()

set(OGRE_INCLUDE_FILE OGRE/Ogre.h)
# Search supplied hint directories first if supplied.
find_path(OGRE_INCLUDE_DIR
    NAMES ${OGRE_INCLUDE_FILE}
    PATHS ${OGRE_INCLUDE_DIR_HINTS}
          ${OGRE_CHECK_INCLUDE_DIRS}
    NO_DEFAULT_PATH)
if(NOT OGRE_INCLUDE_DIR OR NOT EXISTS ${OGRE_INCLUDE_DIR})
OGRE_REPORT_NOT_FOUND("Could not find Ogre include directory, "
    "set OGRE_INCLUDE_DIR to directory containing OGRE/Ogre.h")
endif()

find_library(OGRE_LIBRARY
    NAMES OgreMain
    PATHS ${OGRE_LIBRARY_DIR_HINTS}
          ${OGRE_CHECK_LIBRARY_DIRS}
    NO_DEFAULT_PATH)
if(NOT OGRE_LIBRARY OR NOT EXISTS ${OGRE_LIBRARY})
OGRE_REPORT_NOT_FOUND("Could not find Ogre library, "
    "set OGRE_LIBRARY to full path to libOgreMain.so.")
else()
    string(REGEX MATCH ".*/" OGRE_LIBRARY_DIR ${OGRE_LIBRARY})
endif()

# Mark internally as found, then verify. OGRE_REPORT_NOT_FOUND() unsets
# if called.
set(OGRE_FOUND TRUE)

# Catch case when caller has set OGRE_INCLUDE_DIR in the cache / GUI and
# thus FIND_[PATH/LIBRARY] are not called, but specified locations are
# invalid, otherwise we would report the library as found.
if(OGRE_INCLUDE_DIR AND NOT EXISTS ${OGRE_INCLUDE_DIR}/${OGRE_INCLUDE_FILE})
OGRE_REPORT_NOT_FOUND("Caller defined OGRE_INCLUDE_DIR:"
    " ${OGRE_INCLUDE_DIR} does not contain Ogre/config.h header.")
endif()

# TODO: This regex for Ogre library is pretty primitive, we use lowercase
#       for comparison to handle Windows using CamelCase library names, could
#       this check be better?
string(TOLOWER "${OGRE_LIBRARY}" LOWERCASE_OGRE_LIBRARY)
if(OGRE_LIBRARY AND NOT "${LOWERCASE_OGRE_LIBRARY}" MATCHES ".*ogre*")
OGRE_REPORT_NOT_FOUND("Caller defined OGRE_LIBRARY: "
    "${OGRE_LIBRARY} does not match Ogre.")
endif()

# Set standard CMake FindPackage variables if found.
if(OGRE_FOUND)
    list(APPEND OGRE_INCLUDE_DIRS ${OGRE_INCLUDE_DIR} /usr/include/suitesparse)
    file(GLOB OGRE_LIBRARIES ${OGRE_LIBRARY_DIR}libOgre*)
endif()

# Handle REQUIRED / QUIET optional arguments.
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Ogre DEFAULT_MSG
    OGRE_INCLUDE_DIRS OGRE_LIBRARIES)

# Only mark internal variables as advanced if we found Ogre, otherwise
# leave them visible in the standard GUI for the user to set manually.
if(OGRE_FOUND)
    mark_as_advanced(FORCE OGRE_INCLUDE_DIR OGRE_LIBRARY)
endif()
