# FindGtest.cmake - Find gogole test
# quchao@seas.upenn.edu (Chao Qu)
# Modified from FindGlog.cmake by alexs.mac@gmail.com (Alex Stewart)
#
# FindGtest.cmake - Find Google test library.
#
# This module defines the following variables:
#
# Gtest_FOUND: TRUE iff gtest is found.
# Gtest_INCLUDE_DIRS: Include directories for gtest.
# Gtest_LIBRARIES: Libraries required to link gtest.
#
# The following variables control the behaviour of this module:
#
# Gtest_INCLUDE_DIR_HINTS: List of additional directories in which to
#                          search for gtest includes, e.g: /foo/include.
# Gtest_LIBRARY_DIR_HINTS: List of additional directories in which to
#                          search for gtest libraries, e.g: /bar/lib.
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
# Gtest_INCLUDE_DIR: Include directory for gtest, not including the
#                    include directory of any dependencies.
# Gtest_LIBRARY: gtest library, not including the libraries of any
#                dependencies.

# Called if we failed to find gtest or any of it's required dependencies,
# unsets all public (designed to be used externally) variables and reports
# error message at priority depending upon [REQUIRED/QUIET/<NONE>] argument.
macro(Gtest_REPORT_NOT_FOUND REASON_MSG)
    unset(Gtest_FOUND)
    unset(Gtest_INCLUDE_DIRS)
    unset(Gtest_LIBRARIES)
    # Make results of search visible in the CMake GUI if gtest has not
    # been found so that user does not have to toggle to advanced view.
    mark_as_advanced(CLEAR Gtest_INCLUDE_DIR Gtest_LIBRARY)
    # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
    # use the camelcase library name, not uppercase.
    if(Gtest_FIND_QUIETLY)
        message(STATUS "Failed to find gtest - " ${REASON_MSG} ${ARGN})
    elseif(Gtest_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find gtest - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find gtest - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(Gtest_REPORT_NOT_FOUND)

# Search user-installed locations first, so that we prefer user installs
# to system installs where both exist.
list(APPEND Gtest_CHECK_INCLUDE_DIRS
    /usr/local/include
    /usr/local/homebrew/include # Mac OS X
    /opt/local/var/macports/software # Mac OS X.
    /opt/local/include
    /usr/include)
list(APPEND Gtest_CHECK_LIBRARY_DIRS
    /usr/local/lib
    /usr/local/homebrew/lib # Mac OS X.
    /opt/local/lib
    /usr/lib)

# Check general hints
if(Gtest_HINTS AND EXISTS ${Gtest_HINTS})
    set(Gtest_INCLUDE_DIR_HINTS ${Gtest_HINTS}/include)
    set(Gtest_LIBRARY_DIR_HINTS ${Gtest_HINTS}/lib/.libs)
endif()

# Search supplied hint directories first if supplied.
find_path(Gtest_INCLUDE_DIR
    NAMES gtest/gtest.h
    PATHS ${Gtest_INCLUDE_DIR_HINTS}
    ${Gtest_CHECK_INCLUDE_DIRS}
    NO_DEFAULT_PATH)
if(NOT Gtest_INCLUDE_DIR OR NOT EXISTS ${Gtest_INCLUDE_DIR})
    Gtest_REPORT_NOT_FOUND("Could not find gtest include directory, "
        "set Gtest_INCLUDE_DIR to directory containing gtest/gtest.h")
endif()

find_library(Gtest_LIBRARY
    NAMES gtest
    PATHS ${Gtest_LIBRARY_DIR_HINTS}
    ${Gtest_CHECK_LIBRARY_DIRS}
    NO_DEFAULT_PATH)
if(NOT Gtest_LIBRARY OR NOT EXISTS ${Gtest_LIBRARY})
    Gtest_REPORT_NOT_FOUND("Could not find gtest library, "
        "set Gtest_LIBRARY to full path to libgtest.")
else()
    string(REGEX MATCH ".*/" Gtest_LIBRARY_DIR ${Gtest_LIBRARY})
endif()

# Mark internally as found, then verify. Gtest_REPORT_NOT_FOUND() unsets
# if called.
set(Gtest_FOUND TRUE)

# Catch case when caller has set Gtest_INCLUDE_DIR in the cache / GUI and
# thus FIND_[PATH/LIBRARY] are not called, but specified locations are
# invalid, otherwise we would report the library as found.
if(Gtest_INCLUDE_DIR AND NOT EXISTS ${Gtest_INCLUDE_DIR}/gtest/gtest.h)
    Gtest_REPORT_NOT_FOUND("Caller defined Gtest_INCLUDE_DIR:"
        " ${Gtest_INCLUDE_DIR} does not contain gtest/gtest.h header.")
endif()

# TODO: This regex for gtest library is pretty primitive, we use lowercase
#       for comparison to handle Windows using CamelCase library names, could
#       this check be better?
string(TOLOWER "${Gtest_LIBRARY}" LOWERCASE_Gtest_LIBRARY)
if(Gtest_LIBRARY AND NOT "${LOWERCASE_Gtest_LIBRARY}" MATCHES ".*gtest[^/]*")
    Gtest_REPORT_NOT_FOUND("Caller defined Gtest_LIBRARY: "
        "${Gtest_LIBRARY} does not match gtest.")
endif()

# Set standard CMake FindPackage variables if found.
if(Gtest_FOUND)
    set(Gtest_INCLUDE_DIRS ${Gtest_INCLUDE_DIR})
    file(GLOB Gtest_LIBRARIES ${Gtest_LIBRARY_DIR}lib*.so)
endif()

# Handle REQUIRED / QUIET optional arguments.
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Gtest DEFAULT_MSG
    Gtest_INCLUDE_DIRS Gtest_LIBRARIES)

# Only mark internal variables as advanced if we found gtest, otherwise
# leave them visible in the standard GUI for the user to set manually.
if(Gtest_FOUND)
    mark_as_advanced(FORCE Gtest_INCLUDE_DIR Gtest_LIBRARY)
endif()
