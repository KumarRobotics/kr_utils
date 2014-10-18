# FindQGLViewer.cmake - Findqglviewer
# Modified from FindGlog.cmake by alexs.mac@gmail.com (Alex Stewart)
#
# FindQGLViewer.cmake - Find QGLViewer library.
#
# This module defines the following variables:
#
# QGLVIEWER_FOUND: TRUE iff QGLViewer is found.
# QGLVIEWER_INCLUDE_DIRS: Include directories for QGLViewer.
# QGLVIEWER_LIBRARIES: Libraries required to link QGLViewer.
#
# The following variables control the behaviour of this module:
#
# QGLVIEWER_INCLUDE_DIR_HINTS: List of additional directories in which to
#                              search for QGLViewer includes, e.g: /foo/include.
# QGLVIEWER_LIBRARY_DIR_HINTS: List of additional directories in which to
#                              search for QGLViewer libraries, e.g: /bar/lib.
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
# QGLVIEWER_INCLUDE_DIR: Include directory for QGLViewer, not including the
#                        include directory of any dependencies.
# QGLVIEWER_LIBRARY: QGLViewer library, not including the libraries of any
#                    dependencies.

# Called if we failed to find QGLViewer or any of it's required dependencies,
# unsets all public (designed to be used externally) variables and reports
# error message at priority depending upon [REQUIRED/QUIET/<NONE>] argument.
macro(QGLVIEWER_REPORT_NOT_FOUND REASON_MSG)
    unset(QGLVIEWER_FOUND)
    unset(QGLVIEWER_INCLUDE_DIRS)
    unset(QGLVIEWER_LIBRARIES)
    # Make results of search visible in the CMake GUI if QGLViewer has not
    # been found so that user does not have to toggle to advanced view.
    mark_as_advanced(CLEAR QGLVIEWER_INCLUDE_DIR QGLVIEWER_LIBRARY)
    # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
    # use the camelcase library name, not uppercase.
    if(QGLViewer_FIND_QUIETLY)
        message(STATUS "Failed to find QGLViewer - " ${REASON_MSG} ${ARGN})
    elseif(Qglviewer_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find QGLViewer - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find QGLViewer - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(QGLVIEWER_REPORT_NOT_FOUND)

# Search user-installed locations first, so that we prefer user installs
# to system installs where both exist.
list(APPEND QGLVIEWER_CHECK_INCLUDE_DIRS
    /usr/local/include
    /usr/local/homebrew/include # Mac OS X
    /opt/local/var/macports/software # Mac OS X.
    /opt/local/include
    /usr/include
    )
list(APPEND QGLVIEWER_CHECK_LIBRARY_DIRS
    /usr/local/lib
    /usr/local/homebrew/lib # Mac OS X.
    /opt/local/lib
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    )

# Check general hints
if(QGLVIEWER_HINTS AND EXISTS ${QGLVIEWER_HINTS})
    set(QGLVIEWER_INCLUDE_DIR_HINTS ${QGLVIEWER_HINTS}/include)
    set(QGLVIEWER_LIBRARY_DIR_HINTS ${QGLVIEWER_HINTS}/lib)
endif()

find_package(Qt4 COMPONENTS QtCore QtXml QtOpenGL QtGui)
include(${QT_USE_FILE})

set(QGLVIEWER_INCLUDE_FILE QGLViewer/config.h)
# Search supplied hint directories first if supplied.
find_path(QGLVIEWER_INCLUDE_DIR
    NAMES ${QGLVIEWER_INCLUDE_FILE}
    PATHS ${QGLVIEWER_INCLUDE_DIR_HINTS}
          ${QGLVIEWER_CHECK_INCLUDE_DIRS}
    NO_DEFAULT_PATH)
if(NOT QGLVIEWER_INCLUDE_DIR OR NOT EXISTS ${QGLVIEWER_INCLUDE_DIR})
QGLVIEWER_REPORT_NOT_FOUND("Could not find QGLViewer include directory, "
    "set QGLVIEWER_INCLUDE_DIR to directory containing QGLViewer/config.h")
endif()

find_library(QGLVIEWER_LIBRARY
    NAMES libQGLViewer.so
    PATHS ${QGLVIEWER_LIBRARY_DIR_HINTS}
          ${QGLVIEWER_CHECK_LIBRARY_DIRS}
    NO_DEFAULT_PATH)
if(NOT QGLVIEWER_LIBRARY OR NOT EXISTS ${QGLVIEWER_LIBRARY})
QGLVIEWER_REPORT_NOT_FOUND("Could not find QGLViewer library, "
    "set QGLVIEWER_LIBRARY to full path to libQGLViewer.so.")
endif()

# Mark internally as found, then verify. QGLVIEWER_REPORT_NOT_FOUND() unsets
# if called.
set(QGLVIEWER_FOUND TRUE)

# Catch case when caller has set QGLVIEWER_INCLUDE_DIR in the cache / GUI and
# thus FIND_[PATH/LIBRARY] are not called, but specified locations are
# invalid, otherwise we would report the library as found.
if(QGLVIEWER_INCLUDE_DIR AND NOT EXISTS ${QGLVIEWER_INCLUDE_DIR}/${QGLVIEWER_INCLUDE_FILE})
QGLVIEWER_REPORT_NOT_FOUND("Caller defined QGLVIEWER_INCLUDE_DIR:"
    " ${QGLVIEWER_INCLUDE_DIR} does not contain QGLViewer/config.h header.")
endif()

# TODO: This regex for QGLViewer library is pretty primitive, we use lowercase
#       for comparison to handle Windows using CamelCase library names, could
#       this check be better?
string(TOLOWER "${QGLVIEWER_LIBRARY}" LOWERCASE_QGLVIEWER_LIBRARY)
if(QGLVIEWER_LIBRARY AND NOT "${LOWERCASE_QGLVIEWER_LIBRARY}" MATCHES ".*qglviewer[^/]*")
QGLVIEWER_REPORT_NOT_FOUND("Caller defined QGLVIEWER_LIBRARY: "
    "${QGLVIEWER_LIBRARY} does not match QGLViewer.")
endif()

# Set standard CMake FindPackage variables if found.
if(QGLVIEWER_FOUND)
    list(APPEND QGLVIEWER_INCLUDE_DIRS ${QGLVIEWER_INCLUDE_DIR} ${QT_INCLUDES})
    list(APPEND QGLVIEWER_LIBRARIES ${QGLVIEWER_LIBRARY} ${QT_LIBRARIES} GL glut GLU)
endif()

# Handle REQUIRED / QUIET optional arguments.
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Qglviewer DEFAULT_MSG
    QGLVIEWER_INCLUDE_DIRS QGLVIEWER_LIBRARIES)

# Only mark internal variables as advanced if we found QGLViewer, otherwise
# leave them visible in the standard GUI for the user to set manually.
if(QGLVIEWER_FOUND)
    mark_as_advanced(FORCE QGLVIEWER_INCLUDE_DIR QGLVIEWER_LIBRARY)
endif()
