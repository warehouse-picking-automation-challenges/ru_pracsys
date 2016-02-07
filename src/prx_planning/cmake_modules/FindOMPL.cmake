find_library(OMPL_LIBRARY ompl DOC "Location of Open Motion Planning Library")
find_path(OMPL_INCLUDE_DIR ompl/base/State.h PATH_SUFFIXES "OMPL"
    DOC "Location of Open Motion Planning Library header files")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ompl DEFAULT_MSG OMPL_LIBRARY OMPL_INCLUDE_DIR)
