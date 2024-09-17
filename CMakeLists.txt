cmake_minimum_required(VERSION 3.25)
project(sahlp VERSION 1
            DESCRIPTION "Solver for the Single Assignment Hub Location Problem"
            LANGUAGES CXX)

find_package(SCIP CONFIG REQUIRED)
if(SCIP_FOUND)
  message(NOTICE "Found SCIP in ${SCIP_DIR}")
### print out which exact .so file we link against:
#   get_target_property(SCIP_LIBRARY libscip IMPORTED_SONAME_RELWITHDEBINFO)
#   get_target_property(SCIP_LIBRARY libscip IMPORTED_SONAME_RELEASE)
#   message(NOTICE "SCIP: ${SCIP_LIBRARY}")
else()
  message(FATAL_ERROR "SCIP not found")
endif()

# using the Min Cost Flow library from https://github.com/frangio68/Min-Cost-Flow-Class
find_package(MCFClass)
if(MCFClass_FOUND)
  message(NOTICE "Found MinCostFlow in ${MCFClass_DIR}")
else()
  message(STATUS "MinCostFlow not found")
endif()


# add boost library
find_package(Boost 1.74.0)
if(Boost_FOUND)
  message(NOTICE "Found Boost in ${Boost_DIR}")
  include_directories(${Boost_INCLUDE_DIRS})
endif()

# add cplex
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(CPLEX)
if(CPLEX_FOUND)
    include_directories(${CPLEX_INCLUDE_DIR})
    link_directories(${CPLEX_LIBRARY})
   message(STATUS "CPLEX found!")
else()
   message(STATUS "CPLEX not found!")
endif ()

if(CPLEX_ILOCPLEX_FOUND)
    include_directories(${CPLEX_ILOCPLEX_INCLUDE_DIR})
    link_directories(${CPLEX_ILOCPLEX_LIBRARY})
   message(STATUS "ILOCPLEX found!")
else()
   message(STATUS "ILOCPLEX not found!")
endif()

# add executable and library
add_subdirectory(src)

# add doxygen documentation
option(BUILD_DOC "Build doxygen documentation" ON)
include(FindDoxygen)
find_package(Doxygen)
if(DOXYGEN_FOUND)
  message("Found Doxygen")
  # doxygen settings: show private members and variables of classes, but only if they are documented
  set(DOXYGEN_EXTRACT_PRIVATE YES)
  set(DOXYGEN_HIDE_UNDOC_MEMBERS YES)
  doxygen_add_docs(doc src/)
else(DOXYGEN_FOUND)
  message("Doxygen not found")
endif(DOXYGEN_FOUND)



# ===== Google Testing Framework ===== #
enable_testing()
## Download and unpack googletest at configure time
configure_file(tests/CMakeLists.txt.googletest googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

## prevent overriding the parent project's compiler/linker
## settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

## add googletest directly to our build 
## this defines the gtest and gtest_main targets
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

## the gtest/gtest_main targets carry header search path
## dependencies automatically when using CMake 2.8.11 or
## later. Otherwise we have to add them here ourselves
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()

## link against gtest as needed
set(TEST_SRC 
    tests/test_solver.cpp
)
add_executable(${PROJECT_NAME}-test ${TEST_SRC})
target_include_directories(${PROJECT_NAME}-test PUBLIC "src")
set_property(TARGET ${PROJECT_NAME}-test PROPERTY CXX_STANDARD 17)
target_link_libraries(${PROJECT_NAME}-test gtest_main)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}lib)

# copy the directory with the testing files to the test execution location
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_data/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/test_data)


# # make library and header file installable
# https://iamsorush.com/posts/cpp-cmake-config/#build-and-install
# https://stackoverflow.com/questions/10487256/cmake-how-to-properly-copy-static-librarys-header-file-into-usr-include
install(TARGETS sahlplib 
EXPORT sahlplibTargets
FILE_SET HEADERS
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
RUNTIME DESTINATION bin
INCLUDES DESTINATION include
)

install(EXPORT sahlplibTargets
FILE sahlplibTargets.cmake
NAMESPACE sahlplib::
DESTINATION lib/cmake/sahlplib)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
"cmake/sahlplibConfigVersion.cmake"
VERSION 1
COMPATIBILITY AnyNewerVersion)

install(FILES "cmake/sahlplibConfig.cmake" "${CMAKE_CURRENT_BINARY_DIR}/cmake/sahlplibConfigVersion.cmake" DESTINATION lib/cmake/sahlplib)