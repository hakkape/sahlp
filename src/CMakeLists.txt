# library that containes all solver code
set(SOLVERLIB_SRC
    probdata_sahlp.cpp 
    cons_benders_sahlp.cpp
    cons_benderslp_sahlp.cpp
    solver_sahlp.cpp
    reader_sahlp.cpp
    matheuristic_sahlp.cpp
    prop_partialenum.cpp
)

add_library(${PROJECT_NAME}lib SHARED)
## set sources for exporting when library is installed
target_sources(${PROJECT_NAME}lib
    PRIVATE ${SOLVERLIB_SRC}
    PUBLIC FILE_SET HEADERS 
    BASE_DIRS ${PROJECT_SOURCE_DIR}/src
    FILES solver_sahlp.hpp)
## link dependencies
target_link_libraries(${PROJECT_NAME}lib PUBLIC libscip)
target_link_libraries(${PROJECT_NAME}lib PRIVATE ${Boost_LIBRARIES})
if (CPLEX_FOUND)
    target_compile_definitions(${PROJECT_NAME}lib PRIVATE WITHCPLEX)
    target_link_libraries(${PROJECT_NAME}lib PRIVATE cplex-library ilocplex)
endif()
if (MCFClass_FOUND)
    target_compile_definitions(${PROJECT_NAME}lib PRIVATE WITHMCF)
    target_link_libraries(${PROJECT_NAME}lib PRIVATE MCFClass::MCFClass)
endif()
include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
FetchContent_MakeAvailable(json)
target_link_libraries(${PROJECT_NAME}lib PRIVATE nlohmann_json::nlohmann_json)

## compilation specifications
set_property(TARGET ${PROJECT_NAME}lib PROPERTY CXX_STANDARD 17)
target_compile_options(${PROJECT_NAME}lib PRIVATE -Wall)

# executable that calls solver on specific input file
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}lib)
set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
