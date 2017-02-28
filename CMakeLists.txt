cmake_minimum_required(VERSION 2.8)
project("cmavnode" C CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, options are: Debug, Release."
        FORCE)
endif(NOT CMAKE_BUILD_TYPE)

set(Boost_USE_STATIC_LIBS ON)

find_package( Boost 1.40 COMPONENTS program_options thread system REQUIRED )
find_package( Threads )

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

file(GLOB cmavnode_SRC
    "src/*.cpp"
    )

#actual executable
add_executable(cmavnode ${cmavnode_SRC})
set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG " -ggdb")

target_compile_options(cmavnode PRIVATE -std=c++11 -ggdb)
TARGET_LINK_LIBRARIES(cmavnode ${Boost_LIBRARIES} -lpthread -lreadline)

install(TARGETS cmavnode DESTINATION bin)
