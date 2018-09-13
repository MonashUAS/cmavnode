cmake_minimum_required(VERSION 2.8)
project("cmavnode" C CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, options are: Debug, Release."
        FORCE)
endif(NOT CMAKE_BUILD_TYPE)


# Include find files
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(Boost_USE_STATIC_LIBS ON)

find_package( Boost 1.40 COMPONENTS program_options thread system REQUIRED )
find_package( Threads REQUIRED)
find_package( Readline REQUIRED)

add_subdirectory(include/pistache)

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${READLINE_INCLUDE_DIR})
INCLUDE_DIRECTORIES( ${PISTACHE_INCLUDE_DIR})

# glob source files
file(GLOB cmavnode_SRC
    "src/*.cpp"
    )

#actual executable
add_executable(cmavnode ${cmavnode_SRC})
set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG " -ggdb")
set(CMAKE_CXX_FLAGS "-std=c++1z")


target_link_libraries(cmavnode pistache)
TARGET_LINK_LIBRARIES(cmavnode ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${READLINE_LIBRARY})

install(TARGETS cmavnode DESTINATION bin)
