cmake_minimum_required(VERSION 2.8)
project("cmavnode" C CXX)

find_package( Boost 1.40 COMPONENTS program_options thread system REQUIRED )
find_package( Threads )

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

file(GLOB cmavnode_SRC
    "src/*.cpp"
    )

#actual executable
add_executable(cmavnode ${cmavnode_SRC})

target_compile_options(cmavnode PRIVATE -ggdb -DELPP_NO_DEFAULT_LOG_FILE -DELP_STACKTRACE_ON_CRASH -DELPP_THREAD_SAFE )
TARGET_LINK_LIBRARIES(cmavnode ${Boost_LIBRARIES} -lpthread )
