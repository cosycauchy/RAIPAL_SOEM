
cmake_minimum_required(VERSION 3.16)
project(raipal_soem_app LANGUAGES C CXX)

# Standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add ~/.local to package search path
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/.local")

# Find the installed SOEM package (lowercase name per your SOEM CMake)
find_package(soem CONFIG REQUIRED)  # looks for soemConfig.cmake

# Common link libs for Linux raw-socket backend
set(COMMON_LINK_LIBS soem pthread rt)
# If you built SOEM with PCAP backend, uncomment:
# list(APPEND COMMON_LINK_LIBS pcap)

# Include path for your public headers (if you have any)
# e.g., raipal_soem/include/{*.hpp}
set(RAIPAL_PUBLIC_INCLUDE "${CMAKE_CURRENT_LIST_DIR}/include")

# ============== C++ core library (your new modular code) =====================
# Files below are examples; adjust to your actual files
add_library(raipal_core STATIC
        src/ethercat_master.cpp
        src/ethercat_slave.cpp
        src/ethercat_actuator.cpp
)
target_include_directories(raipal_core
        PUBLIC
        ${RAIPAL_PUBLIC_INCLUDE}
)

#target_link_libraries(raipal_core PRIVATE ${COMMON_LINK_LIBS})
target_link_libraries(raipal_core
        PUBLIC  soem        # make SOEM public so <ethercat.h> is visible to dependents
        PRIVATE pthread rt  # keep system libs private
)



# ============== C++ demo that uses the new library ===========================
# Optional: only if you have a C++ main
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/src/main.cpp")
    add_executable(demo src/main.cpp)
    target_link_libraries(demo PRIVATE raipal_core)
    target_link_libraries(demo PRIVATE yaml-cpp)
endif()


# ============== Legacy C test executables (unchanged) ========================
# All your C test files reside in raipal_soem/src
# Build them as pure C executables and link to SOEM/system libs.
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/src/simple_test.c")
    add_executable(simple_test src/simple_test.c)
    target_link_libraries(simple_test PRIVATE ${COMMON_LINK_LIBS})
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/src/slaveinfo.c")
    add_executable(slaveinfo src/slaveinfo.c)
    target_link_libraries(slaveinfo PRIVATE ${COMMON_LINK_LIBS})
endif()

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/src/simple_test_dongdrive.c")
    add_executable(simple_test_dongdrive src/simple_test_dongdrive.c)
    target_link_libraries(simple_test_dongdrive PRIVATE ${COMMON_LINK_LIBS})
endif()


# ============== 백래시 측정 ==============
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/src/backlash_test.cpp")
    add_executable(backlash_test src/backlash_test.cpp)
    target_link_libraries(backlash_test PRIVATE raipal_core yaml-cpp)
endif()