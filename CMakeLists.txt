cmake_minimum_required(VERSION 3.21)
project(maestro_chai3d)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_STANDARD 14)

if(WIN32)
    set(OS "win32")
    include_directories(include/soem_include/oshw/win32/wpcap/Include)
    link_directories(${CMAKE_SOURCE_DIR}include/soem_include/oshw/win32/wpcap/Lib)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /D _CRT_SECURE_NO_WARNINGS")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  /WX")
    set(OS_LIBS wpcap.lib Packet.lib Ws2_32.lib Winmm.lib)
elseif(UNIX)
    set(OS "linux")
    #set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wextra -Wno-error=stringop-overflow")
    set(OS_LIBS pthread rt)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rt-kernel")
    set(OS "rtk")
    message("ARCH is ${ARCH}")
    message("BSP is ${BSP}")
    include_directories(include/soem_include/oshw/${OS}/${ARCH})
    file(GLOB OSHW_EXTRA_SOURCES include/soem_include/oshw/${OS}/${ARCH}/*.c)
    set(OSHW_SOURCES "${OS_HW_SOURCES} ${OSHW_ARCHSOURCES}" src/cMaestroHand.cpp src/cMaestroHand.h)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-error ")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-but-set-variable")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-function")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-format")
    set(OS_LIBS "-Wl,--start-group -l${BSP} -l${ARCH} -lkern -ldev -lsio -lblock -lfs -lusb -llwip -leth -li2c -lrtc -lcan -lnand -lspi -lnor -lpwm -ladc -ltrace -lc -lm -Wl,--end-group")
endif()

message("OS is ${OS}")

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    # Default to installing in SOEM source directory
    set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)
endif()

file(GLOB SOEM_SOURCES include/soem_include/soem/*.c)
file(GLOB OSAL_SOURCES include/soem_include/osal/${OS}/*.c)
file(GLOB OSHW_SOURCES include/soem_include/oshw/${OS}/*.c)
file(GLOB ESMACAT_SOURCES include/esmacat_server/*.cpp)
file(GLOB MAESTRO3_SOURCES include/maestro3/*.cpp)

file(GLOB SOEM_HEADERS include/soem_include/soem/*.h)
file(GLOB OSAL_HEADERS include/soem_include/osal/osal.h osal/${OS}/*.h)
file(GLOB OSHW_HEADERS include/soem_include/oshw/${OS}/*.h)
file(GLOB ESMACAT_HEADERS esmacat_server/*.h)
file(GLOB MAESTRO3_HEADERS include/maestro3/*.h)

include_directories(include/soem_include/soem)
include_directories(include/soem_include/osal)
include_directories(include/soem_include/osal/${OS})
include_directories(include/soem_include/oshw/${OS})
include_directories(include/esmacat_server)
include_directories(include/maestro3)

# output location
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY                ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG          ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL     ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE        ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${PROJECT_SOURCE_DIR}/bin/${OS}-${ARCH})

#Generate the shared library from the sources
add_library(soem STATIC
        ${SOEM_SOURCES}
        ${OSAL_SOURCES}
        ${OSHW_SOURCES}
        ${OSHW_EXTRA_SOURCES}
)
target_link_libraries(soem ${OS_LIBS})

add_library(esmacat STATIC
        ${ESMACAT_SOURCES}
        )
#target_link_libraries(esmacat soem)

add_library(maestro3 STATIC
        ${MAESTRO3_SOURCES}
        )
target_link_libraries(maestro3 esmacat soem)

#CHAI-3D
# include Chai3d
# -- this is required by Chai3d to pick the correct path for OpenGL headers
FIND_PACKAGE(CHAI3D REQUIRED)
ADD_DEFINITIONS(${CHAI3D_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin)
    ADD_DEFINITIONS(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
    ADD_DEFINITIONS(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
LINK_DIRECTORIES(${CHAI3D_LIBRARY_DIRS})

# Add the include directory to the include paths
INCLUDE_DIRECTORIES (
        ${CHAI3D_INCLUDE_DIRS}
)

#LINK_DIRECTORIES(/home/herolab/chai3d-3.2.0/build/modules/BULLET)
#FIND_PACKAGE(CHAI3D-BULLET REQUIRED)
#ADD_DEFINITIONS(${CHAI3D-BULLET_DEFINITIONS})
#LINK_DIRECTORIES(${CHAI3D-BULLET_LIBRARY_DIRS})
#INCLUDE_DIRECTORIES (
#        ${CHAI3D-BULLET_INCLUDE_DIRS}}
#)

FIND_PACKAGE(CHAI3D-GEL REQUIRED)
ADD_DEFINITIONS(${CHAI3D-GEL_DEFINITIONS})
LINK_DIRECTORIES(${CHAI3D-GEL_LIBRARY_DIRS})
INCLUDE_DIRECTORIES (
        ${CHAI3D-GEL_INCLUDE_DIRS}}
)

#LINK_DIRECTORIES(/home/herolab/chai3d-3.2.0/build/modules/ODE)
#FIND_PACKAGE(CHAI3D-ODE REQUIRED)
#ADD_DEFINITIONS(${CHAI3D-ODE_DEFINITIONS})
#LINK_DIRECTORIES(${CHAI3D-ODE_LIBRARY_DIRS})
#INCLUDE_DIRECTORIES (
#        ${CHAI3D-ODE_INCLUDE_DIRS}}
#)
#




#include eigen
set(EIGEN3_INCLUDE_DIR "/home/herolab/eigen-3.4.0/build")
FIND_PACKAGE(Eigen3 REQUIRED)

# Find python and Boost - both are required dependencies
#find_package(PythonLibs 2.7 REQUIRED)
#find_package(Boost COMPONENTS python REQUIRED)

# Without this, any build libraries automatically have names "lib{x}.so"
set(CMAKE_SHARED_MODULE_PREFIX "")

#
# static library
#

# header search path
include_directories(src)

# static library source files
file (GLOB MAESTRO_HAPTICS_SOURCES src/*.cpp src/*.h)

# static library
add_library (maestro_chai3d STATIC ${MAESTRO_HAPTICS_SOURCES} include/mr/mr.cpp)

#${Boost_LIBRARIES} ${PYTHON_LIBRARIES}
# ${catkin_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS}

# Link to libraries
target_link_libraries(maestro_chai3d PUBLIC maestro3 ${catkin_LIBRARIES} ${CHAI3D-GEL_LIBRARIES} ${CHAI3D-ODE_LIBRARIES} ${CHAI3D-BULLET_LIBRARIES} ${CHAI3D_LIBRARIES} Eigen3::Eigen glfw)

target_include_directories(maestro_chai3d PUBLIC ${PROJECT_SOURCE_DIR}/include/alglib-cpp ${PROJECT_SOURCE_DIR}/include/mr ${CHAI3D-ODE_INCLUDE_DIRS} ${CHAI3D-BULLET_INCLUDE_DIRS} ${CHAI3D-GEL_INCLUDE_DIRS})

# build examples
add_subdirectory(examples)

# build the experiment directory
# add_subdirectory(experiments)

