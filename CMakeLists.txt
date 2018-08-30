cmake_minimum_required (VERSION 3.2)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

project (VisionTest)

# compiler version checking does not work if CMAKE_CXX_COMPILER is not set
if (CMAKE_CXX_COMPILER)
    execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion
                    OUTPUT_VARIABLE GCC_VERSION)
    message("GCC version: ${GCC_VERSION}")
    if (GCC_VERSION VERSION_GREATER "5.0.0" OR GCC_VERSION VERSION_EQUAL "5.0.0")
            message(STATUS "GCC version >= 5.0, use CXX11_ABI=1")
    else()
        add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
    endif()
endif()

find_package(OpenCV)
find_package(SDL2 REQUIRED)
find_package(SDL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

include_directories(
    ${GLEW_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIR}
    ${SDL2_INCLUDE_DIRS}
    )

add_executable(
    VisionTest
    src/arucoDetect.cpp
)

target_link_libraries(VisionTest ${OpenCV_LIBS}
    ${SDL2_LIBRARY} 
    ${SDL_LIBRARY} 
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    )
