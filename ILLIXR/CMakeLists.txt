SET(targetname ILLIXR_plugin)

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenNI.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UsePNG.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseRealSense.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseRealSense2.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseUVC.cmake)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -std=c++17 -Wno-deprecated-declarations -Wno-unused-function")

message(STATUS "ILLIXR CUDA Include Directory: ${CUDA_INCLUDE_DIRS}")
message(STATUS "ILLIXR CUDA Library Directory: ${CUDA_LIBRARY_DIRS}")
add_library(plugin SHARED plugin.cpp)



