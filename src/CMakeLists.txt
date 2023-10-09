# install header
# file(GLOB ex_headers "include/*.h")
# install(FILES ${ex_headers} DESTINATION unaryFator.h)

# Add unit tests
#gtsamAddTestsGlob(tests "tests/*.cpp" "" ${PROJECT_NAME})

# Add executables
# gtsamAddExamplesGlob("src/*.cpp" "" "${PROJECT_NAME}")


include_directories("/home/jason/VSLAM/gtsam-tutorial/gtsam_simple_demo")


### install lib, only do that if *.c or *.cpp exist
# since C srcs exist, use *.c* as pattern, not *.cpp
# file(GLOB ex_srcs "*.cpp")

# export cmake setting to the source code
# add_library(${PROJECT_NAME} SHARED ${ex_srcs} ${ex_headers})

# linking with dependences
# target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} gtsam)

add_library(${PROJECT_NAME} SHARED GpsFactor.cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} gtsam)


add_executable(slamMotionOnly slamMotionOnly.cpp )
target_link_libraries(slamMotionOnly ${PROJECT_NAME})

add_executable(slamUnaryMeasurement slamUnaryMeasurement.cpp )
target_link_libraries(slamUnaryMeasurement ${PROJECT_NAME})


