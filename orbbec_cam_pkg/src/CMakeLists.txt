add_executable(${PROJECT_NAME} orbbec.cpp main.cpp)

file(GLOB ORBBEC_LIBS "../lib/x86_64/*")
# file(GLOB ORBBEC_LIBS "../lib/aarch64/*")
find_package(OpenCV REQUIRED)

include_directories(../include)
include_directories(../utility)
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${ORBBEC_LIBS}
)
