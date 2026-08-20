#--------------------------------------------------------
# The CMakeLists.txt for:                       iSerialWitMotion
# Author(s):                              labs247
#--------------------------------------------------------

INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}/lib/HWT9053
  ${CMAKE_SOURCE_DIR}/lib/MadgwickAHRS
  ${CMAKE_SOURCE_DIR}/lib/SimpleKalman
  ${CMAKE_SOURCE_DIR}/lib/nlohmann
)

SET(SRC
  SerialWitMotion.cpp
  SerialWitMotion_Info.cpp
  main.cpp
  ${CMAKE_SOURCE_DIR}/lib/HWT9053/HWT9053.cpp
  ${CMAKE_SOURCE_DIR}/lib/MadgwickAHRS/MadgwickAHRS.cpp
  ${CMAKE_SOURCE_DIR}/lib/SimpleKalman/SimpleKalman.cpp
)

ADD_EXECUTABLE(iSerialWitMotion ${SRC})

TARGET_LINK_LIBRARIES(iSerialWitMotion
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)
