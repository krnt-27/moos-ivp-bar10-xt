#--------------------------------------------------------
# The CMakeLists.txt for:                       iBar10XT
# Author(s):                              Labs247
#--------------------------------------------------------

# Include the KellerLD library from lib/
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/lib/KellerLD)

SET(SRC
  Bar10XT.cpp
  Bar10XT_Info.cpp
  main.cpp
)

# Build KellerLD as an object library (from lib/)
ADD_LIBRARY(KellerLD STATIC
  ${CMAKE_SOURCE_DIR}/lib/KellerLD/KellerLD.cpp
)

ADD_EXECUTABLE(iBar10XT ${SRC})

TARGET_LINK_LIBRARIES(iBar10XT
   ${MOOS_LIBRARIES}
   KellerLD
   apputil
   mbutil
   m
   pthread)
