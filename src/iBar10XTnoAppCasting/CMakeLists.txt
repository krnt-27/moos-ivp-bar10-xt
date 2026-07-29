#--------------------------------------------------------
# The CMakeLists.txt for:                       iBar10XTnoAppCasting
# Author(s):                              Labs247
#--------------------------------------------------------

SET(SRC
  Bar10XTnoAppCasting.cpp
  Bar10XTnoAppCasting_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iBar10XTnoAppCasting ${SRC})

TARGET_LINK_LIBRARIES(iBar10XTnoAppCasting
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread)

