cmake_minimum_required(VERSION 2.8)

FIND_PACKAGE(yaml-cpp QUIET)

IF(NOT yaml-cpp_FOUND)
   find_package(PkgConfig QUIET)
   IF(PKG_CONFIG_FOUND)
      pkg_check_modules(yaml_cpp QUIET yaml-cpp)
	  IF(yaml_cpp_FOUND)
         SET(YAML_CPP_LIBRARIES ${yaml_cpp_LIBRARIES})
         SET(YAML_CPP_INCLUDE_DIRS ${yaml_cpp_INCLUDEDIR})
         SET(yaml-cpp_FOUND ${yaml_cpp_FOUND})
	  ENDIF(yaml_cpp_FOUND)
   ENDIF(PKG_CONFIG_FOUND)
ENDIF(NOT yaml-cpp_FOUND)

IF(yaml-cpp_FOUND)

# inside rtabmap project (see below for external build)
SET(RTABMap_INCLUDE_DIRS 
    ${PROJECT_BINARY_DIR}/corelib/include
    ${PROJECT_SOURCE_DIR}/utilite/include
	${PROJECT_SOURCE_DIR}/corelib/include
)
SET(RTABMap_LIBRARIES 
    rtabmap_core
	rtabmap_utilite
)  
	
if(POLICY CMP0020)
	cmake_policy(SET CMP0020 NEW)
endif()

SET(INCLUDE_DIRS
	${RTABMap_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)

SET(LIBRARIES
	${RTABMap_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
	${YAML_CPP_LIBRARIES}
)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_EXECUTABLE(mbari main.cpp)
  
TARGET_LINK_LIBRARIES(mbari ${LIBRARIES})

SET_TARGET_PROPERTIES( mbari 
    PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-mbari)
    
INSTALL(TARGETS mbari
	RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
	BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)
	
ELSE()
  MESSAGE(STATUS "yaml-cpp not found, mbari tool won't be built...")
ENDIF()
