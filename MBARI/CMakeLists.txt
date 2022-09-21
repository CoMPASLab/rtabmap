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

SET(RTABMap_INCLUDE_DIRS 
    ${PROJECT_BINARY_DIR}/corelib/include
    ${PROJECT_SOURCE_DIR}/utilite/include
	${PROJECT_SOURCE_DIR}/corelib/include
    ${PROJECT_SOURCE_DIR}/guilib/include
)
SET(RTABMap_LIBRARIES 
    rtabmap_core
    rtabmap_gui
	rtabmap_utilite
)  
	
# Find Qt5 first
FIND_PACKAGE(Qt5 COMPONENTS Widgets Core Gui Svg QUIET)
IF(NOT Qt5_FOUND)
    FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtSvg)
ENDIF(NOT Qt5_FOUND)
if(POLICY CMP0020)
	cmake_policy(SET CMP0020 NEW)
endif()

SET(INCLUDE_DIRS
	${RTABMap_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)

IF(QT4_FOUND)
    INCLUDE(${QT_USE_FILE})
ENDIF(QT4_FOUND)

SET(LIBRARIES
	${RTABMap_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
	${YAML_CPP_LIBRARIES}
)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

IF(QT4_FOUND)
    QT4_WRAP_CPP(moc_srcs MapBuilder.h)
ELSE()
    QT5_WRAP_CPP(moc_srcs MapBuilder.h)
ENDIF()

ADD_EXECUTABLE(mbari mbari.cpp)
ADD_EXECUTABLE(mbari-3d-mapping mbari.cpp ${moc_srcs})

TARGET_COMPILE_OPTIONS(mbari-3d-mapping PRIVATE -DBUILD_WITH_3D_MAPPING)
  
TARGET_LINK_LIBRARIES(mbari ${LIBRARIES})
TARGET_LINK_LIBRARIES(mbari-3d-mapping ${LIBRARIES} ${QT_LIBRARIES})

SET_TARGET_PROPERTIES( mbari 
    PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-mbari)
SET_TARGET_PROPERTIES( mbari-3d-mapping 
    PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-mbari-3d-mapping)
    
INSTALL(TARGETS mbari mbari-3d-mapping
	RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
	BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)
	
ELSE()
  MESSAGE(STATUS "yaml-cpp not found, mbari tool won't be built...")
ENDIF()
