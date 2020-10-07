cmake_minimum_required(VERSION 3.5.0)
project(libgeodesic VERSION 0.1.0 LANGUAGES CXX C)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

find_package(Boost COMPONENTS system REQUIRED)
find_package(LAPACK REQUIRED)
find_package(SuiteSparse REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/hmCholeskyFactor.c
  src/hmContext.c
  src/hmDenseMatrix.c
  src/hmPairSizeTDouble.c
  src/hmSparseMatrix.c
  src/hmTriDistance.c
  src/hmTriMesh.c
  src/hmVectorSizeT.c
  src/hmVectorDouble.c
  src/hmVectorPairSizeTDouble.c)
target_link_libraries(${PROJECT_NAME} PUBLIC ${LAPACK_LIBRARIES} ${SUITESPARSE_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${LAPACK_INCLUDE_DIRS}
    ${SUITESPARSE_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_cpp SHARED
  src/hmHeatPath.cc)
target_link_libraries(${PROJECT_NAME}_cpp PUBLIC ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_cpp PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

add_executable(heatmethod src/main.c)
target_link_libraries(heatmethod PUBLIC ${PROJECT_NAME})
target_include_directories(heatmethod PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

add_executable(heatmethod_cpp src/mymain.cc)
target_link_libraries(heatmethod_cpp PUBLIC ${PROJECT_NAME}_cpp)
target_include_directories(heatmethod_cpp PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_cpp heatmethod heatmethod_cpp
        EXPORT ${PROJECT_NAME}-targets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets NAMESPACE ${PROJECT_NAME}:: DESTINATION lib/cmake/${PROJECT_NAME})

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Create cmake config files
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

export(EXPORT ${PROJECT_NAME}-targets NAMESPACE ${PROJECT_NAME}:: FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  "${CMAKE_CURRENT_LIST_DIR}/cmake/FindSuiteSparse.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

