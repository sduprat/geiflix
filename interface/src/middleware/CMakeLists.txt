message("Generating the build files for the middleware")

set(MIDDLEWARE_SOURCES 
  ${PROJECT_SOURCE_DIR}/src/middleware/src/main.cpp 
  ${PROJECT_SOURCE_DIR}/src/middleware/src/middleware.cpp
)
set(MIDDLEWARE_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src/middleware/include/)

add_executable(cristal-ui ${MIDDLEWARE_SOURCES})
target_include_directories(cristal-ui PUBLIC ${MIDDLEWARE_INCLUDE_DIRS})
target_link_libraries(cristal-ui PUBLIC cristal-graphics)
