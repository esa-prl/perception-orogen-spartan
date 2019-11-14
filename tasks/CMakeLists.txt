# Generated from orogen/lib/orogen/templates/tasks/CMakeLists.txt

include(spartanTaskLib)

set(SPARTAN_SRC_DIR "/home/marta/compass/software/vomap/src")
set(SPARTAN_BUILD_DIR "/home/marta/compass/software/vomap/src/build")


## SPARTAN imgio headers ##
include_directories("${SPARTAN_SRC_DIR}")
include_directories("${SPARTAN_SRC_DIR}/imgio")
include_directories("${SPARTAN_SRC_DIR}/VO")
include_directories("${SPARTAN_SRC_DIR}/multires")
include_directories("${SPARTAN_SRC_DIR}/sweep")

ADD_LIBRARY(${SPARTAN_TASKLIB_NAME} SHARED
    ${SPARTAN_TASKLIB_SOURCES})

add_library(vo STATIC IMPORTED)
set_property(TARGET vo PROPERTY IMPORTED_LOCATION "${SPARTAN_BUILD_DIR}/VO/libvo.a")

add_library(mr STATIC IMPORTED)
set_property(TARGET mr PROPERTY IMPORTED_LOCATION "${SPARTAN_BUILD_DIR}/multires/libmultires.a")

add_library(sw STATIC IMPORTED)
set_property(TARGET sw PROPERTY IMPORTED_LOCATION "${SPARTAN_BUILD_DIR}/sweep/libsweep.a")

## SPARTAN imgio library import ##
add_library(imgio STATIC IMPORTED)
set_property(TARGET imgio PROPERTY IMPORTED_LOCATION "${SPARTAN_BUILD_DIR}/imgio/libimgio.a")

add_dependencies(${SPARTAN_TASKLIB_NAME}
    regen-typekit)

TARGET_LINK_LIBRARIES(${SPARTAN_TASKLIB_NAME}
    ${OrocosRTT_LIBRARIES}
    ${QT_LIBRARIES}
    ${SPARTAN_TASKLIB_DEPENDENT_LIBRARIES}
    imgio
    jpeg
    vo
    lapack
    blas
    mr
    sw)

SET_TARGET_PROPERTIES(${SPARTAN_TASKLIB_NAME}
    PROPERTIES LINK_INTERFACE_LIBRARIES "${SPARTAN_TASKLIB_INTERFACE_LIBRARIES}")
SET_TARGET_PROPERTIES(${SPARTAN_TASKLIB_NAME}
    PROPERTIES INTERFACE_LINK_LIBRARIES "${SPARTAN_TASKLIB_INTERFACE_LIBRARIES}")

INSTALL(TARGETS ${SPARTAN_TASKLIB_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/orocos)

INSTALL(FILES ${SPARTAN_TASKLIB_HEADERS}
    DESTINATION include/orocos/spartan)

