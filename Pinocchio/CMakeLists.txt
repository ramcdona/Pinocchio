CMAKE_MINIMUM_REQUIRED(VERSION 3.21)

SET(PINOCCHIO_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "Path to pinocchio headers")

# INCLUDE_DIRECTORIES(
# )

ADD_LIBRARY( pinocchio )
ADD_LIBRARY( pinocchio::pinocchio ALIAS pinocchio )

SET( public_headers
        Pinocchio.h
        attachment.h
        debugging.h
        deriv.h
        dtree.h
        dual_quat_cu.h
        graphutils.h
        hashutils.h
        indexer.h
        intersector.h
        lsqSolver.h
        mat3.h
        mathutils.h
        matrix.h
        pin_mesh.h
        multilinear.h
        pinocchioApi.h
        point3.h
        pointprojector.h
        quaddisttree.h
        quat_cu.h
        quatinterface.h
        rect.h
        skeleton.h
        transfo.h
        transform.h
        utils.h
        vec3.h
        vector.h
        vecutils.h
)

SET( sources
        ${public_headers}
        Pinocchio.cpp
        attachment.cpp
        discretization.cpp
        embedding.cpp
        graphutils.cpp
        indexer.cpp
        intersector.cpp
        lsqSolver.cpp
        matrix.cpp
        pin_mesh.cpp
        pinocchioApi.cpp
        quatinterface.cpp
        refinement.cpp
        skeleton.cpp
)

TARGET_SOURCES( pinocchio PRIVATE ${sources} )

# TARGET_COMPILE_DEFINITIONS( pinocchio
#         PUBLIC
#         "$<$<NOT:$<BOOL:${BUILD_SHARED_LIBS}>>:PINOCCHIO_STATIC_DEFINE"
# )

TARGET_INCLUDE_DIRECTORIES( pinocchio
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
        PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

SET_TARGET_PROPERTIES( pinocchio PROPERTIES
        CXX_STANDARD 17
        PUBLIC_HEADER "${public_headers}"
)

IF ( BUILD_SHARED_LIBS )
        TARGET_COMPILE_DEFINITIONS( pinocchio PUBLIC PINOCCHIO_DLL=1 )
ENDIF ()

TARGET_INCLUDE_DIRECTORIES( pinocchio PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )

# ADD_DEPENDENCIES( pinocchio
# )

IF( PINOCCHIO_INSTALL AND NOT CMAKE_SKIP_INSTALL_RULES )

INSTALL( TARGETS pinocchio
         EXPORT pinocchio
         LIBRARY DESTINATION lib
         INCLUDES DESTINATION include
         PUBLIC_HEADER COMPONENT include
)

ENDIF ()
