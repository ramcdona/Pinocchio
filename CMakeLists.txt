CMAKE_MINIMUM_REQUIRED(VERSION 3.21)

PROJECT( Pinocchio
        VERSION 1.0.0
        DESCRIPTION "Pinocchio character rigging library"
        LANGUAGES CXX
)

# PINOCCHIO_SHARED_LIBS determines static/shared build when defined

option( PINOCCHIO_CONSOLE_APP "Build pinocchio console app" OFF )
option( PINOCCHIO_GUI_APP "Build pinocchio GUI app" OFF )
option( PINOCCHIO_INSTALL "Generate target for installing mylib" ${PROJECT_IS_TOP_LEVEL} )

IF ( DEFINED PINOCCHIO_SHARED_LIBS )
    SET( BUILD_SHARED_LIBS ${PINOCCHIO_SHARED_LIBS} )
ENDIF ( )

ADD_SUBDIRECTORY( Pinocchio )

IF ( PINOCCHIO_CONSOLE_APP )
    ADD_SUBDIRECTORY( AttachWeights )
ENDIF ()

IF ( PINOCCHIO_GUI_APP )
    ADD_SUBDIRECTORY( DemoUI )
ENDIF ()
