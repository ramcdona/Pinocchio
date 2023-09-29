CMAKE_MINIMUM_REQUIRED(VERSION 3.21)

FIND_PACKAGE(FLTK REQUIRED)

IF ( FLTK_FOUND )

  #   FLTK_WRAP_UI( demoscreens
  #   demo.fl
  #   )

  #  ADD_LIBRARY(demoscreens ${demoscreens_FLTK_UI_SRCS} )

  ADD_EXECUTABLE(demoui
          DemoUI.cpp
          MyWindow.cpp
          defmesh.cpp
          filter.cpp
          motion.cpp
          processor.cpp
          DisplayMesh.h
          MyWindow.h
          defmesh.h
          filter.h
          motion.h
          processor.h
          shared.h
  )

  TARGET_INCLUDE_DIRECTORIES(demoui PUBLIC
          ${PINOCCHIO_INCLUDE_DIR}
          ${FLTK_INCLUDE_DIR}
  )

  TARGET_LINK_LIBRARIES(demoui PUBLIC
    pinocchio
    ${FLTK_LIBRARIES}
    ${OPENGL_LIBRARIES}
  )

ENDIF ()
