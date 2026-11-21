target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    app.c acquisition.c stream_encode.c scope.c measurement.c spectrum.c serial_link.c commands.c ui.c
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
