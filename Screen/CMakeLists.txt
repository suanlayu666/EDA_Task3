target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    lcd.c touch.c iic.c 24cxx.c screen_delay.c
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
