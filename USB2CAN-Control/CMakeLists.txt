add_executable(control
    main.cpp
    math_ops.cpp
    message_ops.cpp
    serial_listener.cpp
)

target_link_libraries(control libcserialport)
