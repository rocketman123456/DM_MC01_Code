add_executable(control
    main.cpp
    math_ops.cpp
    message_ops.cpp
)

target_link_libraries(control libcserialport)
