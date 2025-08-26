px4_add_module(
    MODULE drivers__hello
    MAIN hello
    COMPILE_FLAGS
    SRCS
        hello.cpp
    DEPENDS
    )
