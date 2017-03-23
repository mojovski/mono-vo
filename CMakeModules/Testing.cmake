MACRO (REGISTER_GLOBAL_TEST NAME)
    #MESSAGE(STATUS ">> Registering a global lib: ${NAME}")
    GET_PROPERTY(TESTS_STORE
            GLOBAL
            PROPERTY GLOBAL_TESTS
            )

    SET_PROPERTY(GLOBAL
            PROPERTY GLOBAL_TESTS
            ${TESTS_STORE}
            ${NAME}
            )
ENDMACRO(REGISTER_GLOBAL_TEST)


MACRO (SetupTest NAME)
    set(targetname test_${NAME})
    add_executable( ${targetname} ${targetname}.cpp )
    if(${ARGC} GREATER 1)
        string(STRIP "${ARGV1}" ARGV1_stripped)
        string(LENGTH "${ARGV1_stripped}" ARGV1_stripped_length)
        if(${ARGV1_stripped_length} GREATER 0)
            #MESSAGE(STATUS "Linking libraries: ${ARGV1}")
            target_link_libraries(${targetname} ${ARGV1})
        endif(${ARGV1_stripped_length} GREATER 0)
    endif(${ARGC} GREATER 1)

    # Link test executable against gtest & gtest_main
    target_link_libraries(${targetname} gtest gtest_main)
    add_test( ${NAME} ${targetname} )
    REGISTER_GLOBAL_TEST(${targetname})

ENDMACRO(SetupTest)

