MACRO (SetupPerformanceTest NAME)
    set(targetname perf_${NAME})
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

ENDMACRO(SetupPerformanceTest)