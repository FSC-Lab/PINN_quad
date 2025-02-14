# add a single executable
function(add_single_exc required_args)
    cmake_parse_arguments(PARSE_ARGV
        0
        EXC
        ""
        "TARGET;SOURCE"
        "LIBRARIES;COMPILE_OPTIONS;DIRECTORIES_INCLUDE"
    )
    add_executable(${EXC_TARGET} ${EXC_SOURCE})
    target_link_libraries(${EXC_TARGET} ${EXC_LIBRARIES})
    target_compile_options(${EXC_TARGET} PRIVATE ${EXC_COMPILE_OPTIONS})
    target_include_directories(${EXC_TARGET} PRIVATE ${EXC_DIRECTORIES_INCLUDE})
endfunction()

# batch add mutiple targets
function(batch_add_exc required_args)
    cmake_parse_arguments(PARSE_ARGV
        0
        EXCS
        ""
        ""
        "SOURCE_LIST;LIBRARIES;COMPILE_OPTIONS;DIRECTORIES_INCLUDE"
    )

    foreach(fileName ${EXCS_SOURCE_LIST})
        get_filename_component(targetname ${fileName} NAME_WE)
        add_single_exc(
            TARGET ${targetname}
            SOURCE ${fileName}
            LIBRARIES ${EXCS_LIBRARIES}
            COMPILE_OPTIONS ${EXCS_COMPILE_OPTIONS}
            DIRECTORIES_INCLUDE ${EXCS_DIRECTORIES_INCLUDE}
        )
    endforeach()
endfunction()