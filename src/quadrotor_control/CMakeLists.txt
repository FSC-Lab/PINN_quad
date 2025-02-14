add_subdirectory(controller)
set(TARGET_NAME quadrotor_control1)

add_executable(${TARGET_NAME} quadrotor_control_1.cpp)
target_link_directories(${TARGET_NAME} PUBLIC ${MOSEK_LIB_DIR})
target_link_libraries(${TARGET_NAME} PRIVATE 
    ${LASAN}
    mosek_fusion_lib
    mosek64
    Eigen3::Eigen
    convex_psd_lib
    applications
    -Wl,--whole-archive mdss -Wl,--no-whole-archive
    utility_functions)
target_compile_options(${TARGET_NAME} PRIVATE ${COMPILE_FLAGS})


add_executable(pointStabilization point_stabilization.cpp)
target_link_directories(pointStabilization PUBLIC ${MOSEK_LIB_DIR})
target_link_libraries(pointStabilization PRIVATE 
    ${LASAN}
    mosek_fusion_lib
    mosek64
    Eigen3::Eigen
    convex_psd_lib
    applications
    quad_pos_pid
    -Wl,--whole-archive mdss -Wl,--no-whole-archive
    utility_functions)
target_compile_options(pointStabilization PRIVATE ${COMPILE_FLAGS})


add_executable(quad_pid_test quad_sim_test.cpp)
target_link_directories(quad_pid_test PUBLIC ${MOSEK_LIB_DIR})
target_link_libraries(quad_pid_test PRIVATE 
    ${LASAN}
    mosek_fusion_lib
    mosek64
    Eigen3::Eigen
    convex_psd_lib
    applications
    quad_pos_pid
    -Wl,--whole-archive mdss -Wl,--no-whole-archive
    utility_functions)
target_compile_options(quad_pid_test PRIVATE ${COMPILE_FLAGS})


add_executable(quad_ccm_test1 quad_ccm_test1.cpp)
target_link_directories(quad_ccm_test1 PUBLIC ${MOSEK_LIB_DIR})
target_link_libraries(quad_ccm_test1 PRIVATE 
    ${LASAN}
    mosek_fusion_lib
    mosek64
    Eigen3::Eigen
    convex_psd_lib
    applications
    quad_pos_pid
    -Wl,--whole-archive mdss -Wl,--no-whole-archive
    utility_functions)
target_compile_options(quad_ccm_test1 PRIVATE ${COMPILE_FLAGS})