cmake_minimum_required(VERSION 3.8)
project(putm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

link_directories(/home/putm/acados/lib)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(putm_vcl_interfaces REQUIRED)
find_package(putm_vcl REQUIRED)
find_package(vectornav_msgs REQUIRED)

add_executable(controller 
    src/controller.cpp 
    src/putm_sim_gen/acados_solver_tv_nmpc.c 
    src/putm_sim_gen/tv_nmpc_model/tv_nmpc_expl_ode_fun.c 
    src/putm_sim_gen/tv_nmpc_model/tv_nmpc_expl_vde_forw.c 
    src/putm_sim_gen/tv_nmpc_model/tv_nmpc_expl_vde_adj.c 
    src/putm_sim_gen/tv_nmpc_cost/tv_nmpc_cost_ext_cost_0_fun.c
    src/putm_sim_gen/tv_nmpc_cost/tv_nmpc_cost_ext_cost_0_fun_jac.c
    src/putm_sim_gen/tv_nmpc_cost/tv_nmpc_cost_ext_cost_0_fun_jac_hess.c
    src/putm_sim_gen/tv_nmpc_cost/tv_nmpc_cost_ext_cost_fun.c
    src/putm_sim_gen/tv_nmpc_cost/tv_nmpc_cost_ext_cost_fun_jac.c
    src/putm_sim_gen/tv_nmpc_cost/tv_nmpc_cost_ext_cost_fun_jac_hess.c
    src/putm_sim_gen/tv_nmpc_cost/tv_nmpc_cost_ext_cost_e_fun.c
    src/putm_sim_gen/tv_nmpc_cost/tv_nmpc_cost_ext_cost_e_fun_jac.c
    src/putm_sim_gen/tv_nmpc_cost/tv_nmpc_cost_ext_cost_e_fun_jac_hess.c
    src/putm_sim_gen/tv_nmpc_constraints/tv_nmpc_constr_h_fun.c
    src/putm_sim_gen/tv_nmpc_constraints/tv_nmpc_constr_h_fun_jac_uxt_zt.c
)
target_include_directories(
  controller
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/putm_sim_gen>
         /home/putm/acados/include
         /home/putm/acados/include/blasfeo/include
         /home/putm/acados/include/hpipm/include
        ${EIGEN3_INCLUDE_DIRS}
        )

target_link_libraries(controller acados hpipm blasfeo m)
ament_target_dependencies(controller rclcpp putm_vcl_interfaces putm_vcl vectornav_msgs) 
target_compile_features(controller PUBLIC c_std_99 cxx_std_17)

install(TARGETS controller DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
