cmake_minimum_required(VERSION 3.8)
project(pkg_parte_3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#InstallPythonmodules
ament_python_install_package(${PROJECT_NAME})

 #InstallPythonexecutables
install(PROGRAMS nodes/mapas/path_line.txt nodes/mapas/path_sin.txt nodes/mapas/path_sqrt.txt nodes/controlador_angular.py 
nodes/follow_the_carrot.py nodes/parametros.py nodes/publicador_ruta.py nodes/publicador_velocidad.py nodes/mostrar_ruta.py DESTINATION lib/${PROJECT_NAME})

#Installlaunchfiles
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
