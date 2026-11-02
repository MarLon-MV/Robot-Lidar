cmake_minimum_required(VERSION 3.8)
project(mtrb_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # xmllint on package.xml; the URDF itself is checked by test_urdf.py.
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_urdf test/test_urdf.py)
endif()

ament_package()
