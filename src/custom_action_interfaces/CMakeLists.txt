cmake_minimum_required(VERSION 3.8)
project(custom_action_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Emotion.action"
  "action/SpeakEmotion.action"  # ← Agregar esta línea
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)  # 🔹 NECESARIO para otros paquetes

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

