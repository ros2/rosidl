# copied from rosidl_cmake/rosidl_cmake-extras.cmake

# register ament_package() hook for definitions once
macro(_rosidl_cmake_register_package_hook)
  if(NOT DEFINED _ROSIDL_CMAKE_PACKAGE_HOOK_REGISTERED)
    set(_ROSIDL_CMAKE_PACKAGE_HOOK_REGISTERED TRUE)

    find_package(ament_cmake_core REQUIRED)
    ament_register_extension("ament_package" "rosidl_cmake"
      "rosidl_cmake_package_hook.cmake")

    find_package(ament_cmake_export_dependencies REQUIRED)
  endif()
endmacro()

include("${rosidl_cmake_DIR}/rosidl_generate_interfaces.cmake")
