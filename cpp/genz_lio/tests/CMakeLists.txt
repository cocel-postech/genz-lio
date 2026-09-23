foreach(test_name IN ITEMS core_types downsample scale_aware_voxelizer imu_processor
                          preprocessor hybrid_metric pipeline imu_lifecycle filter_update
                          deskew_geometry plane_refit plane_degeneracy adaptive_gate scan_buffer worker_threads observation_product)
  add_executable(test_${test_name} test_${test_name}.cpp)
  target_link_libraries(test_${test_name} PRIVATE genz_lio::core)
  add_test(NAME ${test_name} COMMAND test_${test_name})
endforeach()

if(GENZ_LIO_WITH_YAML)
  add_executable(test_config_io test_config_io.cpp)
  target_link_libraries(test_config_io PRIVATE genz_lio::config)
  add_test(NAME config_io COMMAND test_config_io)
  # Points the test at the shipped configuration files.
  set_tests_properties(config_io PROPERTIES ENVIRONMENT
    "GENZ_LIO_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../../../ros/config")
endif()
