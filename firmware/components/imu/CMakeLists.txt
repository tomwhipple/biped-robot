# imu.cpp and fusion.cpp are pure maths -- they compile and run on the host
# under sanitizers (firmware/host), and the attitude filter is tested there.
# qmi8658_idf.cpp is the half with I2C in it and only builds for the target;
# that split is the same seam main/scs_port_idf.cpp uses for the servo bus.
idf_component_register(SRCS "imu.cpp" "fusion.cpp" "qmi8658_idf.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES driver esp_driver_i2c esp_timer)
