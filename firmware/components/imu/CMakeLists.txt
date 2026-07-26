# imu.cpp is pure maths + the stub. The BNO085 SH-2 driver joins this list
# when the part arrives (see firmware/README.md, v2 seams).
idf_component_register(SRCS "imu.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES driver)
