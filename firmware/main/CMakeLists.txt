idf_component_register(
    SRCS "app_main.cpp" "ctrl_task.cpp" "link_task.cpp" "cli.cpp" "imu_sampler.cpp"
         "scs_port_idf.cpp" "cal_store.cpp" "wifi_link.cpp" "obs_dump.cpp"
         "timesync.cpp" "flight_rec.cpp" "net_bench.cpp" "wifi_cli.cpp" "power_rail.cpp"
    INCLUDE_DIRS "."
    REQUIRES scsbus linkproto obs policy imu battguard balance flightrec fallguard flailguard railguard wcli
             driver esp_timer esp_system esp_hw_support nvs_flash freertos
             esp_wifi esp_netif esp_event lwip esp_app_format)
# Power lite reads RAIL_V on the ADC. Only there: esp_adc on the link line replaces
# the weak adc2_wifi_acquire stubs Wi-Fi links against, and a default image must not
# change. It is already part of the build, so a late link is enough (and an `if` in
# REQUIRES would not work: IDF reads those before this variable exists).
if(BIMO_POWER_LITE)
    target_link_libraries(${COMPONENT_LIB} PRIVATE idf::esp_adc)
endif()
