idf_component_register(
    SRCS "app_main.cpp" "ctrl_task.cpp" "link_task.cpp" "cli.cpp" "imu_sampler.cpp"
         "scs_port_idf.cpp" "cal_store.cpp" "wifi_link.cpp" "obs_dump.cpp"
         "timesync.cpp" "flight_rec.cpp"
    INCLUDE_DIRS "."
    REQUIRES scsbus linkproto obs policy imu battguard balance flightrec
             driver esp_timer esp_system nvs_flash freertos
             esp_wifi esp_netif esp_event lwip esp_app_format)
