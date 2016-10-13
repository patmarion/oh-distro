add_executable(drc-contact-monitor contact-monitor.cpp)
target_link_libraries(drc-contact-monitor lcm)
pods_use_pkg_config_packages(drc-contact-monitor lcmtypes_bot2-core)
pods_install_executables(drc-contact-monitor)