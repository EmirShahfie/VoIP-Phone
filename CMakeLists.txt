cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr)
project(my_zephyr_app)

zephyr_include_directories(include)

target_sources(app PRIVATE src/main.c)

target_sources(app PRIVATE src/wm8960.c)
