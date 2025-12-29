cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr)
project(my_zephyr_app)

target_include_directories(app PRIVATE include)

target_sources(app PRIVATE 
                src/main.c
                src/wm8960.c
                src/audio.c
                src/network.c)
