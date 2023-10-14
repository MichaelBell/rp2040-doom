add_library(common_pico INTERFACE)
target_sources(common_pico INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/i_glob.c
        ${CMAKE_CURRENT_LIST_DIR}/i_input.c
        ${CMAKE_CURRENT_LIST_DIR}/i_system.c
        ${CMAKE_CURRENT_LIST_DIR}/piconet.c
        ${CMAKE_CURRENT_LIST_DIR}/i_timer.c
        ${CMAKE_CURRENT_LIST_DIR}/i_video.c
        ${CMAKE_CURRENT_LIST_DIR}/stubs.c

        ${CMAKE_CURRENT_LIST_DIR}/picovision/aps6404.cpp
        ${CMAKE_CURRENT_LIST_DIR}/picovision/swd_load.cpp
        ${CMAKE_CURRENT_LIST_DIR}/picovision/picovision.cpp

        ${CMAKE_CURRENT_LIST_DIR}/i_picosound.c
)
if (PICO_ON_DEVICE)
    target_sources(common_pico INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/blit.S
            ${CMAKE_CURRENT_LIST_DIR}/picoflash.c
            )
    pico_wrap_function(common_pico malloc)
    pico_wrap_function(common_pico calloc)
    pico_wrap_function(common_pico free)
    target_compile_definitions(common_pico INTERFACE
            PICO_HEAP_SIZE=0
            USE_ZONE_FOR_MALLOC=1
    )
    target_link_libraries(common_pico INTERFACE  hardware_i2c)
endif()
target_include_directories(common_pico INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/..
        ${CMAKE_CURRENT_LIST_DIR}/../../textscreen
        )

target_compile_definitions(common_pico INTERFACE
        NO_USE_MOUSE=1
        PICO_AUDIO_I2S_PIO=1
        PICO_AUDIO_I2S_DMA_IRQ=1
        SDCARD_SPI_BUS=spi1
        SDCARD_PIN_SPI0_CS=15
        SDCARD_PIN_SPI0_SCK=10
        SDCARD_PIN_SPI0_MOSI=11
        SDCARD_PIN_SPI0_MISO=12        
        )

pico_generate_pio_header(common_pico ${CMAKE_CURRENT_LIST_DIR}/video_doom.pio)
pico_generate_pio_header(common_pico ${CMAKE_CURRENT_LIST_DIR}/picovision/aps6404.pio)
pico_generate_pio_header(common_pico ${CMAKE_CURRENT_LIST_DIR}/picovision/swd.pio)

include(${CMAKE_CURRENT_LIST_DIR}/picovision/fatfs/fatfs.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/picovision/sdcard/sdcard.cmake)

target_link_libraries(common_pico INTERFACE pico_stdlib pico_multicore sdcard fatfs) #pico_scanvideo_dpi)

add_library(pico_cd INTERFACE)
if (TARGET tinyusb_host)
    target_link_libraries(pico_cd INTERFACE tinyusb_host)
endif()