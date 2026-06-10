cmake_minimum_required(VERSION 3.22)
project(SBGC_API C)

set(CMAKE_C_STANDARD 11)

set(PATH_TO_SERIAL_API ${CMAKE_CURRENT_SOURCE_DIR})
set(PATH_TO_CORE ${PATH_TO_SERIAL_API}/core)
set(PATH_TO_DRIVERS ${PATH_TO_SERIAL_API}/drivers/driverLinux)
set(PATH_TO_MODULES ${PATH_TO_SERIAL_API}/modules)
set(PATH_TO_OS_GLUE ${PATH_TO_SERIAL_API}/os)

add_library(${PROJECT_NAME} OBJECT
        ${PATH_TO_CORE}/debug/debug.c
        ${PATH_TO_CORE}/debug/debug.h
        ${PATH_TO_CORE}/highLayer/highLayer.c
        ${PATH_TO_CORE}/highLayer/highLayer.h
        ${PATH_TO_CORE}/parser/parser.c
        ${PATH_TO_CORE}/parser/parser.h
        ${PATH_TO_CORE}/commandBuild.c
        ${PATH_TO_CORE}/core.h
        ${PATH_TO_CORE}/lowLayer.c

        ${PATH_TO_DRIVERS}/driverLinux.c
        ${PATH_TO_DRIVERS}/driverLinux.h

        ${PATH_TO_MODULES}/adjvar/adjvar.c
        ${PATH_TO_MODULES}/adjvar/adjvar.h
        ${PATH_TO_MODULES}/calib/calib.c
        ${PATH_TO_MODULES}/calib/calib.h
        ${PATH_TO_MODULES}/eeprom/eeprom.c
        ${PATH_TO_MODULES}/eeprom/eeprom.h
        ${PATH_TO_MODULES}/gimbalControl/gimbalControl.c
        ${PATH_TO_MODULES}/gimbalControl/gimbalControl.h
        ${PATH_TO_MODULES}/imu/imu.c
        ${PATH_TO_MODULES}/imu/imu.h
        ${PATH_TO_MODULES}/profiles/profiles.c
        ${PATH_TO_MODULES}/profiles/profiles.h
        ${PATH_TO_MODULES}/realtime/realtime.c
        ${PATH_TO_MODULES}/realtime/realtime.h
        ${PATH_TO_MODULES}/service/service.c
        ${PATH_TO_MODULES}/service/service.h

        ${PATH_TO_OS_GLUE}/glueAzureRTOS/glueAzureRTOS.c
        ${PATH_TO_OS_GLUE}/glueAzureRTOS/glueAzureRTOS.h
        ${PATH_TO_OS_GLUE}/glueFreeRTOS/glueFreeRTOS.c
        ${PATH_TO_OS_GLUE}/glueFreeRTOS/glueFreeRTOS.h
        ${PATH_TO_OS_GLUE}/glueLinuxOS/glueLinuxOS.c
        ${PATH_TO_OS_GLUE}/glueLinuxOS/glueLinuxOS.h
  
        ${PATH_TO_SERIAL_API}/adjunct.h
        ${PATH_TO_SERIAL_API}/sbgc32.c
        ${PATH_TO_SERIAL_API}/sbgc32.h
        ${PATH_TO_SERIAL_API}/serialAPI_Config.h)

