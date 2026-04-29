cmake_minimum_required(VERSION 3.10)
project(Estun_ERI_SDK_Examples)

set(ERI_INTERNAL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(ERI_INTERNAL_BIN_DIR     "${CMAKE_CURRENT_SOURCE_DIR}/bin")
set(ERI_INTERNAL_LIB_DIR     "${CMAKE_CURRENT_SOURCE_DIR}/lib")

set(ERI_CONFIG_FILES
    "${ERI_INTERNAL_BIN_DIR}/log4cpp.conf"
)

if(WIN32)
    set(ERI_LIBRARY "${ERI_INTERNAL_LIB_DIR}/EstunRobotERI.lib")
    set(ERI_RUNTIME_FILES
        "${ERI_INTERNAL_BIN_DIR}/EstunRobotERI.dll"
        "${ERI_INTERNAL_BIN_DIR}/log4cpp.dll"
    )
elseif(UNIX)
    set(ERI_LIBRARY "${ERI_INTERNAL_LIB_DIR}/libEstunRobotERI.so")
    set(LOG4CPP_LIB "${ERI_INTERNAL_LIB_DIR}/liblog4cpp.so")
    set(ERI_LIBRARIES ${ERI_LIBRARY} ${LOG4CPP_LIB})
else()
    message(FATAL_ERROR "当前示例仅支持 Windows 和 Linux。")
endif()

set(ERI_REQUIRED_FILES ${ERI_LIBRARY} ${ERI_CONFIG_FILES})
if(WIN32)
    list(APPEND ERI_REQUIRED_FILES ${ERI_RUNTIME_FILES})
elseif(UNIX)
    list(APPEND ERI_REQUIRED_FILES ${LOG4CPP_LIB})
endif()

foreach(ERI_REQUIRED_FILE IN LISTS ERI_REQUIRED_FILES)
    if(NOT EXISTS "${ERI_REQUIRED_FILE}")
        message(FATAL_ERROR "找不到文件: ${ERI_REQUIRED_FILE}，请先从 Releases 下载对应系统的二进制包，并解压到项目根目录。")
    endif()
endforeach()

function(add_eri_example target_name source_file)
    add_executable(${target_name} ${source_file})
    target_include_directories(${target_name} PRIVATE "${ERI_INTERNAL_INCLUDE_DIR}")

    if(WIN32)
        target_link_libraries(${target_name} PRIVATE "${ERI_LIBRARY}")
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${ERI_CONFIG_FILES}
                ${ERI_RUNTIME_FILES}
                $<TARGET_FILE_DIR:${target_name}>
            COMMENT "Windows: 正在同步 DLL 和配置文件..."
        )
    elseif(UNIX)
        set(output_name "${target_name}_bin")

        target_link_libraries(${target_name} PRIVATE ${ERI_LIBRARIES})
        set_target_properties(${target_name} PROPERTIES
            OUTPUT_NAME "${output_name}"
            INSTALL_RPATH "${ERI_INTERNAL_LIB_DIR}"
            BUILD_WITH_INSTALL_RPATH TRUE
        )

        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${ERI_CONFIG_FILES}
                $<TARGET_FILE_DIR:${target_name}>
            COMMENT "Linux: 正在同步配置文件..."
        )

        set(run_script_path "${CMAKE_CURRENT_BINARY_DIR}/${target_name}")
        file(WRITE "${run_script_path}" "#!/usr/bin/env bash\nSCRIPT_DIR=\"$(cd \"$(dirname \"$0\")\" && pwd)\"\nPROJECT_LIB_DIR=\"${ERI_INTERNAL_LIB_DIR}\"\nexport LD_LIBRARY_PATH=\"\$PROJECT_LIB_DIR\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}\"\ncd \"\$SCRIPT_DIR\"\nexec \"\$SCRIPT_DIR/${output_name}\" \"\$@\"\n")
        execute_process(COMMAND chmod +x "${run_script_path}")
    endif()
endfunction()

add_subdirectory(examples/01_APOS_demo)
add_subdirectory(examples/02_CPOS_demo)
add_subdirectory(examples/03_GETDO_demo)
add_subdirectory(examples/04_STATUS_CALLBACK_demo)
