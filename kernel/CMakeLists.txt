# cadoc kernel binding. One source tree, two targets:
#   - Emscripten: cadoc-kernel.js + cadoc-kernel.wasm (browser Web Worker / Node)
#   - native:     cadoc-kernel-cli (cloud job subprocess, benchmarks)
# Configure with -DOpenCASCADE_DIR=<occt-install>/lib/cmake/opencascade.
# Driven by build/build-kernel.sh; see build/README in kernel/README.md.
# SPDX-License-Identifier: LGPL-2.1-only
cmake_minimum_required(VERSION 3.20)
project(cadoc_kernel CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCASCADE REQUIRED CONFIG)

set(OCCT_LIBS TKDESTEP TKXCAF TKHLR TKMesh TKBO TKShHealing TKPrim TKTopAlgo TKGeomAlgo
              TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel)

add_library(cadoc_kernel_core STATIC src/kernel.cpp src/session.cpp src/import.cpp src/mesh.cpp src/hlr.cpp
            src/curves.cpp src/spatial.cpp src/shapes.cpp)
target_include_directories(cadoc_kernel_core PUBLIC src ${OpenCASCADE_INCLUDE_DIR})
target_link_libraries(cadoc_kernel_core PUBLIC ${OCCT_LIBS})

if(EMSCRIPTEN)
  add_executable(cadoc-kernel src/kernel_wasm.cpp)
  target_link_libraries(cadoc-kernel PRIVATE cadoc_kernel_core)
  set(CADOC_WASM_LINK_FLAGS
    -fwasm-exceptions -sSUPPORT_LONGJMP=wasm
    -sALLOW_MEMORY_GROWTH=1 -sINITIAL_MEMORY=64MB -sMAXIMUM_MEMORY=4GB -sSTACK_SIZE=8MB
    -sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=createCadocKernel
    -sENVIRONMENT=web,worker,node
    "-sEXPORTED_FUNCTIONS=_cadoc_handle,_cadoc_binary_ptr,_cadoc_binary_len,_cadoc_last_timing,_cadoc_version,_cadoc_heap_in_use,_cadoc_heap_footprint,_malloc,_free"
    "-sEXPORTED_RUNTIME_METHODS=UTF8ToString,stringToNewUTF8,lengthBytesUTF8,HEAPU8"
    ${CADOC_EXTRA_LINK_FLAGS})
  target_link_options(cadoc-kernel PRIVATE ${CADOC_WASM_LINK_FLAGS})
  if(CADOC_EXTRA_LINK_FLAGS MATCHES "mimalloc")
    target_compile_definitions(cadoc-kernel PRIVATE CADOC_NO_MALLINFO)
  endif()
  set_target_properties(cadoc-kernel PROPERTIES SUFFIX ".mjs")
else()
  add_executable(cadoc-kernel-cli src/kernel_cli.cpp)
  target_link_libraries(cadoc-kernel-cli PRIVATE cadoc_kernel_core pthread dl)
endif()
