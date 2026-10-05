# The import-cache exactness diagnostic: the kernel's own sources plus diag.cpp, native only.
# SPDX-License-Identifier: LGPL-2.1-only
cmake_minimum_required(VERSION 3.20)
project(cadoc_cache_diag CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(OpenCASCADE REQUIRED CONFIG)
set(K ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
add_executable(cadoc-cache-diag diag.cpp ${K}/kernel.cpp ${K}/session.cpp ${K}/import.cpp ${K}/mesh.cpp ${K}/hlr.cpp
               ${K}/curves.cpp ${K}/spatial.cpp ${K}/shapes.cpp)
target_include_directories(cadoc-cache-diag PRIVATE ${K} ${OpenCASCADE_INCLUDE_DIR})
target_link_libraries(cadoc-cache-diag PRIVATE TKDESTEP TKXCAF TKHLR TKMesh TKBO TKShHealing TKPrim TKTopAlgo TKGeomAlgo
                      TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel pthread dl)
if(CADOC_HAVE_BIN5)
  target_compile_definitions(cadoc-cache-diag PRIVATE CADOC_HAVE_BIN5)
endif()

# The standalone reproduction of patch 0004's bug (UPSTREAM.md): OCCT only, no kernel code.
add_executable(bintools-repro bintools-repro.cpp)
target_include_directories(bintools-repro PRIVATE ${OpenCASCADE_INCLUDE_DIR})
target_link_libraries(bintools-repro PRIVATE TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel pthread dl)

# The standalone reproduction of patch 0001's bug (UPSTREAM.md): OCCT only.
add_executable(shapefix-repro shapefix-repro.cpp)
target_include_directories(shapefix-repro PRIVATE ${OpenCASCADE_INCLUDE_DIR})
target_link_libraries(shapefix-repro PRIVATE TKShHealing TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel pthread dl)
