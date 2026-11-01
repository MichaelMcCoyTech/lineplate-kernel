# Shared OCCT CMake configuration for BOTH builds (WASM and native).
# Sourced by build-occt.sh inside the build container. Anything that differs
# between the two builds is set in build-occt.sh and listed in RESULT.md.

OCCT_TAG="V8.0.1"
OCCT_COMMIT="b8f597c677811d1f9f4d8a97f5ae2825c0353a42"

# First-level toolkits; OCCT's CMake (EXCTRACT_TOOLKIT_FULL_DEPS) adds the
# full dependency closure. TKXCAF -> TKVCAF -> TKV3d/TKService drags in two
# Visualization toolkits; no OpenGL toolkit is built.
# TKDEIGES (IGES import) and TKDESTL (STL read, for STL to STEP) added for kernel 1.3.
OCCT_TOOLKITS="TKDESTEP TKDEIGES TKDESTL TKXCAF TKHLR TKMesh TKBO TKShHealing TKPrim"

# OCCT adds -DOCC_CONVERT_SIGNALS for every non-MSVC build, which makes each
# OCC_CATCH_SIGNALS a setjmp() inside a C++ try block. Under emscripten that mix
# of wasm exception handling and wasm setjmp/longjmp produces a module that
# binaryen cannot parse ("popping from empty stack", in
# ShapeUpgrade_ShapeDivide::Perform). It is undefined on BOTH builds so the
# flags stay identical; without it a hardware signal (e.g. SIGSEGV) is no longer
# turned into a C++ exception, which the one-process-per-job design tolerates.
OCCT_COMMON_CXX_FLAGS="-UOCC_CONVERT_SIGNALS"

OCCT_CMAKE_COMMON=(
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG"
  -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG"
  -DBUILD_LIBRARY_TYPE=Static
  -DBUILD_OPT_PROFILE=Default
  -DBUILD_CPP_STANDARD=C++17
  -DBUILD_USE_PCH=OFF
  -DBUILD_GTEST=OFF
  -DBUILD_DOC_Overview=OFF
  -DBUILD_DOC_RefMan=OFF
  -DBUILD_RESOURCES=OFF
  -DBUILD_MODULE_FoundationClasses=ON
  -DBUILD_MODULE_ModelingData=OFF
  -DBUILD_MODULE_ModelingAlgorithms=OFF
  -DBUILD_MODULE_Visualization=OFF
  -DBUILD_MODULE_ApplicationFramework=OFF
  -DBUILD_MODULE_DataExchange=OFF
  -DBUILD_MODULE_Draw=OFF
  -DBUILD_MODULE_Deprecated=OFF
  -DBUILD_ADDITIONAL_TOOLKITS="${OCCT_TOOLKITS}"
  -DUSE_MMGR_TYPE=NATIVE
  -DUSE_TBB=OFF
  -DUSE_FREETYPE=OFF
  -DUSE_TK=OFF
  -DUSE_TCL=OFF
  -DUSE_FREEIMAGE=OFF
  -DUSE_FFMPEG=OFF
  -DUSE_OPENVR=OFF
  -DUSE_RAPIDJSON=OFF
  -DUSE_DRACO=OFF
  -DUSE_EIGEN=OFF
  -DUSE_VTK=OFF
  -DUSE_XLIB=OFF
  -DUSE_OPENGL=OFF
  -DUSE_GLES2=OFF
  -DUSE_D3D=OFF
)
