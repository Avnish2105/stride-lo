# Common compile flags for STRIDE packages (installed with lo_core, used by every lo_* package).
#   stride_target_flags(<target>)
# Warnings are errors unless -DSTRIDE_WERROR=OFF. Sanitizer via -DSTRIDE_SANITIZER=address|
# undefined|thread (scripts/build.sh --sanitizer=...). Never -march=native (Jetson portability).
if(NOT DEFINED STRIDE_WERROR)
  set(STRIDE_WERROR ON)
endif()
if(NOT DEFINED STRIDE_SANITIZER)
  set(STRIDE_SANITIZER "")
endif()

function(stride_target_flags target)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wnon-virtual-dtor
    -Wcast-align -Woverloaded-virtual -Wnull-dereference)
  if(STRIDE_WERROR)
    target_compile_options(${target} PRIVATE -Werror)
  endif()
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # GCC 12/13 report false maybe-uninitialized inside Eigen fixed-size code at -O2.
    target_compile_options(${target} PRIVATE -Wno-maybe-uninitialized)
  endif()
  if(STRIDE_SANITIZER STREQUAL "address")
    target_compile_options(${target} PRIVATE -fsanitize=address -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE -fsanitize=address)
  elseif(STRIDE_SANITIZER STREQUAL "undefined")
    target_compile_options(${target} PRIVATE -fsanitize=undefined -fno-sanitize-recover=undefined
      -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE -fsanitize=undefined)
  elseif(STRIDE_SANITIZER STREQUAL "thread")
    target_compile_options(${target} PRIVATE -fsanitize=thread)
    target_link_options(${target} PRIVATE -fsanitize=thread)
  elseif(NOT STRIDE_SANITIZER STREQUAL "")
    message(FATAL_ERROR "STRIDE_SANITIZER must be address, undefined or thread")
  endif()
endfunction()
