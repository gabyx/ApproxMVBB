include(DefineVersion)
MACRO(ApproxMVBB_WRITE_CONFIG_FILE ApproxMVBB_CONFIG_FILE ApproxMVBB_ROOT_DIR )

    defineVersion(${ApproxMVBB_ROOT_DIR})

    configure_file(
      ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/Config/Config.hpp.in.cmake
      ${ApproxMVBB_CONFIG_FILE}
    )

ENDMACRO()

