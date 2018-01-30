# Make targets library shared/static ============================================
macro(add_approxmvbb_lib STATICorSHARED TARGETNAME EXPORTNAME DOINSTALL)
    add_library(${TARGETNAME} ${STATICorSHARED} ${ApproxMVBB_SRC} ${ApproxMVBB_INC})
    target_include_directories(${TARGETNAME} PUBLIC ${ApproxMVBB_INC_DIRS} $<INSTALL_INTERFACE:include>)
    target_link_libraries(${TARGETNAME} PUBLIC ${ApproxMVBB_LIBS_DEP_PUBLIC} PRIVATE ${ApproxMVBB_LIBS_DEP_PRIVATE})
    add_library("ApproxMVBB::${EXPORTNAME}" ALIAS ${TARGETNAME})

    set_target_properties(${TARGETNAME} 
        PROPERTIES
        VERSION ${ApproxMVBB_VERSION_STRING} SOVERSION ${ApproxMVBB_VERSION_STRING}
        DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
        EXPORT_NAME ${EXPORTNAME})

    setTargetCompileOptions(${TARGETNAME})
    
    # set build flag, for import/export specifiers in platform.hpp
    # Building a library means we only export the symbols which have APPROXMVBB_EXPORT set in the Platform.hpp file
    # We define all other symbols to be hidden by default as suggested by https://gcc.gnu.org/wiki/Visibility
    if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR 
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(${TARGETNAME} PRIVATE -DApproxMVBB_BUILD_LIBRARY -fvisibility=hidden)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        target_compile_options(${TARGETNAME} PRIVATE /DApproxMVBB_BUILD_LIBRARY)
    else()
        message(FATAL_ERROR "Could not set compile options for compiler id: '${CMAKE_CXX_COMPILER_ID}'")
    endif()

    if(${DOINSTALL})
        install(TARGETS ${TARGETNAME} EXPORT ApproxMVBBTargets
                RUNTIME       DESTINATION ${ApproxMVBB_RUNTIME_INSTALL_DIR}
                LIBRARY       DESTINATION ${ApproxMVBB_LIBRARY_INSTALL_DIR}
                ARCHIVE       DESTINATION ${ApproxMVBB_ARCHIVE_INSTALL_DIR}
                FRAMEWORK     DESTINATION ${ApproxMVBB_FRAMEWORK_INSTALL_DIR})
    endif()

endmacro()

# Make library targets
add_approxmvbb_lib("SHARED" "ApproxMVBB" "Core" ON)
if(ApproxMVBB_BUILD_LIBRARY_STATIC)
    add_approxmvbb_lib("STATIC" "ApproxMVBB_s" "CoreS" ON)
endif()

# Make support targets for KdTree
if(${ApproxMVBB_KDTREE_SUPPORT})
    add_library(ApproxMVBB_KdTreeSupport INTERFACE)
    add_library("ApproxMVBB::KdTreeSupport" ALIAS ApproxMVBB_KdTreeSupport)
    if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR 
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        set_target_properties(ApproxMVBB_KdTreeSupport PROPERTIES INTERFACE_COMPILE_OPTIONS "-DApproxMVBB_KDTREE_SUPPORT")
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        set_target_properties(ApproxMVBB_KdTreeSupport PROPERTIES INTERFACE_COMPILE_OPTIONS "/DApproxMVBB_KDTREE_SUPPORT")
    else()
        message(FATAL_ERROR "Could not set compile options for compiler id: '${CMAKE_CXX_COMPILER_ID}'")
    endif()
    set_target_properties(ApproxMVBB_KdTreeSupport  PROPERTIES 
                                                    INTERFACE_LINK_LIBRARIES metaLib
                                                    EXPORT_NAME "KdTreeSupport")

    install(TARGETS ApproxMVBB_KdTreeSupport EXPORT ApproxMVBBTargets)
endif()

# Make support targets for XML
if(${ApproxMVBB_XML_SUPPORT})
    add_library(ApproxMVBB_XML_SUPPORT INTERFACE)
    add_library("ApproxMVBB::XMLSupport" ALIAS ApproxMVBB_XML_SUPPORT)
    if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR 
         CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        set_target_properties(ApproxMVBB_XML_SUPPORT PROPERTIES INTERFACE_COMPILE_OPTIONS "-DApproxMVBB_XML_SUPPORT")
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        set_target_properties(ApproxMVBB_XML_SUPPORT PROPERTIES INTERFACE_COMPILE_OPTIONS "-DApproxMVBB_XML_SUPPORT")
    else()
        message(FATAL_ERROR "Could not set compile options for compiler id: '${CMAKE_CXX_COMPILER_ID}'")
    endif()
    set_target_properties(ApproxMVBB_XML_SUPPORT PROPERTIES
                                                 INTERFACE_LINK_LIBRARIES pugixml
                                                 EXPORT_NAME "XMLSupport")

    install(TARGETS ApproxMVBB_XML_SUPPORT EXPORT ApproxMVBBTargets)
endif()

# ======================================================================

# Make target library for tests ================================================
if(ApproxMVBB_BUILD_TESTS)
    add_approxmvbb_lib("SHARED" "ApproxMVBBForTestsOnly" "CoreForTests" OFF)

    # set build flag, for import/export specifiers in platform.hpp
    # Building a library means we only export the symbols which have APPROXMVBB_EXPORT set in the Platform.hpp file
    # We define all other symbols to be hidden by default as suggested by https://gcc.gnu.org/wiki/Visibility
    if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR 
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(ApproxMVBBForTestsOnly PRIVATE -DApproxMVBB_BUILD_TESTS)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        target_compile_options(ApproxMVBBForTestsOnly PRIVATE /DApproxMVBB_BUILD_TESTS)
    else()
        message(FATAL_ERROR "Could not set compile options for compiler id: '${CMAKE_CXX_COMPILER_ID}'")
    endif()
endif()
# ======================================================================

# Make cmake config files
install(EXPORT  ApproxMVBBTargets
                DESTINATION ${ApproxMVBB_CMAKE_CONFIG_INSTALL_DIR}
                NAMESPACE ApproxMVBB::
                FILE approxmvbb-config.cmake)

# Write dependency cmake config file
configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/approxmvbb-config-dependencies.cmake"
                "${CMAKE_CURRENT_BINARY_DIR}/cmake/approxmvbb-config-dependencies.cmake" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/cmake/approxmvbb-config-dependencies.cmake"
                DESTINATION "${ApproxMVBB_CMAKE_CONFIG_INSTALL_DIR}"
                COMPONENT Devel)

install(FILES "${ApproxMVBB_ROOT_DIR}/cmake/FindMeta.cmake"
                DESTINATION "${ApproxMVBB_CMAKE_CONFIG_INSTALL_DIR}/modules"
                COMPONENT Devel)

# Write/install version file
include(CMakePackageConfigHelpers)
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/cmake/approxmvbb-config-version.cmake"
                                 VERSION ${ApproxMVBB_VERSION_STRING}
                                 COMPATIBILITY AnyNewerVersion)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/cmake/approxmvbb-config-version.cmake"
        DESTINATION ${ApproxMVBB_CMAKE_CONFIG_INSTALL_DIR}
        COMPONENT Devel)

# Install readme and license
install(FILES "${ApproxMVBB_LICENSE_FILE}" "${ApproxMVBB_README_FILE}"
        DESTINATION ${ApproxMVBB_ADDITIONAL_FILES_INSTALL_DIR}
        PERMISSIONS OWNER_READ)
