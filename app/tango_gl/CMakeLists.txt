
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/glm
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libpng/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfreetype/include
    )

# compile libpng
set(png_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libpng/png.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libpng/pngerror.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libpng/pngget.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libpng/pngmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libpng/pngpread.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libpng/pngread.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libpng/pngrio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libpng/pngrtran.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libpng/pngrutil.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libpng/pngset.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libpng/pngtrans.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libpng/pngwio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libpng/pngwrite.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libpng/pngwtran.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libpng/pngwutil.c
    )

add_library(local_png_lib STATIC ${png_sources})

# compile libfreetype
set(freetype_sources
      ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfreetype/src/base/ftbbox.c
      ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfreetype/src/base/ftbitmap.c
      ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfreetype/src/base/ftglyph.c
      ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfreetype/src/base/ftstroke.c
      ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfreetype/src/base/ftxf86.c
      ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfreetype/src/base/ftbase.c
      ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfreetype/src/base/ftsystem.c
      ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfreetype/src/base/ftinit.c
      ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfreetype/src/base/ftgasp.c
      ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfreetype/src/raster/raster.c
      ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfreetype/src/sfnt/sfnt.c
      ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfreetype/src/smooth/smooth.c
      ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfreetype/src/autofit/autofit.c
      ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfreetype/src/truetype/truetype.c
      ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfreetype/src/cff/cff.c
      ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfreetype/src/psnames/psnames.c
      ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfreetype/src/pshinter/pshinter.c
    )

add_library(libfreetype STATIC ${freetype_sources})

# compile tango_gl
set(gl_sources
    src/axis.cc
    src/band.cc
    src/bounding_box.cc
    src/camera.cc
    src/circle.cc
    src/conversions.cc
    src/cube.cc
    src/drawable_object.cc
    src/frustum.cc
    src/gesture_camera.cc
    src/goal_marker.cc
    src/grid.cc
    src/line.cc
    src/mesh.cc
    src/meshes.cc
    src/obj_loader.cc
    src/quad.cc
    src/segment_drawable.cc
    src/shaders.cc
    src/tango_gl.cc
    src/texture.cc
    src/trace.cc
    src/transform.cc
    src/triangle.cc
    src/util.cc
    src/video_overlay.cc )

add_library( tango_gl STATIC ${gl_sources} )

target_link_libraries(
    tango_gl
    android
    log
    GLESv2
    local_png_lib
    )
