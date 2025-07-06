file(GLOB ZLIB_SOURCES "${TPD_PATH}/zlib/*.c")
add_library(zlib-objects OBJECT ${ZLIB_SOURCES})
set_target_properties(zlib-objects PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

project(PackedExtract)

build_target(
  NAME
  packed_extract
  TYPE
  ESMODULE
  VERSION
  1
  SOURCES
  packed_extract.cpp
  LINKS
  zlib-objects
  spike-interface
  INCLUDES
  ${TPD_PATH}/zlib
  AUTHOR
  "Lukas Cone"
  DESCR
  "MercurySteam Archive Extractor"
  START_YEAR
  2025)
