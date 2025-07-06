project(Cinematic2GLTF)

build_target(
  NAME
  cinematic_to_gltf
  TYPE
  ESMODULE
  VERSION
  1
  SOURCES
  cinematic_to_gltf.cpp
  LINKS
  spike-interface
  gltf-interface
  AUTHOR
  "Lukas Cone"
  DESCR
  "MercurySteam cm3 to gltf"
  START_YEAR
  2025)
