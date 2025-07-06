project(Scene2GLTF)

build_target(
  NAME
  scene_to_gltf
  TYPE
  ESMODULE
  VERSION
  1
  SOURCES
  scene_to_gltf.cpp
  l1_to_utf8.cpp
  kmp_search.cpp
  LINKS
  spike-interface
  gltf-interface
  AUTHOR
  "Lukas Cone"
  DESCR
  "MercurySteam sm3 to gltf"
  START_YEAR
  2025)
