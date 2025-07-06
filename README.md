
<h1 align="center">Mercurio</h1>

<p align="center">
    Mercurio is a collection of modding tools for Mercury Engine.
</p>

<p align="center">
    This toolset runs on Spike foundation.
</p>

<p align="center">
    Head to this
    <b><a href="https://github.com/PredatorCZ/Spike/wiki/Spike">Wiki</a></b>
    for more information on how to effectively use it.
</p>

<p align="center">
<b><a href="https://github.com/PredatorCZ/Mercurio/releases">Latest Release</a></b>
</p><h2>Module list</h2>
<ul>
<li><a href="#Cinematic-to-GLTF">Cinematic to GLTF</a></li>
<li><a href="#Extract-PACKED">Extract PACKED</a></li>
<li><a href="#Scene-to-GLTF">Scene to GLTF</a></li>
</ul>

## Cinematic to GLTF

### Module command: cinematic_to_gltf

Convert cinematic .cm3 into GLTF format.

> [!NOTE]
> The following file patterns apply to `batch.json` which is described [HERE](https://github.com/PredatorCZ/Spike/wiki/Spike---Batching)

### Main file patterns: `.glb$`, `.gltf$`

### Secondary file patterns: `.cm3$`

## Extract PACKED

### Module command: packed_extract

Extracts content from .packed archives.

### Input file patterns: `.packed$`

## Scene to GLTF

### Module command: scene_to_gltf

Convert .sm3 scene into GLTF format.

### Input file patterns: `.sm3$`

## License

This toolset is available under GPL v3 license. (See LICENSE.md)\
This toolset uses following libraries:

- Spike, Copyright (c) 2016-2025 Lukas Cone
- zlib, Copyright (C) 1995-2025 Jean-loup Gailly and Mark Adler
- latin1-to-utf8, Copyright: 2015 Eliezio Oliveira
