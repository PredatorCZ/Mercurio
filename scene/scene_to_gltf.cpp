/*  Scene2GLTF
    Copyright(C) 2025 Lukas Cone

    This program is free software : you can redistribute it and / or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.If not, see <https://www.gnu.org/licenses/>.
*/

#include "l1_to_utf8.hpp"
#include "project.h"
#include "spike/app_context.hpp"
#include "spike/except.hpp"
#include "spike/gltf.hpp"
#include "spike/io/bincore_fwd.hpp"
#include "spike/io/binreader_stream.hpp"
#include "spike/master_printer.hpp"
#include "spike/type/matrix44.hpp"
#include "spike/type/vectors.hpp"
#include <cassert>

std::string_view filters[]{
    ".sm3$",
};

static AppInfo_s appInfo{
    .header = Scene2GLTF_DESC " v" Scene2GLTF_VERSION ", " Scene2GLTF_COPYRIGHT
                              "Lukas Cone",
    .filters = filters,
};

AppInfo_s *AppInitModule() { return &appInfo; }

struct Block {
  uint32 id;
  uint32 size;
};

struct Metadata {
  uint32 id;
  uint32 times[4];
};

std::string ReadString(BinReaderRef rd) {
  std::string retVal;
  rd.ReadContainer(retVal);
  while (retVal.size() && retVal.back() == 0) {
    retVal.pop_back();
  }

  std::string buffer(retVal.size() * 3, 0);

  L1U8Recode recode;
  const size_t recSize =
      recode.translate(reinterpret_cast<uint8 *>(retVal.data()), retVal.size(),
                       reinterpret_cast<uint8 *>(buffer.data()));
  buffer.resize(recSize);

  return buffer;
}

bool ReadBool(BinReaderRef rd) {
  uint32 item;
  rd.Read(item);
  return item != 0;
}

struct INI {
  std::vector<std::string> lines;

  void Read(BinReaderRef rd) {
    Block head;
    rd.Read(head);
    uint32 version;
    rd.Read(version);

    if (head.id != CompileFourCC("INI")) {
      throw es::InvalidHeaderError(head.id);
    }

    if (version != 1) {
      throw es::InvalidVersionError(version);
    }

    rd.ReadContainerLambda(lines, [](BinReaderRef rd, std::string &item) {
      item = ReadString(rd);
    });
  }
};

struct BBOX {
  Vector min;
  Vector max;
};

struct Node {
  std::string name;
  std::string parentName;
  int32 parentIndex;
  INI ini;
  int32 index;
  int32 unk1;
  int32 unk2;
  int32 unk3;
  int32 unk4;
  Vector position;
  Vector4 rotation;
  float unk5;
  int32 unk6;
  es::Matrix44 ibm;
  Vector4 rotationUnk;
  int32 meshIndex;
  int32 unk9;
  int32 unk10;

  void Read(BinReaderRef rd) {
    name = ReadString(rd);
    parentName = ReadString(rd);
    rd.Read(parentIndex);
    if (ReadBool(rd)) {
      rd.Read(ini);
    }
    rd.Read(index);
    rd.Read(unk1);
    rd.Read(unk2);
    rd.Read(unk3);
    rd.Read(unk4);
    rd.Read(position);
    rd.Read(rotation);
    rd.Read(unk5);
    rd.Read(unk6);
    rd.Read(ibm);
    rd.Read(rotationUnk);
    rd.Read(meshIndex);
    rd.Read(unk9);
    rd.Read(unk10);
  }
};

struct SM3 {
  std::string rootName;
  INI ini;
  std::string info;
  BBOX bounds0;
  BBOX bounds1;
  Vector origin;
  uint32 unk1;
  bool unk2[5];
  uint32 unk3;
  std::vector<std::string> materialNames;
  std::vector<Node> nodes;

  void Read(BinReaderRef rd) {
    Block head;
    rd.Read(head);
    Metadata metadata;
    rd.Read(metadata);

    if (head.id != CompileFourCC("SM3")) {
      throw es::InvalidHeaderError(head.id);
    }

    rd.Read(head);
    uint32 version;
    rd.Read(version);

    if (head.id != CompileFourCC("SCN")) {
      throw es::InvalidHeaderError(head.id);
    }

    if (version != 5) {
      throw es::InvalidVersionError(version);
    }

    rootName = ReadString(rd);
    rd.Read(version); // unk

    if (ReadBool(rd)) {
      rd.Read(ini);
    }

    info = ReadString(rd);

    rd.Read(bounds0);
    rd.Read(bounds1);
    rd.Read(origin);
    rd.Read(unk1);
    rd.Read(unk2);
    rd.Read(unk3);

    rd.ReadContainerLambda(
        materialNames,
        [](BinReaderRef rd, std::string &item) { item = ReadString(rd); });
    rd.ReadContainer(nodes);
  }
};

size_t LoadFaces(BinReaderRef rd, GLTFModel &main, bool skip) {
  uint32 numFaces;
  uint32 faceStride;
  rd.Read(numFaces);
  rd.Read(faceStride);

  if (faceStride != 6) {
    throw std::runtime_error("Invalid face stride");
  }

  if (skip) {
    rd.Skip(numFaces * faceStride);
    return {};
  }

  std::vector<USVector> faces;
  rd.ReadContainerLambda(faces, numFaces, [](BinReaderRef rd, USVector &item) {
    rd.Read(item.x);
    rd.Read(item.z);
    rd.Read(item.y);
  });

  auto indices = main.SaveIndices(faces.data(), numFaces * 3);
  return indices.accessorIndex;
}

static const Attribute LVERT_POS{
    .type = uni::DataType::R32G32B32,
    .format = uni::FormatType::FLOAT,
    .usage = AttributeType::Position,
};
static const Attribute LVERT_COLOR{
    .type = uni::DataType::R8G8B8A8,
    .format = uni::FormatType::UNORM,
    .usage = AttributeType::VertexColor,
};
static const Attribute LVERT_NORMAL{
    .type = uni::DataType::R32G32B32,
    .format = uni::FormatType::FLOAT,
    .usage = AttributeType::Normal,
};
static const Attribute LVERT_TEX2D{
    .type = uni::DataType::R32G32,
    .format = uni::FormatType::FLOAT,
    .usage = AttributeType::TextureCoordiante,
};
static const Attribute LVERT_TANGENT{
    .type = uni::DataType::R32G32B32A32,
    .format = uni::FormatType::FLOAT,
    .usage = AttributeType::Tangent,
};
static const Attribute LVERT_SNORMAL{
    .type = uni::DataType::R16G16B16A16,
    .format = uni::FormatType::NORM,
    .usage = AttributeType::Normal,
};
static const Attribute LVERT_STANGENT{
    .type = uni::DataType::R16G16B16A16,
    .format = uni::FormatType::NORM,
    .usage = AttributeType::Tangent,
};
static const Attribute LVERT_SKIN_BINDICES{
    .type = uni::DataType::R8G8B8A8,
    .format = uni::FormatType::UINT,
    .usage = AttributeType::BoneIndices,
};
static const Attribute LVERT_SKIN_BWEIGHT{
    .type = uni::DataType::R8G8B8A8,
    .format = uni::FormatType::UNORM,
    .usage = AttributeType::BoneWeights,
};
static const Attribute LVERT_FTEX2D{
    .type = uni::DataType::R16G16,
    .format = uni::FormatType::FLOAT,
    .usage = AttributeType::TextureCoordiante,
};

static const std::vector<Attribute> FORMAT[]{
    {},
    {
        LVERT_POS,
    },
    {
        LVERT_POS,
        // 32b skin, unused
    },
    {
        LVERT_POS,
        LVERT_SKIN_BINDICES,
        LVERT_SKIN_BWEIGHT,
    },
    {
        LVERT_POS,
        LVERT_COLOR,
        LVERT_TEX2D,
    },
    {
        LVERT_POS, LVERT_COLOR, LVERT_TEX2D,
        // 32b skin, unused
    },
    {
        LVERT_POS,
        LVERT_COLOR,
        LVERT_TEX2D,
        LVERT_SKIN_BINDICES,
        LVERT_SKIN_BWEIGHT,
    },
    {
        LVERT_POS,
        LVERT_COLOR,
        LVERT_NORMAL,
        LVERT_TEX2D,
        LVERT_TANGENT,
    },
    {
        LVERT_POS, LVERT_COLOR, LVERT_NORMAL, LVERT_TEX2D, LVERT_TANGENT,
        // 32b skin, unused
    },
    {
        LVERT_POS,
        LVERT_COLOR,
        LVERT_SNORMAL,
        LVERT_TEX2D,
        LVERT_STANGENT,
    },
    {
        LVERT_POS,
        LVERT_COLOR,
        LVERT_SNORMAL,
        LVERT_TEX2D,
        LVERT_STANGENT,
        LVERT_SKIN_BINDICES,
        LVERT_SKIN_BWEIGHT,
    },
    {
        LVERT_POS,
        LVERT_COLOR,
        LVERT_SNORMAL,
        LVERT_FTEX2D,
        LVERT_STANGENT,
    },
    {
        LVERT_POS,
        LVERT_COLOR,
        LVERT_SNORMAL,
        LVERT_FTEX2D,
        LVERT_STANGENT,
        LVERT_SKIN_BINDICES,
        LVERT_SKIN_BWEIGHT,
    },
    {
        LVERT_POS,
        LVERT_COLOR,
        LVERT_NORMAL,
        LVERT_TEX2D,
        LVERT_TANGENT,
        LVERT_TEX2D,
    },
    {
        LVERT_POS,
        LVERT_COLOR,
        LVERT_SNORMAL,
        LVERT_TEX2D,
        LVERT_STANGENT,
        LVERT_TEX2D,
    },
    {
        LVERT_POS,
        LVERT_COLOR,
        LVERT_SNORMAL,
        LVERT_FTEX2D,
        LVERT_STANGENT,
        LVERT_FTEX2D,
    },
    {
        LVERT_POS,
        LVERT_COLOR,
        LVERT_TEX2D,
        LVERT_TEX2D,
    },
};

gltf::Attributes LoadVertices(BinReaderRef rd, GLTFModel &main, bool skip) {
  Block head;
  uint32 version;

  rd.Read(head);
  rd.Read(version);

  if (head.id != CompileFourCC("LFVF")) {
    throw es::InvalidHeaderError(head.id);
  }

  if (version != 3) {
    throw es::InvalidVersionError(version);
  }

  uint32 format;
  rd.Read(format);
  std::string formatText = ReadString(rd);
  rd.Skip(4);
  uint32 vertexStride;
  uint32 numVertices;
  rd.Read(vertexStride);
  rd.Read(numVertices);

  if (skip) {
    rd.Skip(numVertices * vertexStride);
    return {};
  }

  std::string vertexData;
  rd.ReadContainer(vertexData, vertexStride * numVertices);

  return main.SaveVertices(vertexData.data(), numVertices, FORMAT[format],
                           vertexStride);
}

struct GLTFContext {
  GLTFModel render;
  GLTFModel shadow;
  GLTFModel opt;
  GLTFModel optShadow;
  std::vector<uint32> meshToNode;
  std::map<uint16, uint16> joints;
  std::vector<uint8> lastGatheredJoints;
};

enum RENDER_PASS {
  RENDER_PASS_SHADOWS = 1,
  RENDER_PASS_SHADOWS_OPT = 2,
};

uint32 LoadPrimitive(BinReaderRef rd, GLTFModel *mainPtr, uint32 li, uint32 mi,
                     GLTFContext &gctx, uint32 renderPass, bool skip) {
  Block head;
  uint32 version;
  rd.Read(head);
  rd.Read(version);

  if (head.id != CompileFourCC("mesh")) {
    throw es::InvalidHeaderError(head.id);
  }

  if (version != 5) {
    throw es::InvalidVersionError(version);
  }

  ReadString(rd);

  if (!skip) {
    GLTFModel &glMain = renderPass == 2 ? gctx.opt : gctx.render;
    if (glMain.nodes.empty()) {
      glMain.nodes = mainPtr->nodes;
      glMain.materials = mainPtr->materials;
      glMain.scenes = mainPtr->scenes;
      glMain.transform = mainPtr->transform;
      glMain.QuantizeMesh(false);
    }
    glMain.boneRemaps = gctx.lastGatheredJoints;
    mainPtr = &glMain;
  }

  GLTFModel &main = *mainPtr;
  GLTFModel *mainShadow = mainPtr;

  const size_t meshIndex = main.meshes.size();
  int32_t shadowMeshIndex = -1;

  {
    auto &mesh = main.meshes.emplace_back();
    auto &prim = mesh.primitives.emplace_back();
    prim.indices = LoadFaces(rd, main, skip);
    prim.attributes = LoadVertices(rd, main, skip);
  }

  if (ReadBool(rd)) {
    if (!skip) {
      GLTFModel &glMain = renderPass == 2 ? gctx.optShadow : gctx.shadow;
      if (glMain.nodes.empty()) {
        glMain.nodes = mainPtr->nodes;
        glMain.scenes = mainPtr->scenes;
        glMain.transform = mainPtr->transform;
        glMain.QuantizeMesh(false);
      }
      glMain.boneRemaps = gctx.lastGatheredJoints;
      mainShadow = &glMain;
    }
    shadowMeshIndex = mainShadow->meshes.size();
    auto &mesh = mainShadow->meshes.emplace_back();
    auto &prim = mesh.primitives.emplace_back();
    prim.indices = LoadFaces(rd, *mainShadow, skip);
    prim.attributes = LoadVertices(rd, *mainShadow, skip);
  }

  ReadBool(rd);
  uint32 numJoints;
  uint32 jointIndexStride;
  rd.Read(numJoints);
  rd.Read(jointIndexStride);

  if (jointIndexStride != 2) {
    throw std::runtime_error("Incorrect joint index size");
  }

  if (skip) {
    std::vector<uint16> joints;
    rd.ReadContainer(joints, numJoints);
    gctx.lastGatheredJoints.clear();
    for (uint16 j : joints) {
      if (!gctx.joints.contains(j)) {
        gctx.lastGatheredJoints.emplace_back(gctx.joints.size());
        gctx.joints.emplace(j, gctx.joints.size());
        assert(gctx.joints.size() <= 256);
      } else {
        gctx.lastGatheredJoints.emplace_back(gctx.joints.at(j));
      }
    }

    if (shadowMeshIndex > -1) {
      mainShadow->boneRemaps = main.boneRemaps;
    }
  } else {
    auto LoadSkin = [li, &gctx, mi, numJoints](GLTFModel &main,
                                               uint32 meshIndex) {
      size_t nodeIndex = gctx.meshToNode.at(mi);
      const size_t ogNodeIndex = nodeIndex;
      if (li > 0) {
        main.nodes.at(ogNodeIndex).children.emplace_back(main.nodes.size());
        nodeIndex = main.nodes.size();
        main.nodes.emplace_back().name =
            main.nodes.at(ogNodeIndex).name + "_" + std::to_string(li);
      }

      main.nodes.at(nodeIndex).mesh = meshIndex;
      main.meshes.at(meshIndex).name = main.nodes.at(nodeIndex).name;

      if (numJoints > 0) {
        main.nodes.at(nodeIndex).skin = main.skins.size();
      }
    };

    rd.Skip(numJoints * jointIndexStride);
    LoadSkin(main, meshIndex);

    if (shadowMeshIndex > -1) {
      LoadSkin(*mainShadow, shadowMeshIndex);
    }
  }

  ReadBool(rd);
  uint32 materialIndex;
  int32 unk10;
  uint32 primitiveIndex;
  BBOX bbox1;
  rd.Read(materialIndex);
  rd.Read(unk10);
  rd.Read(primitiveIndex);
  rd.Read(bbox1);
  rd.Read(renderPass);

  main.meshes.at(meshIndex).primitives.front().material = materialIndex;

  return renderPass;
}

static const float SCALE = 0.01f;

static const es::Matrix44 CORMAT{
    {SCALE, 0, 0, 0},
    {0, SCALE, 0, 0},
    {0, 0, -SCALE, 0},
};

void LoadMeshData(BinReaderRef rd, GLTFModel &main, GLTFContext &gctx) {
  GLTFModel dummy;
  Block head;
  uint32 version;

  rd.Read(head);
  rd.Read(version);

  if (head.id != CompileFourCC("MSH")) {
    throw es::InvalidHeaderError(head.id);
  }

  if (version != 2) {
    throw es::InvalidVersionError(version);
  }

  uint32 numMeshes;
  rd.Read(numMeshes);

  for (uint32 mi = 0; mi < numMeshes; mi++) {
    if (!ReadBool(rd)) {
      continue;
    }

    rd.Read(head);
    rd.Read(version);

    if (head.id != CompileFourCC("MESH")) {
      throw es::InvalidHeaderError(head.id);
    }

    if (version != 1) {
      throw es::InvalidVersionError(version);
    }
    uint8 unk0;
    rd.Read(unk0);
    BBOX bounds;
    rd.Read(bounds);

    uint32 numPrimitives;
    rd.Read(numPrimitives);

    for (uint32 li = 0; li < numPrimitives; li++) {
      rd.Push();
      const uint32 renderPass =
          LoadPrimitive(rd, &dummy, li, mi, gctx, 0, true);
      rd.Pop();
      LoadPrimitive(rd, &main, li, mi, gctx, renderPass, false);
    }
  }
}

void CreateSkin(GLTFModel &main, GLTFContext &gctx, const SM3 &scene) {
  auto &skin = main.skins.emplace_back();
  skin.joints.resize(gctx.joints.size());

  for (auto &[from, to] : gctx.joints) {
    skin.joints.at(to) = from;
  }

  auto &stream = main.SkinStream();
  auto [acc, accId] = main.NewAccessor(stream, 16);
  skin.inverseBindMatrices = accId;
  acc.count = skin.joints.size();
  acc.componentType = gltf::Accessor::ComponentType::Float;
  acc.type = gltf::Accessor::Type::Mat4;

  for (uint32 j : skin.joints) {
    es::Matrix44 ibm = scene.nodes.at(j).ibm;
    stream.wr.Write(ibm);
  }
}

void AppProcessFile(AppContext *ctx) {
  BinReaderRef rd(ctx->GetStream());
  SM3 scene;
  rd.Read(scene);

  GLTFModel main;
  main.QuantizeMesh(false);
  main.transform = CORMAT;
  GLTFContext gctx;

  std::vector<es::Matrix44> globalTms;
  std::vector<es::Matrix44> globalTmsOld;

  for (auto &n : scene.nodes) {
    auto &node = main.nodes.emplace_back();
    node.name = n.name;
    Vector4A16 rotation(n.rotation * Vector4{1, 1, -1, 1});
    Vector position = n.position * Vector{SCALE, SCALE, -SCALE};
    es::Matrix44 localTm;
    es::Matrix44 localTmOld;

    localTm.Compose(position, rotation, {1, 1, 1, 0});
    localTmOld.Compose(n.position,
                       Vector4A16(n.rotation) * Vector4A16{-1, -1, -1, 1},
                       {1, 1, 1, 0});

    if (n.parentIndex > -1) {
      main.nodes.at(n.parentIndex).children.emplace_back(main.nodes.size() - 1);
      globalTms.emplace_back() = globalTms.at(n.parentIndex) * localTm;
      globalTmsOld.emplace_back() = globalTmsOld.at(n.parentIndex) * localTmOld;
    } else {
      main.scenes.front().nodes.emplace_back(main.nodes.size() - 1);
      globalTms.emplace_back() = localTm;
      globalTmsOld.emplace_back() = localTmOld;
      position += scene.origin * Vector{SCALE, SCALE, -SCALE};
    }

    memcpy(node.translation.data(), &position, 12);
    memcpy(node.rotation.data(), &rotation, 16);

    n.ibm = -globalTmsOld.back() * -n.ibm;
    n.ibm = -(globalTms.back() * n.ibm);

    if (n.meshIndex > -1) {
      if (size_t(n.meshIndex) >= gctx.meshToNode.size()) {
        gctx.meshToNode.resize(n.meshIndex + 1);
      }
      gctx.meshToNode.at(n.meshIndex) = main.nodes.size() - 1;
    }
  }

  for (auto &n : scene.materialNames) {
    main.materials.emplace_back().name = n;
  }

  AppContextStream meshStream;
  try {
    meshStream = ctx->RequestFile(ctx->workingFile.ChangeExtension2("msh"));
  } catch (const es::FileNotFoundError &) {
    if (gctx.meshToNode.size() > 0) {
      PrintWarning("Mesh file not found, but scene contains mesh links");
    }
  }

  if (meshStream) {
    LoadMeshData(*meshStream.Get(), main, gctx);
  }

  BinWritterRef wr(ctx->NewFile(ctx->workingFile.ChangeExtension2("glb")).str);

  CreateSkin(gctx.render, gctx, scene);
  gctx.render.FinishAndSave(wr, std::string(ctx->workingFile.GetFolder()));

  if (gctx.shadow.meshes.size() > 0) {
    BinWritterRef wr(
        ctx->NewFile(ctx->workingFile.ChangeExtension("_shadow.glb")).str);

    CreateSkin(gctx.shadow, gctx, scene);
    gctx.shadow.FinishAndSave(wr, std::string(ctx->workingFile.GetFolder()));
  }

  if (gctx.opt.meshes.size() > 0) {
    BinWritterRef wr(
        ctx->NewFile(ctx->workingFile.ChangeExtension("_opt.glb")).str);

    CreateSkin(gctx.opt, gctx, scene);
    gctx.opt.FinishAndSave(wr, std::string(ctx->workingFile.GetFolder()));
  }

  if (gctx.optShadow.meshes.size() > 0) {
    BinWritterRef wr(
        ctx->NewFile(ctx->workingFile.ChangeExtension("_opt_shadow.glb")).str);

    CreateSkin(gctx.optShadow, gctx, scene);
    gctx.optShadow.FinishAndSave(wr, std::string(ctx->workingFile.GetFolder()));
  }
}
