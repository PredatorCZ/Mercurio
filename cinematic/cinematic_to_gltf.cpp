/*  Cinematic2GLTF
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

#include "project.h"
#include "spike/app_context.hpp"
#include "spike/except.hpp"
#include "spike/gltf.hpp"
#include "spike/io/binreader_stream.hpp"
#include "spike/type/vectors.hpp"
#include <algorithm>

std::string_view filters[]{
    ".cm3$",
};

std::string_view controlFilters[]{
    ".glb$",
    ".gltf$",
};

static AppInfo_s appInfo{
    .header = Cinematic2GLTF_DESC " v" Cinematic2GLTF_VERSION
                                  ", " Cinematic2GLTF_COPYRIGHT "Lukas Cone",
    .filters = filters,
    .batchControlFilters = controlFilters,
};

AppInfo_s *AppInitModule() { return &appInfo; }

struct GLTFAni : GLTF {
  using GLTF::GLTF;

  GLTFStream &AnimStream() {
    if (aniStream < 0) {
      auto &newStream = NewStream("anims");
      aniStream = newStream.slot;
      return newStream;
    }
    return Stream(aniStream);
  }

private:
  int32 aniStream = -1;
};

struct Block {
  uint32 id;
  uint32 size;
};

struct Metadata {
  uint32 id;
  uint32 times[4];
};

bool ReadBool(BinReaderRef rd) {
  uint32 item;
  rd.Read(item);
  return item != 0;
}

struct RotationFrames {
  uint16 blockSize;
  uint16 compressionType;
  uint16 startFrame;
  uint16 numFrames;
  union {
    struct {
      Vector4 frames[1];
    } type0;
    struct {
      Vector offset;
      Vector scale;
      union {
        SVector frames16[1];
        CVector frames8[1];
        int16 uniformFrames[1];
      };
    } typeN;
  };
};

struct PositionFrames {
  uint16 blockSize;
  uint16 compressionType;
  uint16 startFrame;
  uint16 numFrames;
  union {
    struct {
      Vector frames[1];
    } type0;
    struct {
      Vector offset;
      float scale;
      union {
        SVector frames16[1];
        CVector frames8[1];
      };
    } typeN;
  };
};

using GetFrame = Vector4 (*)(const char *data, uint32 ith);

#if defined(_MSC_VER)
void sincosf(float x, float *sinx, float *cosx) {
  *sinx = sinf(x);
  *cosx = cosf(x);
}
#endif

Vector4 EulerToQuat(const Vector &euler) {
  Vector cosr;
  Vector sinr;
  sincosf(euler.x * 0.5f, &sinr.x, &cosr.x);
  sincosf(euler.y * 0.5f, &sinr.y, &cosr.y);
  sincosf(euler.z * 0.5f, &sinr.z, &cosr.z);
  Vector4 retVal;

  retVal.x = -1.0 * sinr.y * sinr.z * cosr.x + sinr.x * cosr.z * cosr.y;
  retVal.y = sinr.y * cosr.z * cosr.x + cosr.y * sinr.x * sinr.z;
  retVal.z = cosr.y * sinr.z * cosr.x + sinr.y * sinr.x * cosr.z;
  retVal.w = cosr.y * cosr.x * cosr.z + sinr.y * sinr.x * sinr.z * -1.0;

  return retVal;
}

static const GetFrame POS_CODECS[]{
    [](const char *data, uint32 ith) -> Vector4 {
      const PositionFrames *frames =
          reinterpret_cast<const PositionFrames *>(data);
      return frames->type0.frames[ith];
    },
    [](const char *data, uint32 ith) -> Vector4 {
      const PositionFrames *frames =
          reinterpret_cast<const PositionFrames *>(data);

      return frames->typeN.offset +
             frames->typeN.frames16[ith].Convert<float>() * frames->typeN.scale;
    },
    {},
    [](const char *data, uint32 ith) -> Vector4 {
      const PositionFrames *frames =
          reinterpret_cast<const PositionFrames *>(data);

      return frames->typeN.offset +
             frames->typeN.frames8[ith].Convert<float>() * frames->typeN.scale;
    },
    {},
    [](const char *data, uint32 ith) -> Vector4 {
      const PositionFrames *frames =
          reinterpret_cast<const PositionFrames *>(data);
      const uint32 startFrame = ith / 4;
      const float delta = (ith % 4) * 0.25;

      Vector frame0 = frames->typeN.offset +
                      frames->typeN.frames16[startFrame].Convert<float>() *
                          frames->typeN.scale;
      Vector frame1 = frames->typeN.offset +
                      frames->typeN.frames16[startFrame + 1].Convert<float>() *
                          frames->typeN.scale;

      return frame0 * (1 - delta) + frame1 * delta;
    },
    [](const char *data, uint32 ith) -> Vector4 {
      return reinterpret_cast<const Vector *>(data)[ith];
    },
};

// https://en.wikipedia.org/wiki/Slerp
static Vector4A16 slerp(const Vector4A16 &v0, const Vector4A16 &_v1, float t) {
  Vector4A16 v1 = _v1;
  float dot = v0.Dot(v1);

  // If the dot product is negative, slerp won't take
  // the shorter path. Fix by reversing one quaternion.
  if (dot < 0.0f) {
    v1 *= -1;
    dot *= -1;
  }

  static const float DOT_THRESHOLD = 0.9995f;
  if (dot > DOT_THRESHOLD) {
    // If the inputs are too close for comfort, linearly interpolate
    // and normalize the result.

    Vector4A16 result = v0 + (v1 - v0) * t;
    return result.Normalize();
  }

  const float theta00 = acos(dot);   // theta00 = angle between input vectors
  const float theta01 = theta00 * t; // theta01 = angle between v0 and result
  const float theta02 = sin(theta01);
  const float theta03 = 1.0f / sin(theta00);
  const float s0 = cos(theta01) - dot * theta02 * theta03;
  const float s1 = theta02 * theta03;

  return (v0 * s0) + (v1 * s1);
}

static const GetFrame ROTATION_CODECS[]{
    [](const char *data, uint32 ith) -> Vector4 {
      const RotationFrames *frames =
          reinterpret_cast<const RotationFrames *>(data);
      return frames->type0.frames[ith];
    },
    [](const char *data, uint32 ith) -> Vector4 {
      const RotationFrames *frames =
          reinterpret_cast<const RotationFrames *>(data);
      static const float decomp = 1.f / 0x7fff;

      Vector euler =
          frames->typeN.offset + frames->typeN.frames16[ith].Convert<float>() *
                                     decomp * frames->typeN.scale;

      return EulerToQuat(euler);
    },
    {},
    [](const char *data, uint32 ith) -> Vector4 {
      const RotationFrames *frames =
          reinterpret_cast<const RotationFrames *>(data);
      static const float decomp = 1.f / 0x7f;

      Vector euler =
          frames->typeN.offset + frames->typeN.frames8[ith].Convert<float>() *
                                     decomp * frames->typeN.scale;
      return EulerToQuat(euler);
    },
    [](const char *data, uint32 ith) -> Vector4 {
      const RotationFrames *frames =
          reinterpret_cast<const RotationFrames *>(data);
      static const float decomp = 1.f / 0x7fff;

      Vector euler =
          frames->typeN.offset + Vector(frames->typeN.uniformFrames[ith]) *
                                     decomp * frames->typeN.scale;
      return EulerToQuat(euler);
    },
    [](const char *data, uint32 ith) -> Vector4 {
      const RotationFrames *frames =
          reinterpret_cast<const RotationFrames *>(data);
      static const float decomp = 1.f / 0x7fff;
      const uint32 startFrame = ith / 4;
      const float delta = (ith % 4) * 0.25;

      Vector frame0 = frames->typeN.offset +
                      frames->typeN.frames16[startFrame].Convert<float>() *
                          decomp * frames->typeN.scale;
      Vector frame1 = frames->typeN.offset +
                      frames->typeN.frames16[startFrame + 1].Convert<float>() *
                          decomp * frames->typeN.scale;
      Vector4 out0 = EulerToQuat(frame0);
      Vector4 out1 = EulerToQuat(frame1);
      Vector4A16 out(slerp(out0, out1, delta));
      return reinterpret_cast<Vector4 &>(out);
    },
    [](const char *data, uint32 ith) -> Vector4 {
      return reinterpret_cast<const Vector4 *>(data)[ith];
    },
};

#include <cassert>
struct TrackHeader {
  struct Descr {
    std::string data;
    uint32 dataSize;
    uint16 startFrame;
    uint16 numFrames;
    GetFrame codec;
  };
  uint32 firstFrame;
  uint32 numFrames;
  uint32 flags;
  uint32 optFlags;
  uint32 stmFlags;

  Descr translation;
  Descr rotation;
  Descr unk0;
  Descr unk1;

  void Read(BinReaderRef rd) {
    rd.Read(firstFrame);
    rd.Read(numFrames);
    rd.Read(flags);
    rd.Read(optFlags);
    rd.Read(stmFlags);

    if (flags & 1) {
      rd.Read(translation.dataSize);
      if (!ReadBool(rd)) {
        rd.ReadContainer(translation.data, translation.dataSize);
      }
    }

    if (flags & 2) {
      rd.Read(rotation.dataSize);
      if (!ReadBool(rd)) {
        rd.ReadContainer(rotation.data, rotation.dataSize);
      }
    }

    if (flags & 4) {
      rd.Read(unk0.dataSize);
      if (!ReadBool(rd)) {
        rd.ReadContainer(unk0.data, unk0.dataSize);
      }
    }

    if (flags & 0x80) {
      rd.Read(unk1.dataSize);
      if (!ReadBool(rd)) {
        rd.ReadContainer(unk1.data, unk1.dataSize);
      }
      assert((stmFlags & 0x80) == 0);
      assert((optFlags & 0x80) == 0);
    }

    assert((flags & ~0x87) == 0);
  }
};

struct CM3 {
  float frameRate;
  uint32 unk5;
  uint32 numFrames;
  uint32 numTracks;
  bool unk51;
  Vector origin;
  std::vector<uint16> boneToTrack;
  std::vector<TrackHeader> tracks;

  void Read(BinReaderRef rd) {
    Block head;
    rd.Read(head);
    Metadata metadata;
    rd.Read(metadata);

    if (head.id != CompileFourCC("CM3")) {
      throw es::InvalidHeaderError(head.id);
    }

    rd.Read(head);
    uint32 version;
    rd.Read(version);

    if (head.id != CompileFourCC("ANIM")) {
      throw es::InvalidHeaderError(head.id);
    }

    if (version != 7) {
      throw es::InvalidVersionError(version);
    }

    rd.Read(frameRate);
    rd.Read(unk5);
    rd.Read(numFrames);
    rd.Read(numTracks);
    unk51 = ReadBool(rd);
    rd.Read(origin);
    rd.ReadContainer(boneToTrack);

    rd.Read(head);

    if (head.id != CompileFourCC("ABLK")) {
      throw es::InvalidHeaderError(head.id);
    }

    uint32 blockDataSize;
    rd.Read(blockDataSize);
    rd.Push();
    rd.Skip(blockDataSize);
    rd.ReadContainer(tracks, numTracks);
    rd.Pop();

    for (auto &t : tracks) {
      if (t.flags & 1) {
        if (t.translation.data.empty()) {
          rd.ReadContainer(t.translation.data, t.translation.dataSize);
          rd.Skip(GetPadding(t.translation.dataSize, 4));
        }

        if (t.stmFlags & 1) {
          PositionFrames *frame =
              reinterpret_cast<PositionFrames *>(t.translation.data.data());
          assert(frame->compressionType < 6);
          t.translation.codec = POS_CODECS[frame->compressionType];
          assert(t.translation.codec);
          t.translation.startFrame = frame->startFrame;
          t.translation.numFrames = frame->numFrames;
        } else {
          t.translation.codec = POS_CODECS[6];
          t.translation.startFrame = 0;
          t.translation.numFrames = 1;
        }
      }
      if (t.flags & 2) {
        if (t.rotation.data.empty()) {
          rd.ReadContainer(t.rotation.data, t.rotation.dataSize);
          rd.Skip(GetPadding(t.rotation.dataSize, 4));
        }

        if (t.stmFlags & 2) {
          PositionFrames *frame =
              reinterpret_cast<PositionFrames *>(t.rotation.data.data());
          assert(frame->compressionType < 6);
          t.rotation.codec = ROTATION_CODECS[frame->compressionType];
          assert(t.rotation.codec);
          t.rotation.startFrame = frame->startFrame;
          t.rotation.numFrames = frame->numFrames;
        } else {
          t.rotation.codec = ROTATION_CODECS[6];
          t.rotation.startFrame = 0;
          t.rotation.numFrames = 1;
        }
      }
      if (t.flags & 4) {
        if (t.unk0.data.empty()) {
          rd.ReadContainer(t.unk0.data, t.unk0.dataSize);
          rd.Skip(GetPadding(t.unk0.dataSize, 4));
        }

        if (t.stmFlags & 4) {
          PositionFrames *frame =
              reinterpret_cast<PositionFrames *>(t.unk0.data.data());
          assert(frame->compressionType == 0);
          // t.unk0.codec = ROTATION_CODECS[frame->compressionType];
          // assert(t.unk0.codec);
        } else {
          // t.unk0.codec = ROTATION_CODECS[0];
        }
      }
      if (t.flags & 0x80) {
        if (t.unk1.data.empty()) {
          rd.ReadContainer(t.unk1.data, t.unk1.dataSize);
          rd.Skip(GetPadding(t.unk1.dataSize, 4));
        }
      }
    }
  }
};

static const float SCALE = 0.01f;

void LoadCine(GLTFAni &main, BinReaderRef rd, std::string name) {
  CM3 cinematic;
  cinematic.Read(rd);

  gltf::Animation &gAnim = main.animations.emplace_back();
  gAnim.name = name;

  auto &stream = main.AnimStream();
  uint32 timesBaseAccIndex = 0;
  gltf::Accessor timesBaseAcc;
  uint32 timesStaticAcc = 0;
  std::vector<float> times = gltfutils::MakeSamples(
      cinematic.frameRate, (cinematic.numFrames - 1) / cinematic.frameRate);

  {
    auto [acc, accid] = main.NewAccessor(stream, 4);
    acc.componentType = gltf::Accessor::ComponentType::Float;
    acc.type = gltf::Accessor::Type::Scalar;
    acc.count = cinematic.numFrames;
    acc.min.push_back(0);
    acc.max.push_back(times.back());
    timesBaseAccIndex = accid;
    timesBaseAcc = acc;

    stream.wr.WriteContainer(times);
  }

  auto StaticTimes = [&] {
    if (timesStaticAcc) {
      return timesStaticAcc;
    }

    timesStaticAcc = main.accessors.size();
    auto &acc = main.accessors.emplace_back(timesBaseAcc);
    acc.count = 1;
    acc.min.front() = 0;
    acc.max.front() = 0;
    return timesStaticAcc;
  };

  for (int16 boneIndex = -1; uint16 trackIndex : cinematic.boneToTrack) {
    boneIndex++;

    if (trackIndex == 0) {
      continue;
    }

    TrackHeader &track = cinematic.tracks.at(trackIndex - 1);

    const bool isRootNode =
        std::find(main.scenes.front().nodes.begin(),
                  main.scenes.front().nodes.end(),
                  boneIndex) != main.scenes.front().nodes.end();

    if (track.flags & 1) {
      gltf::Animation::Channel &channel = gAnim.channels.emplace_back();
      channel.target.node = boneIndex;
      channel.target.path = "translation";
      channel.sampler = gAnim.samplers.size();
      auto &sampler = gAnim.samplers.emplace_back();

      if (track.translation.startFrame == 0 &&
          track.translation.numFrames == 1) {
        sampler.input = StaticTimes();
      } else if (track.translation.startFrame == 0 &&
                 track.translation.numFrames == cinematic.numFrames) {
        sampler.input = timesBaseAccIndex;
      } else {
        sampler.input = main.accessors.size();
        auto &acc = main.accessors.emplace_back(timesBaseAcc);
        acc.byteOffset += track.translation.startFrame * 4;
        acc.count = track.translation.numFrames;
        acc.min.front() = times.at(track.translation.startFrame);
        acc.max.front() =
            times.at(track.translation.startFrame + acc.count - 1);
      }

      auto [acc, accid] = main.NewAccessor(stream, 4);
      acc.componentType = gltf::Accessor::ComponentType::Float;
      acc.type = gltf::Accessor::Type::Vec3;
      acc.count = track.translation.numFrames;
      sampler.output = accid;

      for (uint16 f = 0; f < track.translation.numFrames; f++) {
        Vector trn = track.translation.codec(track.translation.data.data(), f);
        trn += cinematic.origin * isRootNode;
        stream.wr.Write(trn * Vector(SCALE, SCALE, -SCALE));
      }
    }

    if (track.flags & 2) {
      gltf::Animation::Channel &channel = gAnim.channels.emplace_back();
      channel.target.node = boneIndex;
      channel.target.path = "rotation";
      channel.sampler = gAnim.samplers.size();
      auto &sampler = gAnim.samplers.emplace_back();

      if (track.rotation.startFrame == 0 && track.rotation.numFrames == 1) {
        sampler.input = StaticTimes();
      } else if (track.rotation.startFrame == 0 &&
                 track.rotation.numFrames == cinematic.numFrames) {
        sampler.input = timesBaseAccIndex;
      } else {
        sampler.input = main.accessors.size();
        auto &acc = main.accessors.emplace_back(timesBaseAcc);
        acc.byteOffset += track.rotation.startFrame * 4;
        acc.count = track.rotation.numFrames;
        acc.min.front() = times.at(track.rotation.startFrame);
        acc.max.front() = times.at(track.rotation.startFrame + acc.count - 1);
      }

      auto [acc, accid] = main.NewAccessor(stream, 2);
      acc.componentType = gltf::Accessor::ComponentType::Short;
      acc.type = gltf::Accessor::Type::Vec4;
      acc.normalized = true;
      acc.count = track.rotation.numFrames;
      sampler.output = accid;

      for (uint16 f = 0; f < track.rotation.numFrames; f++) {
        Vector4A16 value = track.rotation.codec(track.rotation.data.data(), f);
        value.Normalize();
        value *= Vector4A16(0x7fff, 0x7fff, -0x7fff, 0x7fff);
        value = Vector4A16(_mm_round_ps(value._data, _MM_ROUND_NEAREST));
        stream.wr.Write(value.Convert<int16>());
      }
    }
  }
}

void AppProcessFile(AppContext *ctx) {
  GLTFAni main(gltf::LoadFromBinary(ctx->GetStream(), ""));
  auto &anims = ctx->SupplementalFiles();

  for (auto &animFile : anims) {
    auto animStream = ctx->RequestFile(animFile);
    LoadCine(main, *animStream.Get(),
             std::string(AFileInfo(animFile).GetFilename()));
  }

  BinWritterRef wr(
      ctx->NewFile(std::string(ctx->workingFile.GetFullPathNoExt()) +
                   "_out.glb")
          .str);
  main.FinishAndSave(wr, std::string(ctx->workingFile.GetFolder()));
}
