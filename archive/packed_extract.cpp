/*  PackedExtract
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
#include "spike/io/binreader_stream.hpp"
#include "spike/util/supercore.hpp"
#include "zlib.h"

std::string_view filters[]{
    ".packed$",
};

static AppInfo_s appInfo{
    .header = PackedExtract_DESC " v" PackedExtract_VERSION
                                 ", " PackedExtract_COPYRIGHT "Lukas Cone",
    .filters = filters,
};

AppInfo_s *AppInitModule() { return &appInfo; }

struct FileToc {
  std::string name;
  uint32 uncompressedSize;
  uint64 offset;

  void Read(BinReaderRef rd) {
    rd.ReadContainer(name);
    rd.Read(uncompressedSize);
    rd.Read(offset);
  }
};

void AppProcessFile(AppContext *ctx) {
  BinReaderRef rd(ctx->GetStream());

  uint32 id;
  rd.Read(id);

  if (id != CompileFourCC("BFPK")) {
    throw es::InvalidHeaderError(id);
  }

  uint32 version;
  rd.Read(version);

  if (version != 257) {
    throw es::InvalidVersionError(version);
  }

  std::vector<FileToc> files;
  rd.ReadContainer(files);
  std::string outBuffer;
  std::string inBuffer;
  auto ectx = ctx->ExtractContext();

  for (FileToc &f : files) {
    rd.Seek(f.offset);
    uint32 compressedSize;
    rd.Read(compressedSize);
    outBuffer.resize(f.uncompressedSize);
    uLongf destlen = f.uncompressedSize;
    rd.ReadContainer(inBuffer, compressedSize);

    if (uncompress(reinterpret_cast<Bytef *>(outBuffer.data()), &destlen,
                   reinterpret_cast<const Bytef *>(inBuffer.data()),
                   compressedSize) != Z_OK) {
      throw std::runtime_error("Failed to uncompress data");
    }

    ectx->NewFile(f.name);
    ectx->SendData(outBuffer);
  }
}


size_t AppExtractStat(request_chunk requester) {
  auto buffer = requester(8, 4);
  auto numfiles = reinterpret_cast<const uint32 *>(buffer.data());
  return *numfiles;
}
