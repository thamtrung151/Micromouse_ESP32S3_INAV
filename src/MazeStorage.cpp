#include "MazeStorage.h"

#include <Preferences.h>

namespace {

constexpr const char* kNs = "maze";
constexpr const char* kKey = "blob";
constexpr uint32_t kMagic = 0x4D415A45u;  // "MAZE"
constexpr uint16_t kVersion = 1u;

struct __attribute__((packed)) MazeBlob {
  uint32_t magic;
  uint16_t version;
  uint8_t size;
  uint8_t reserved;
  uint8_t walls[Maze::SIZE][Maze::SIZE];
  uint8_t known[Maze::SIZE][Maze::SIZE];
  uint8_t visited[Maze::VISITED_BYTES];
  uint32_t crc32;
};

uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t len) {
  crc = ~crc;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      const uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

uint32_t mazeBlobCrc(const MazeBlob& blob) {
  uint32_t crc = 0u;
  crc = crc32Update(crc, reinterpret_cast<const uint8_t*>(&blob.magic), sizeof(blob.magic));
  crc = crc32Update(crc, reinterpret_cast<const uint8_t*>(&blob.version), sizeof(blob.version));
  crc = crc32Update(crc, reinterpret_cast<const uint8_t*>(&blob.size), sizeof(blob.size));
  crc = crc32Update(crc, reinterpret_cast<const uint8_t*>(&blob.reserved), sizeof(blob.reserved));
  crc = crc32Update(crc, reinterpret_cast<const uint8_t*>(blob.walls), sizeof(blob.walls));
  crc = crc32Update(crc, reinterpret_cast<const uint8_t*>(blob.known), sizeof(blob.known));
  crc = crc32Update(crc, reinterpret_cast<const uint8_t*>(blob.visited), sizeof(blob.visited));
  return crc;
}

bool validateBlob(const MazeBlob& blob) {
  if (blob.magic != kMagic) return false;
  if (blob.version != kVersion) return false;
  if (blob.size != Maze::SIZE) return false;
  if (blob.reserved != 0u) return false;
  return mazeBlobCrc(blob) == blob.crc32;
}

}  // namespace

bool MazeStorage::load(Maze& maze) {
  Preferences prefs;
  if (!prefs.begin(kNs, true)) return false;

  const size_t len = prefs.getBytesLength(kKey);
  if (len != sizeof(MazeBlob)) {
    prefs.end();
    return false;
  }

  MazeBlob blob = {};
  const size_t readLen = prefs.getBytes(kKey, &blob, sizeof(blob));
  prefs.end();

  if (readLen != sizeof(blob)) return false;
  if (!validateBlob(blob)) {
    clear();
    return false;
  }
  if (!maze.importState(blob.walls, blob.known, blob.visited)) {
    clear();
    return false;
  }
  return true;
}

bool MazeStorage::save(const Maze& maze) {
  MazeBlob blob = {};
  blob.magic = kMagic;
  blob.version = kVersion;
  blob.size = Maze::SIZE;
  blob.reserved = 0u;
  maze.exportState(blob.walls, blob.known, blob.visited);
  blob.crc32 = mazeBlobCrc(blob);

  Preferences prefs;
  if (!prefs.begin(kNs, false)) return false;
  const size_t written = prefs.putBytes(kKey, &blob, sizeof(blob));
  prefs.end();
  return written == sizeof(blob);
}

bool MazeStorage::clear() {
  Preferences prefs;
  if (!prefs.begin(kNs, false)) return false;
  if (prefs.getBytesLength(kKey) == 0u) {
    prefs.end();
    return true;
  }
  const bool ok = prefs.remove(kKey);
  prefs.end();
  return ok;
}
