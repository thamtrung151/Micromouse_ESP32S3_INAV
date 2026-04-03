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

struct NormalizeStats {
  uint16_t boundaryFixes = 0;
  uint16_t knownMismatches = 0;
  uint16_t wallMismatches = 0;
  uint16_t unknownWallBitsCleared = 0;
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

inline bool edgeBit(const uint8_t grid[Maze::SIZE][Maze::SIZE], int x, int y, uint8_t dir) {
  return (grid[x][y] & (uint8_t)(1u << dir)) != 0;
}

void setEdgeBits(uint8_t walls[Maze::SIZE][Maze::SIZE],
                 uint8_t known[Maze::SIZE][Maze::SIZE],
                 int x, int y, uint8_t dir, bool edgeKnown, bool wall) {
  const uint8_t mask = (uint8_t)(1u << dir);
  if (edgeKnown) known[x][y] |= mask;
  else           known[x][y] &= (uint8_t)~mask;

  if (wall) walls[x][y] |= mask;
  else      walls[x][y] &= (uint8_t)~mask;
}

NormalizeStats normalizeBlob(MazeBlob& blob) {
  NormalizeStats stats;
  uint8_t walls[Maze::SIZE][Maze::SIZE] = {};
  uint8_t known[Maze::SIZE][Maze::SIZE] = {};

  for (int x = 0; x < (int)Maze::SIZE; x++) {
    for (int y = 0; y < (int)Maze::SIZE; y++) {
      for (uint8_t dir = 0; dir < 4; dir++) {
        int nx, ny;
        Maze::step(x, y, dir, nx, ny);
        const bool rawKnown = edgeBit(blob.known, x, y, dir);
        const bool rawWall = edgeBit(blob.walls, x, y, dir);

        if (!Maze().inBounds(nx, ny)) {
          if (!rawKnown || !rawWall) stats.boundaryFixes++;
          setEdgeBits(walls, known, x, y, dir, true, true);
          continue;
        }

        if ((dir == Maze::S) || (dir == Maze::W)) continue;

        const uint8_t od = Maze::opposite(dir);
        const bool neighborKnown = edgeBit(blob.known, nx, ny, od);
        const bool neighborWall = edgeBit(blob.walls, nx, ny, od);

        if (!rawKnown && !neighborKnown) {
          if (rawWall || neighborWall) stats.unknownWallBitsCleared++;
          continue;
        }

        if (rawKnown != neighborKnown) stats.knownMismatches++;
        if (rawKnown && neighborKnown && rawWall != neighborWall) stats.wallMismatches++;

        bool wall = false;
        if (rawKnown && neighborKnown) {
          wall = rawWall || neighborWall;
        } else if (rawKnown) {
          wall = rawWall;
        } else {
          wall = neighborWall;
        }

        setEdgeBits(walls, known, x, y, dir, true, wall);
        setEdgeBits(walls, known, nx, ny, od, true, wall);
      }
    }
  }

  memcpy(blob.walls, walls, sizeof(blob.walls));
  memcpy(blob.known, known, sizeof(blob.known));
  return stats;
}

bool tryImportNormalizedBlob(const MazeBlob& blob, Maze& maze) {
  MazeBlob normalized = blob;
  normalizeBlob(normalized);
  return maze.importState(normalized.walls, normalized.known, normalized.visited);
}

bool hasRepairs(const NormalizeStats& stats) {
  return stats.boundaryFixes != 0 ||
         stats.knownMismatches != 0 ||
         stats.wallMismatches != 0 ||
         stats.unknownWallBitsCleared != 0;
}

}  // namespace

bool MazeStorage::load(Maze& maze) {
  Preferences prefs;
  if (!prefs.begin(kNs, true)) {
    Serial.println("MazeStorage: load open failed.");
    return false;
  }

  if (!prefs.isKey(kKey)) {
    prefs.end();
    Serial.println("MazeStorage: no saved blob.");
    return false;
  }

  const size_t len = prefs.getBytesLength(kKey);
  if (len != sizeof(MazeBlob)) {
    prefs.end();
    Serial.printf("MazeStorage: blob size %u != %u.\n",
                  (unsigned)len, (unsigned)sizeof(MazeBlob));
    return false;
  }

  MazeBlob blob = {};
  const size_t readLen = prefs.getBytes(kKey, &blob, sizeof(blob));
  prefs.end();

  if (readLen != sizeof(blob)) {
    Serial.printf("MazeStorage: blob read %u != %u.\n",
                  (unsigned)readLen, (unsigned)sizeof(blob));
    return false;
  }

  if (!validateBlob(blob)) {
    Serial.println("MazeStorage: blob CRC/header invalid.");
    clear();
    return false;
  }

  if (!maze.importState(blob.walls, blob.known, blob.visited)) {
    Maze repaired;
    repaired.begin();
    if (!tryImportNormalizedBlob(blob, repaired)) {
      Serial.println("MazeStorage: blob import inconsistent.");
      clear();
      return false;
    }

    maze = repaired;
    Serial.println("MazeStorage: blob repaired during load.");
    return true;
  }

  Serial.println("MazeStorage: load ok.");
  return true;
}

bool MazeStorage::save(const Maze& maze) {
  MazeBlob blob = {};
  blob.magic = kMagic;
  blob.version = kVersion;
  blob.size = Maze::SIZE;
  blob.reserved = 0u;
  maze.exportState(blob.walls, blob.known, blob.visited);

  const bool sourceConsistent = maze.isConsistent();
  const NormalizeStats stats = normalizeBlob(blob);
  if (!sourceConsistent || hasRepairs(stats)) {
    Serial.printf("MazeStorage: normalized map before save (boundary=%u known=%u wall=%u unknown=%u).\n",
                  (unsigned)stats.boundaryFixes,
                  (unsigned)stats.knownMismatches,
                  (unsigned)stats.wallMismatches,
                  (unsigned)stats.unknownWallBitsCleared);
  }

  Maze verify;
  verify.begin();
  if (!verify.importState(blob.walls, blob.known, blob.visited)) {
    Serial.println("MazeStorage: normalized blob still inconsistent.");
    return false;
  }

  blob.crc32 = mazeBlobCrc(blob);

  Preferences prefs;
  if (!prefs.begin(kNs, false)) {
    Serial.println("MazeStorage: save open failed.");
    return false;
  }
  const size_t written = prefs.putBytes(kKey, &blob, sizeof(blob));
  prefs.end();

  if (written != sizeof(blob)) {
    Serial.printf("MazeStorage: save wrote %u != %u.\n",
                  (unsigned)written, (unsigned)sizeof(blob));
    return false;
  }

  Serial.println("MazeStorage: save ok.");
  return true;
}

bool MazeStorage::clear() {
  Preferences prefs;
  if (!prefs.begin(kNs, false)) {
    Serial.println("MazeStorage: clear open failed.");
    return false;
  }
  if (prefs.getBytesLength(kKey) == 0u) {
    prefs.end();
    return true;
  }
  const bool ok = prefs.remove(kKey);
  prefs.end();
  if (ok) Serial.println("MazeStorage: clear ok.");
  return ok;
}
