// nav_oracle: reference driver over a pinned recastnavigation checkout.
//
// It bakes a case with upstream Recast/Detour/DetourCrowd/DetourTileCache and
// writes canonical records: one JSON line per stage or query family, each
// carrying counts and an FNV-1a 64-bit digest of a canonical little-endian byte
// stream. With --records FILE it writes every stream into one record file (the
// container format in reference/RECORDS.md) that the Rust comparator reads; with
// --dump DIR it also writes each stream to DIR/<record>.bin. The Rust port emits
// the same streams and digests.
//
// Usage:
//   nav_oracle bake  <case-file> [--records FILE] [--dump DIR]
//   nav_oracle bench <case-file> --repeat N [--threads T] [--query-repeat Q]
//   nav_oracle gen   <kind> <out.obj>
//
// Relative mesh paths in case files resolve against $NAV_CORPUS_DIR.
// The case-file format and every record are specified in reference/RECORDS.md.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "DetourCommon.h"
#include "DetourCrowd.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "Recast.h"

#if defined(__GLIBC__)
#include <gnu/libc-version.h>
#endif

#ifndef NAV_UPSTREAM_COMMIT
#define NAV_UPSTREAM_COMMIT "unknown"
#endif
#ifndef NAV_ORACLE_FLAGS
#define NAV_ORACLE_FLAGS "unknown"
#endif

namespace {

// ---------------------------------------------------------------------------
// Canonical byte stream with an FNV-1a 64 digest.

struct Stream {
    uint64_t hash = 14695981039346656037ull; // FNV-1a 64 offset basis (0xcbf29ce484222325)
    uint64_t bytes = 0;
    FILE* out = nullptr;
    std::vector<unsigned char>* buf = nullptr; // set when a record file is being written

    void put(const void* data, size_t n) {
        const unsigned char* p = static_cast<const unsigned char*>(data);
        for (size_t i = 0; i < n; ++i) {
            hash ^= p[i];
            hash *= 1099511628211ull;
        }
        bytes += n;
        if (out) fwrite(p, 1, n, out);
        if (buf) buf->insert(buf->end(), p, p + n);
    }
    // Explicit little-endian encoders so the stream does not depend on host layout.
    void u8(uint32_t v) { unsigned char b = (unsigned char)v; put(&b, 1); }
    void u16(uint32_t v) { unsigned char b[2] = {(unsigned char)v, (unsigned char)(v >> 8)}; put(b, 2); }
    void u32(uint32_t v) {
        unsigned char b[4] = {(unsigned char)v, (unsigned char)(v >> 8), (unsigned char)(v >> 16), (unsigned char)(v >> 24)};
        put(b, 4);
    }
    void i32(int32_t v) { u32((uint32_t)v); }
    void f32(float v) { uint32_t b; memcpy(&b, &v, 4); u32(b); }
    void v3(const float* v) { f32(v[0]); f32(v[1]); f32(v[2]); }
    // Poly and tile refs at their build width (`polyref_bits` in the metadata):
    // u32, or u64 when upstream is built with DT_POLYREF64.
    void ref(dtPolyRef v) {
        u32((uint32_t)v);
        if (sizeof(dtPolyRef) == 8) u32((uint32_t)((uint64_t)v >> 32));
    }
};

const char* g_dumpDir = nullptr;
bool g_quiet = false; // bench mode: compute records without printing them

// Record file (reference/RECORDS.md, "Record file"): magic, metadata, then one
// framed entry per record (name, payload, FNV-1a 64 of the payload), then an
// end marker and the record count. Records are framed when they close, so the
// order is the close order, which is fixed by the driver code.
FILE* g_recordsFile = nullptr;
uint32_t g_recordCount = 0;

void fileU32(FILE* f, uint32_t v) {
    unsigned char b[4] = {(unsigned char)v, (unsigned char)(v >> 8), (unsigned char)(v >> 16), (unsigned char)(v >> 24)};
    fwrite(b, 1, 4, f);
}
void fileU64(FILE* f, uint64_t v) { fileU32(f, (uint32_t)v); fileU32(f, (uint32_t)(v >> 32)); }

bool openRecordsFile(const char* path, const char* caseName) {
    g_recordsFile = fopen(path, "wb");
    if (!g_recordsFile) { fprintf(stderr, "cannot write %s\n", path); return false; }
    static const unsigned char magic[8] = {'N', 'A', 'V', 'R', 'E', 'C', 0, 1};
    fwrite(magic, 1, 8, g_recordsFile);
    const char* libc = "unknown";
#if defined(__GLIBC__)
    libc = gnu_get_libc_version();
#endif
    std::string base(caseName);
    size_t slash = base.find_last_of('/');
    if (slash != std::string::npos) base = base.substr(slash + 1);
    std::string meta = std::string("producer=nav_oracle\nupstream=") + NAV_UPSTREAM_COMMIT + "\ncompiler=" + __VERSION__ +
                       "\nflags=" + NAV_ORACLE_FLAGS + "\nlibc=" + libc + "\ncase=" + base +
                       "\npolyref_bits=" + std::to_string(sizeof(dtPolyRef) * 8) + "\n";
    fileU32(g_recordsFile, (uint32_t)meta.size());
    fwrite(meta.data(), 1, meta.size(), g_recordsFile);
    return true;
}

bool closeRecordsFile() {
    if (!g_recordsFile) return true;
    fileU32(g_recordsFile, 0);
    fileU32(g_recordsFile, g_recordCount);
    const bool ok = fflush(g_recordsFile) == 0 && !ferror(g_recordsFile);
    fclose(g_recordsFile);
    g_recordsFile = nullptr;
    return ok;
}

struct Record {
    std::string stage;
    Stream s;
    std::map<std::string, long long> counts;
    std::vector<unsigned char> payload;

    explicit Record(const std::string& name) : stage(name) {
        if (g_dumpDir) {
            std::string path = std::string(g_dumpDir) + "/" + name + ".bin";
            s.out = fopen(path.c_str(), "wb");
        }
        if (g_recordsFile) s.buf = &payload;
    }
    ~Record() {
        if (s.out) fclose(s.out);
        if (g_recordsFile) {
            fileU32(g_recordsFile, (uint32_t)stage.size());
            fwrite(stage.data(), 1, stage.size(), g_recordsFile);
            fileU64(g_recordsFile, (uint64_t)payload.size());
            if (!payload.empty()) fwrite(payload.data(), 1, payload.size(), g_recordsFile);
            fileU64(g_recordsFile, s.hash);
            ++g_recordCount;
        }
        if (g_quiet) return;
        printf("{\"stage\":\"%s\",\"bytes\":%llu,\"fnv1a64\":\"%016llx\"", stage.c_str(),
               (unsigned long long)s.bytes, (unsigned long long)s.hash);
        for (const auto& kv : counts) printf(",\"%s\":%lld", kv.first.c_str(), kv.second);
        printf("}\n");
    }
};

// ---------------------------------------------------------------------------
// Case file: "key = value" lines, '#' comments. Unknown keys are an error.

// Convex area volume (rcMarkConvexPolyArea): area id, height range, xz outline.
struct AreaVolume {
    int area = 0;
    float hmin = 0, hmax = 0;
    std::vector<float> verts; // x, y, z per point (y unused by Recast)
};

// One tile-cache obstacle script step (`tc_obstacle` lines).
struct ObstacleOp {
    int op = 0; // 1 cylinder, 2 box, 3 oriented box, 4 remove, 5 update
    std::vector<float> args;
};

struct Case {
    std::string mesh;
    std::string mode = "solo";          // solo | tiled | tilecache
    std::string partition = "watershed"; // watershed | monotone | layers
    float cellSize = 0.3f, cellHeight = 0.2f;
    float agentHeight = 2.0f, agentRadius = 0.6f, agentMaxClimb = 0.9f, agentMaxSlope = 45.0f;
    float regionMinSize = 8, regionMergeSize = 20;
    float edgeMaxLen = 12.0f, edgeMaxError = 1.3f;
    int vertsPerPoly = 6;
    float detailSampleDist = 6.0f, detailSampleMaxError = 1.0f;
    int tileSize = 32;
    int filterLowHanging = 1, filterLedge = 1, filterWalkableLowHeight = 1;
    int queries = 64;          // random point pairs for Detour queries
    uint32_t seed = 0x9e3779b9u;
    int crowdAgents = 0;       // 0 disables the crowd record
    int crowdTicks = 0;
    std::vector<float> offMesh; // x0 y0 z0 x1 y1 z1 radius bidir, repeated
    std::vector<AreaVolume> areaVolumes;
    std::vector<ObstacleOp> obstacleOps;
};

// Reads whitespace-separated floats; returns how many were read.
int parseFloats(const char* s, std::vector<float>& out) {
    int n = 0;
    while (*s) {
        char* end = nullptr;
        const float v = strtof(s, &end);
        if (end == s) break;
        out.push_back(v);
        ++n;
        s = end;
    }
    return n;
}

// Mesh paths that are not absolute resolve against $NAV_CORPUS_DIR, where
// reference/corpus/fetch.sh materialises the corpus.
bool resolveMesh(const char* val, std::string& out) {
    if (val[0] == '/') { out = val; return true; }
    const char* dir = getenv("NAV_CORPUS_DIR");
    if (!dir || !*dir) {
        fprintf(stderr, "NAV_CORPUS_DIR is not set: export it to the directory filled by reference/corpus/fetch.sh\n");
        return false;
    }
    out = std::string(dir) + "/" + val;
    return true;
}

bool parseCase(const char* path, Case& c) {
    FILE* f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "cannot open case %s\n", path); return false; }
    char line[1024];
    int lineNo = 0;
    while (fgets(line, sizeof(line), f)) {
        ++lineNo;
        char* hash = strchr(line, '#');
        if (hash) *hash = '\0';
        char* eq = strchr(line, '=');
        if (!eq) continue;
        *eq = '\0';
        char key[128] = {0}, val[896] = {0};
        if (sscanf(line, " %127s", key) != 1) continue;
        char* v = eq + 1;
        while (*v == ' ' || *v == '\t') ++v;
        size_t n = strlen(v);
        while (n && (v[n - 1] == '\n' || v[n - 1] == '\r' || v[n - 1] == ' ')) v[--n] = '\0';
        snprintf(val, sizeof(val), "%s", v);
        std::string k(key);
        if (k == "mesh") {
            if (!resolveMesh(val, c.mesh)) { fclose(f); return false; }
        }
        else if (k == "mode") c.mode = val;
        else if (k == "partition") c.partition = val;
        else if (k == "cell_size") c.cellSize = strtof(val, nullptr);
        else if (k == "cell_height") c.cellHeight = strtof(val, nullptr);
        else if (k == "agent_height") c.agentHeight = strtof(val, nullptr);
        else if (k == "agent_radius") c.agentRadius = strtof(val, nullptr);
        else if (k == "agent_max_climb") c.agentMaxClimb = strtof(val, nullptr);
        else if (k == "agent_max_slope") c.agentMaxSlope = strtof(val, nullptr);
        else if (k == "region_min_size") c.regionMinSize = strtof(val, nullptr);
        else if (k == "region_merge_size") c.regionMergeSize = strtof(val, nullptr);
        else if (k == "edge_max_len") c.edgeMaxLen = strtof(val, nullptr);
        else if (k == "edge_max_error") c.edgeMaxError = strtof(val, nullptr);
        else if (k == "verts_per_poly") c.vertsPerPoly = atoi(val);
        else if (k == "detail_sample_dist") c.detailSampleDist = strtof(val, nullptr);
        else if (k == "detail_sample_max_error") c.detailSampleMaxError = strtof(val, nullptr);
        else if (k == "tile_size") c.tileSize = atoi(val);
        else if (k == "filter_low_hanging") c.filterLowHanging = atoi(val);
        else if (k == "filter_ledge") c.filterLedge = atoi(val);
        else if (k == "filter_walkable_low_height") c.filterWalkableLowHeight = atoi(val);
        else if (k == "queries") c.queries = atoi(val);
        else if (k == "seed") c.seed = (uint32_t)strtoul(val, nullptr, 0);
        else if (k == "crowd_agents") c.crowdAgents = atoi(val);
        else if (k == "crowd_ticks") c.crowdTicks = atoi(val);
        else if (k == "off_mesh") {
            float x[8];
            if (sscanf(val, "%f %f %f %f %f %f %f %f", &x[0], &x[1], &x[2], &x[3], &x[4], &x[5], &x[6], &x[7]) != 8) {
                fprintf(stderr, "%s:%d: off_mesh needs 8 numbers\n", path, lineNo);
                fclose(f);
                return false;
            }
            c.offMesh.insert(c.offMesh.end(), x, x + 8);
        } else if (k == "area_volume") {
            // area hmin hmax x0 z0 x1 z1 x2 z2 ...
            std::vector<float> x;
            const int n = parseFloats(val, x);
            if (n < 9 || (n - 3) % 2 != 0 || x[0] < 1 || x[0] > 62) {
                fprintf(stderr, "%s:%d: area_volume needs an area in 1..62, hmin, hmax and at least 3 x z points\n", path, lineNo);
                fclose(f);
                return false;
            }
            AreaVolume v;
            v.area = (int)x[0]; v.hmin = x[1]; v.hmax = x[2];
            for (int i = 3; i + 1 < n; i += 2) { v.verts.push_back(x[i]); v.verts.push_back(0.0f); v.verts.push_back(x[i + 1]); }
            c.areaVolumes.push_back(v);
        } else if (k == "tc_obstacle") {
            // cylinder x y z radius height | box x0 y0 z0 x1 y1 z1 | obox cx cy cz hx hy hz yaw | remove N | update
            char op[32] = {0};
            int used = 0;
            sscanf(val, "%31s %n", op, &used);
            ObstacleOp o;
            const int n = parseFloats(val + used, o.args);
            const std::string ops(op);
            int want = -1;
            if (ops == "cylinder") { o.op = 1; want = 5; }
            else if (ops == "box") { o.op = 2; want = 6; }
            else if (ops == "obox") { o.op = 3; want = 7; }
            else if (ops == "remove") { o.op = 4; want = 1; }
            else if (ops == "update") { o.op = 5; want = 0; }
            if (want < 0 || n != want) {
                fprintf(stderr, "%s:%d: bad tc_obstacle '%s'\n", path, lineNo, val);
                fclose(f);
                return false;
            }
            c.obstacleOps.push_back(o);
        } else {
            fprintf(stderr, "%s:%d: unknown key '%s'\n", path, lineNo, key);
            fclose(f);
            return false;
        }
    }
    fclose(f);
    if (c.mesh.empty()) { fprintf(stderr, "case has no mesh\n"); return false; }
    return true;
}

// ---------------------------------------------------------------------------
// OBJ loading. Same rules as the upstream demo loader: 'v' rows, 'f' rows fan
// triangulated as (0, i-1, i), negative indices relative, out-of-range faces
// dropped. Only the canonical geometry record is part of the parity surface.

struct Geometry {
    std::vector<float> verts;
    std::vector<int> tris;
    float bmin[3], bmax[3];
};

bool loadObj(const std::string& path, Geometry& g) {
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) { fprintf(stderr, "cannot open mesh %s\n", path.c_str()); return false; }
    char row[512];
    while (fgets(row, sizeof(row), f)) {
        if (row[0] == 'v' && (row[1] == ' ' || row[1] == '\t')) {
            float x, y, z;
            if (sscanf(row + 1, "%f %f %f", &x, &y, &z) == 3) {
                g.verts.push_back(x); g.verts.push_back(y); g.verts.push_back(z);
            }
        } else if (row[0] == 'f' && (row[1] == ' ' || row[1] == '\t')) {
            const int vc = (int)g.verts.size() / 3;
            int face[32];
            int n = 0;
            char* p = row + 1;
            while (*p && n < 32) {
                while (*p == ' ' || *p == '\t') ++p;
                if (!*p || *p == '\n' || *p == '\r') break;
                int idx = atoi(p);
                face[n++] = idx < 0 ? idx + vc : idx - 1;
                while (*p && *p != ' ' && *p != '\t') ++p;
            }
            for (int i = 2; i < n; ++i) {
                int a = face[0], b = face[i - 1], c = face[i];
                if (a < 0 || a >= vc || b < 0 || b >= vc || c < 0 || c >= vc) continue;
                g.tris.push_back(a); g.tris.push_back(b); g.tris.push_back(c);
            }
        }
    }
    fclose(f);
    if (g.verts.empty() || g.tris.empty()) { fprintf(stderr, "empty mesh %s\n", path.c_str()); return false; }
    rcCalcBounds(g.verts.data(), (int)g.verts.size() / 3, g.bmin, g.bmax);
    return true;
}

// ---------------------------------------------------------------------------
// Records for each Recast stage.

void recordGeometry(const Geometry& g) {
    Record r("geometry");
    r.counts["verts"] = (long long)g.verts.size() / 3;
    r.counts["tris"] = (long long)g.tris.size() / 3;
    r.s.u32((uint32_t)g.verts.size() / 3);
    for (float v : g.verts) r.s.f32(v);
    r.s.u32((uint32_t)g.tris.size() / 3);
    for (int t : g.tris) r.s.i32(t);
}

void streamConfig(Stream& s, const rcConfig& c) {
    s.i32(c.width); s.i32(c.height); s.i32(c.tileSize); s.i32(c.borderSize);
    s.f32(c.cs); s.f32(c.ch); s.v3(c.bmin); s.v3(c.bmax);
    s.f32(c.walkableSlopeAngle); s.i32(c.walkableHeight); s.i32(c.walkableClimb); s.i32(c.walkableRadius);
    s.i32(c.maxEdgeLen); s.f32(c.maxSimplificationError); s.i32(c.minRegionArea); s.i32(c.mergeRegionArea);
    s.i32(c.maxVertsPerPoly); s.f32(c.detailSampleDist); s.f32(c.detailSampleMaxError);
}

void streamHeightfield(Stream& s, const rcHeightfield& hf, long long& spans) {
    s.i32(hf.width); s.i32(hf.height); s.v3(hf.bmin); s.v3(hf.bmax); s.f32(hf.cs); s.f32(hf.ch);
    spans = 0;
    for (int i = 0; i < hf.width * hf.height; ++i) {
        uint32_t n = 0;
        for (const rcSpan* sp = hf.spans[i]; sp; sp = sp->next) ++n;
        s.u32(n);
        for (const rcSpan* sp = hf.spans[i]; sp; sp = sp->next) {
            s.u16(sp->smin); s.u16(sp->smax); s.u8(sp->area);
        }
        spans += n;
    }
}

void streamCompact(Stream& s, const rcCompactHeightfield& chf, bool withDist, bool withRegions) {
    s.i32(chf.width); s.i32(chf.height); s.i32(chf.spanCount);
    s.i32(chf.walkableHeight); s.i32(chf.walkableClimb); s.i32(chf.borderSize);
    s.v3(chf.bmin); s.v3(chf.bmax); s.f32(chf.cs); s.f32(chf.ch);
    for (int i = 0; i < chf.width * chf.height; ++i) { s.u32(chf.cells[i].index); s.u8(chf.cells[i].count); }
    for (int i = 0; i < chf.spanCount; ++i) {
        const rcCompactSpan& sp = chf.spans[i];
        s.u16(sp.y); s.u32(sp.con); s.u8(sp.h); s.u8(chf.areas[i]);
        if (withRegions) s.u16(sp.reg);
        if (withDist) s.u16(chf.dist ? chf.dist[i] : 0);
    }
    if (withDist) s.u16(chf.maxDistance);
    if (withRegions) s.u16(chf.maxRegions);
}

void streamContours(Stream& s, const rcContourSet& cset, long long& verts) {
    s.i32(cset.nconts); s.v3(cset.bmin); s.v3(cset.bmax); s.f32(cset.cs); s.f32(cset.ch);
    s.i32(cset.width); s.i32(cset.height); s.i32(cset.borderSize); s.f32(cset.maxError);
    verts = 0;
    for (int i = 0; i < cset.nconts; ++i) {
        const rcContour& c = cset.conts[i];
        s.u16(c.reg); s.u8(c.area);
        s.i32(c.nverts); for (int j = 0; j < c.nverts * 4; ++j) s.i32(c.verts[j]);
        s.i32(c.nrverts); for (int j = 0; j < c.nrverts * 4; ++j) s.i32(c.rverts[j]);
        verts += c.nverts;
    }
}

void streamPolyMesh(Stream& s, const rcPolyMesh& m) {
    s.i32(m.nverts); s.i32(m.npolys); s.i32(m.nvp); s.v3(m.bmin); s.v3(m.bmax);
    s.f32(m.cs); s.f32(m.ch); s.i32(m.borderSize); s.f32(m.maxEdgeError);
    for (int i = 0; i < m.nverts * 3; ++i) s.u16(m.verts[i]);
    for (int i = 0; i < m.npolys; ++i) {
        for (int j = 0; j < m.nvp * 2; ++j) s.u16(m.polys[i * m.nvp * 2 + j]);
        s.u16(m.regs[i]); s.u16(m.flags[i]); s.u8(m.areas[i]);
    }
}

void streamDetail(Stream& s, const rcPolyMeshDetail& d) {
    s.i32(d.nmeshes); s.i32(d.nverts); s.i32(d.ntris);
    for (int i = 0; i < d.nmeshes * 4; ++i) s.u32(d.meshes[i]);
    for (int i = 0; i < d.nverts * 3; ++i) s.f32(d.verts[i]);
    for (int i = 0; i < d.ntris * 4; ++i) s.u8(d.tris[i]);
}

void streamLayers(Stream& s, const rcHeightfieldLayerSet& lset) {
    s.i32(lset.nlayers);
    for (int i = 0; i < lset.nlayers; ++i) {
        const rcHeightfieldLayer& l = lset.layers[i];
        s.v3(l.bmin); s.v3(l.bmax); s.f32(l.cs); s.f32(l.ch);
        s.i32(l.width); s.i32(l.height); s.i32(l.minx); s.i32(l.maxx); s.i32(l.miny); s.i32(l.maxy);
        s.i32(l.hmin); s.i32(l.hmax);
        const int n = l.width * l.height;
        for (int k = 0; k < n; ++k) s.u8(l.heights[k]);
        for (int k = 0; k < n; ++k) s.u8(l.areas[k]);
        for (int k = 0; k < n; ++k) s.u8(l.cons[k]);
    }
}

void streamLayerHeader(Stream& s, const dtTileCacheLayerHeader& h) {
    s.i32(h.magic); s.i32(h.version); s.i32(h.tx); s.i32(h.ty); s.i32(h.tlayer);
    s.v3(h.bmin); s.v3(h.bmax); s.u16(h.hmin); s.u16(h.hmax);
    s.u8(h.width); s.u8(h.height); s.u8(h.minx); s.u8(h.maxx); s.u8(h.miny); s.u8(h.maxy);
}

void streamLayerGrid(Stream& s, const dtTileCacheLayer& l) {
    streamLayerHeader(s, *l.header);
    const int n = (int)l.header->width * (int)l.header->height;
    for (int k = 0; k < n; ++k) s.u8(l.heights[k]);
    for (int k = 0; k < n; ++k) s.u8(l.areas[k]);
    for (int k = 0; k < n; ++k) s.u8(l.cons[k]);
}

void streamLayerRegions(Stream& s, const dtTileCacheLayer& l) {
    const int n = (int)l.header->width * (int)l.header->height;
    s.i32(n); s.u8(l.regCount);
    for (int k = 0; k < n; ++k) s.u8(l.regs[k]);
}

void streamLayerContours(Stream& s, const dtTileCacheContourSet& cs, long long& verts) {
    s.i32(cs.nconts);
    verts = 0;
    for (int i = 0; i < cs.nconts; ++i) {
        const dtTileCacheContour& c = cs.conts[i];
        s.u8(c.reg); s.u8(c.area);
        s.i32(c.nverts);
        for (int k = 0; k < c.nverts * 4; ++k) s.u8(c.verts[k]);
        verts += c.nverts;
    }
}

void streamLayerPolyMesh(Stream& s, const dtTileCachePolyMesh& m) {
    s.i32(m.nvp); s.i32(m.nverts); s.i32(m.npolys);
    for (int i = 0; i < m.nverts * 3; ++i) s.u16(m.verts[i]);
    for (int i = 0; i < m.npolys; ++i) {
        for (int j = 0; j < m.nvp * 2; ++j) s.u16(m.polys[i * m.nvp * 2 + j]);
        s.u16(m.flags[i]); s.u8(m.areas[i]);
    }
}

// Poly flags derived from the area id: area 0 walks (flag 1); area a > 0 also
// carries flag (1 << a) for a < 16. The filtered query set excludes flag 0x10,
// so polygons in area 4 become impassable there.
unsigned short polyFlagsForArea(unsigned char area) {
    if (area == 0 || area >= 16) return 1;
    return (unsigned short)(1u | (1u << area));
}

bool markAreaVolumes(rcContext* ctx, const Case& c, rcCompactHeightfield& chf) {
    for (const AreaVolume& v : c.areaVolumes)
        rcMarkConvexPolyArea(ctx, v.verts.data(), (int)v.verts.size() / 3, v.hmin, v.hmax, (unsigned char)v.area, chf);
    return !c.areaVolumes.empty();
}

// Off-mesh links from the case, in the layout dtNavMeshCreateParams expects.
struct OffMeshArrays {
    std::vector<float> verts, rad;
    std::vector<unsigned char> dir, area;
    std::vector<unsigned short> flags;
    std::vector<unsigned int> id;
    explicit OffMeshArrays(const Case& c) {
        for (size_t i = 0; i + 8 <= c.offMesh.size(); i += 8) {
            verts.insert(verts.end(), &c.offMesh[i], &c.offMesh[i] + 6);
            rad.push_back(c.offMesh[i + 6]);
            dir.push_back(c.offMesh[i + 7] != 0 ? DT_OFFMESH_CON_BIDIR : 0);
            area.push_back(0); flags.push_back(1); id.push_back((unsigned int)(1000 + i / 8));
        }
    }
    void apply(dtNavMeshCreateParams& p) const {
        p.offMeshConVerts = verts.data(); p.offMeshConRad = rad.data(); p.offMeshConDir = dir.data();
        p.offMeshConAreas = area.data(); p.offMeshConFlags = flags.data(); p.offMeshConUserID = id.data();
        p.offMeshConCount = (int)rad.size();
    }
};

// ---------------------------------------------------------------------------
// One Recast build over a bounding box (whole mesh for solo, one tile for tiled).

struct BuildOut {
    unsigned char* navData = nullptr;
    int navDataSize = 0;
    bool ok = false;
};

rcConfig makeConfig(const Case& c) {
    rcConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.cs = c.cellSize;
    cfg.ch = c.cellHeight;
    cfg.walkableSlopeAngle = c.agentMaxSlope;
    cfg.walkableHeight = (int)ceilf(c.agentHeight / cfg.ch);
    cfg.walkableClimb = (int)floorf(c.agentMaxClimb / cfg.ch);
    cfg.walkableRadius = (int)ceilf(c.agentRadius / cfg.cs);
    cfg.maxEdgeLen = (int)(c.edgeMaxLen / c.cellSize);
    cfg.maxSimplificationError = c.edgeMaxError;
    cfg.minRegionArea = (int)rcSqr(c.regionMinSize);
    cfg.mergeRegionArea = (int)rcSqr(c.regionMergeSize);
    cfg.maxVertsPerPoly = c.vertsPerPoly;
    cfg.detailSampleDist = c.detailSampleDist < 0.9f ? 0 : c.cellSize * c.detailSampleDist;
    cfg.detailSampleMaxError = c.cellHeight * c.detailSampleMaxError;
    return cfg;
}

BuildOut buildOne(rcContext* ctx, const Case& c, const Geometry& g, rcConfig cfg, const std::vector<int>& tris,
                  int tileX, int tileY, bool record, const std::string& prefix) {
    BuildOut out;
    const int ntris = (int)tris.size() / 3;
    const int nverts = (int)g.verts.size() / 3;
    if (record) { Record r(prefix + "config"); streamConfig(r.s, cfg); }

    rcHeightfield* hf = rcAllocHeightfield();
    rcCompactHeightfield* chf = nullptr;
    rcContourSet* cset = nullptr;
    rcPolyMesh* pmesh = nullptr;
    rcPolyMeshDetail* dmesh = nullptr;
    std::vector<unsigned char> areas(ntris, 0);

    if (!rcCreateHeightfield(ctx, *hf, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch)) goto done;
    rcMarkWalkableTriangles(ctx, cfg.walkableSlopeAngle, g.verts.data(), nverts, tris.data(), ntris, areas.data());
    if (record) {
        Record r(prefix + "tri_areas");
        r.counts["walkable"] = 0;
        r.s.u32((uint32_t)areas.size());
        for (unsigned char a : areas) { r.s.u8(a); if (a) r.counts["walkable"]++; }
    }
    if (ntris && !rcRasterizeTriangles(ctx, g.verts.data(), nverts, tris.data(), areas.data(), ntris, *hf, cfg.walkableClimb)) goto done;
    if (record) { Record r(prefix + "heightfield_initial"); long long n; streamHeightfield(r.s, *hf, n); r.counts["spans"] = n; }
    if (c.filterLowHanging) rcFilterLowHangingWalkableObstacles(ctx, cfg.walkableClimb, *hf);
    if (c.filterLedge) rcFilterLedgeSpans(ctx, cfg.walkableHeight, cfg.walkableClimb, *hf);
    if (c.filterWalkableLowHeight) rcFilterWalkableLowHeightSpans(ctx, cfg.walkableHeight, *hf);
    if (record) { Record r(prefix + "heightfield_filtered"); long long n; streamHeightfield(r.s, *hf, n); r.counts["spans"] = n; }

    chf = rcAllocCompactHeightfield();
    if (!rcBuildCompactHeightfield(ctx, cfg.walkableHeight, cfg.walkableClimb, *hf, *chf)) goto done;
    if (record) { Record r(prefix + "compact_initial"); streamCompact(r.s, *chf, false, false); r.counts["spans"] = chf->spanCount; }
    if (!rcErodeWalkableArea(ctx, cfg.walkableRadius, *chf)) goto done;
    if (record) { Record r(prefix + "compact_eroded"); streamCompact(r.s, *chf, false, false); }
    if (markAreaVolumes(ctx, c, *chf) && record) { Record r(prefix + "compact_marked"); streamCompact(r.s, *chf, false, false); }

    if (c.partition == "watershed") {
        if (!rcBuildDistanceField(ctx, *chf)) goto done;
        if (record) { Record r(prefix + "distance_field"); streamCompact(r.s, *chf, true, false); r.counts["max_distance"] = chf->maxDistance; }
        if (!rcBuildRegions(ctx, *chf, cfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea)) goto done;
    } else if (c.partition == "monotone") {
        if (!rcBuildRegionsMonotone(ctx, *chf, cfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea)) goto done;
    } else if (c.partition == "layers") {
        if (!rcBuildLayerRegions(ctx, *chf, cfg.borderSize, cfg.minRegionArea)) goto done;
    } else {
        fprintf(stderr, "unknown partition %s\n", c.partition.c_str());
        goto done;
    }
    if (record) { Record r(prefix + "regions"); streamCompact(r.s, *chf, false, true); r.counts["max_regions"] = chf->maxRegions; }

    cset = rcAllocContourSet();
    if (!rcBuildContours(ctx, *chf, cfg.maxSimplificationError, cfg.maxEdgeLen, *cset)) goto done;
    if (record) { Record r(prefix + "contours"); long long nv; streamContours(r.s, *cset, nv); r.counts["contours"] = cset->nconts; r.counts["verts"] = nv; }

    pmesh = rcAllocPolyMesh();
    if (!rcBuildPolyMesh(ctx, *cset, cfg.maxVertsPerPoly, *pmesh)) goto done;
    // Walkable -> area 0 as in the upstream demo; flags from polyFlagsForArea.
    for (int i = 0; i < pmesh->npolys; ++i) {
        if (pmesh->areas[i] == RC_WALKABLE_AREA) pmesh->areas[i] = 0;
        pmesh->flags[i] = polyFlagsForArea(pmesh->areas[i]);
    }
    if (record) { Record r(prefix + "poly_mesh"); streamPolyMesh(r.s, *pmesh); r.counts["polys"] = pmesh->npolys; r.counts["verts"] = pmesh->nverts; }

    dmesh = rcAllocPolyMeshDetail();
    if (!rcBuildPolyMeshDetail(ctx, *pmesh, *chf, cfg.detailSampleDist, cfg.detailSampleMaxError, *dmesh)) goto done;
    if (record) { Record r(prefix + "detail_mesh"); streamDetail(r.s, *dmesh); r.counts["verts"] = dmesh->nverts; r.counts["tris"] = dmesh->ntris; }

    if (pmesh->npolys > 0) {
        dtNavMeshCreateParams p;
        memset(&p, 0, sizeof(p));
        p.verts = pmesh->verts; p.vertCount = pmesh->nverts;
        p.polys = pmesh->polys; p.polyAreas = pmesh->areas; p.polyFlags = pmesh->flags;
        p.polyCount = pmesh->npolys; p.nvp = pmesh->nvp;
        p.detailMeshes = dmesh->meshes; p.detailVerts = dmesh->verts; p.detailVertsCount = dmesh->nverts;
        p.detailTris = dmesh->tris; p.detailTriCount = dmesh->ntris;
        OffMeshArrays om(c);
        om.apply(p);
        p.walkableHeight = c.agentHeight; p.walkableRadius = c.agentRadius; p.walkableClimb = c.agentMaxClimb;
        p.tileX = tileX; p.tileY = tileY; p.tileLayer = 0;
        rcVcopy(p.bmin, pmesh->bmin); rcVcopy(p.bmax, pmesh->bmax);
        p.cs = cfg.cs; p.ch = cfg.ch; p.buildBvTree = true;
        if (!dtCreateNavMeshData(&p, &out.navData, &out.navDataSize)) goto done;
    }
    out.ok = true;

done:
    rcFreeHeightField(hf);
    rcFreeCompactHeightfield(chf);
    rcFreeContourSet(cset);
    rcFreePolyMesh(pmesh);
    rcFreePolyMeshDetail(dmesh);
    return out;
}

// ---------------------------------------------------------------------------
// Navmesh assembly.

struct TileKey { int x, y, layer; };

struct TileCacheState; // defined with the tile-cache bake below
void freeTileCacheState(TileCacheState* tc);

struct Baked {
    dtNavMesh* nav = nullptr;
    std::vector<TileKey> tileOrder;
    std::vector<std::vector<unsigned char>> tileBytes; // dtCreateNavMeshData output copies, before addTile
    TileCacheState* tc = nullptr;                     // tilecache mode only
    ~Baked() {
        freeTileCacheState(tc);
        dtFreeNavMesh(nav);
    }
};

bool bakeTileCache(const Case& c, const Geometry& g, bool record, Baked& out);

// Triangles per tile: those whose xz bounds overlap the tile's bordered bounds,
// in input order (reference/RECORDS.md, "Case files").
template <class TileConfigFn>
std::vector<std::vector<int>> bucketTriangles(const Geometry& g, const rcConfig& base, float tcs, int tw, int th,
                                              TileConfigFn tileConfig) {
    std::vector<std::vector<int>> buckets((size_t)tw * th);
    const int nt = (int)g.tris.size() / 3;
    const float margin = (base.walkableRadius + 3) * base.cs + tcs;
    for (int i = 0; i < nt; ++i) {
        float tmin[2] = {INFINITY, INFINITY}, tmax[2] = {-INFINITY, -INFINITY};
        for (int k = 0; k < 3; ++k) {
            const float* v = &g.verts[g.tris[i * 3 + k] * 3];
            tmin[0] = rcMin(tmin[0], v[0]); tmax[0] = rcMax(tmax[0], v[0]);
            tmin[1] = rcMin(tmin[1], v[2]); tmax[1] = rcMax(tmax[1], v[2]);
        }
        const int x0 = rcClamp((int)floorf((tmin[0] - g.bmin[0] - margin) / tcs), 0, tw - 1);
        const int x1 = rcClamp((int)floorf((tmax[0] - g.bmin[0] + margin) / tcs), 0, tw - 1);
        const int y0 = rcClamp((int)floorf((tmin[1] - g.bmin[2] - margin) / tcs), 0, th - 1);
        const int y1 = rcClamp((int)floorf((tmax[1] - g.bmin[2] + margin) / tcs), 0, th - 1);
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) {
                const rcConfig cfg = tileConfig(x, y);
                if (tmin[0] > cfg.bmax[0] || tmax[0] < cfg.bmin[0] || tmin[1] > cfg.bmax[2] || tmax[1] < cfg.bmin[2]) continue;
                std::vector<int>& b = buckets[(size_t)y * tw + x];
                b.push_back(g.tris[i * 3 + 0]); b.push_back(g.tris[i * 3 + 1]); b.push_back(g.tris[i * 3 + 2]);
            }
    }
    return buckets;
}

// Tile config as in the demo tile samples: border walkableRadius + 3 cells.
rcConfig tileConfigAt(const rcConfig& base, const Geometry& g, int ts, int x, int y) {
    const float tcs = ts * base.cs;
    rcConfig cfg = base;
    cfg.tileSize = ts;
    cfg.borderSize = cfg.walkableRadius + 3;
    cfg.width = ts + cfg.borderSize * 2;
    cfg.height = ts + cfg.borderSize * 2;
    cfg.bmin[0] = g.bmin[0] + x * tcs; cfg.bmin[1] = g.bmin[1]; cfg.bmin[2] = g.bmin[2] + y * tcs;
    cfg.bmax[0] = g.bmin[0] + (x + 1) * tcs; cfg.bmax[1] = g.bmax[1]; cfg.bmax[2] = g.bmin[2] + (y + 1) * tcs;
    cfg.bmin[0] -= cfg.borderSize * cfg.cs; cfg.bmin[2] -= cfg.borderSize * cfg.cs;
    cfg.bmax[0] += cfg.borderSize * cfg.cs; cfg.bmax[2] += cfg.borderSize * cfg.cs;
    return cfg;
}

bool bake(const Case& c, const Geometry& g, bool record, int threads, Baked& out) {
    rcConfig base = makeConfig(c);
    if (c.mode == "tilecache") return bakeTileCache(c, g, record, out);
    out.nav = dtAllocNavMesh();
    if (c.mode == "solo") {
        rcConfig cfg = base;
        rcVcopy(cfg.bmin, g.bmin); rcVcopy(cfg.bmax, g.bmax);
        rcCalcGridSize(cfg.bmin, cfg.bmax, cfg.cs, &cfg.width, &cfg.height);
        rcContext ctx(false);
        BuildOut b = buildOne(&ctx, c, g, cfg, g.tris, 0, 0, record, "");
        if (!b.ok || !b.navData) return false;
        out.tileOrder.push_back({0, 0, 0});
        out.tileBytes.emplace_back(b.navData, b.navData + b.navDataSize);
        if (dtStatusFailed(out.nav->init(b.navData, b.navDataSize, DT_TILE_FREE_DATA))) return false;
        return true;
    }
    if (c.mode != "tiled") { fprintf(stderr, "unknown mode %s\n", c.mode.c_str()); return false; }

    int gw = 0, gh = 0;
    rcCalcGridSize(g.bmin, g.bmax, c.cellSize, &gw, &gh);
    const int ts = c.tileSize;
    const int tw = (gw + ts - 1) / ts, th = (gh + ts - 1) / ts;
    const float tcs = ts * c.cellSize;
    int tileBits = 0;
    { unsigned v = (unsigned)(tw * th); unsigned p = 1; while (p < v) p <<= 1; while ((1u << tileBits) < p) ++tileBits; }
    tileBits = rcMin(tileBits, 14);
    dtNavMeshParams np;
    memset(&np, 0, sizeof(np));
    rcVcopy(np.orig, g.bmin);
    np.tileWidth = tcs; np.tileHeight = tcs;
    np.maxTiles = 1 << tileBits; np.maxPolys = 1 << (22 - tileBits);
    if (dtStatusFailed(out.nav->init(&np))) return false;

    std::vector<std::pair<int, int>> coords;
    for (int y = 0; y < th; ++y)
        for (int x = 0; x < tw; ++x) coords.push_back({x, y});
    std::vector<BuildOut> results(coords.size());

    auto tileConfig = [&](int x, int y) { return tileConfigAt(base, g, ts, x, y); };
    // Tile input: the triangles whose xz bounds overlap the tile's bordered bounds,
    // in input order. Both the oracle and the port use this rule, so the upstream
    // demo's chunky partition (which reorders triangles) stays out of the parity
    // surface. Bucketed in one pass instead of a scan per tile.
    const std::vector<std::vector<int>> buckets = bucketTriangles(g, base, tcs, tw, th, tileConfig);

    auto work = [&](size_t begin, size_t step, bool rec) {
        rcContext ctx(false);
        for (size_t i = begin; i < coords.size(); i += step) {
            const int x = coords[i].first, y = coords[i].second;
            const rcConfig cfg = tileConfig(x, y);
            const std::vector<int>& tris = buckets[i];
            char prefix[64];
            snprintf(prefix, sizeof(prefix), "tile_%d_%d.", x, y);
            results[i] = buildOne(&ctx, c, g, cfg, tris, x, y, rec, prefix);
        }
    };
    if (threads <= 1 || record) {
        work(0, 1, record); // records are printed in tile order, so recording runs serially
    } else {
        std::vector<std::thread> pool;
        for (int t = 0; t < threads; ++t) pool.emplace_back(work, (size_t)t, (size_t)threads, false);
        for (auto& th2 : pool) th2.join();
    }
    // Tiles are added in row-major order regardless of build order, so poly refs are stable.
    for (size_t i = 0; i < coords.size(); ++i) {
        if (!results[i].ok) return false;
        if (!results[i].navData) continue;
        out.tileOrder.push_back({coords[i].first, coords[i].second, 0});
        out.tileBytes.emplace_back(results[i].navData, results[i].navData + results[i].navDataSize);
        if (dtStatusFailed(out.nav->addTile(results[i].navData, results[i].navDataSize, DT_TILE_FREE_DATA, 0, 0))) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Tile cache, driven as the upstream demo's temporary-obstacle sample drives it,
// with an uncompressed layer format (RawCompressor) so the layer bytes do not
// depend on a third-party compressor.

struct RawCompressor : public dtTileCacheCompressor {
    int maxCompressedSize(const int bufferSize) override { return bufferSize; }
    dtStatus compress(const unsigned char* buffer, const int bufferSize, unsigned char* compressed,
                      const int maxCompressedSize, int* compressedSize) override {
        if (bufferSize > maxCompressedSize) return DT_FAILURE | DT_BUFFER_TOO_SMALL;
        memcpy(compressed, buffer, bufferSize);
        *compressedSize = bufferSize;
        return DT_SUCCESS;
    }
    dtStatus decompress(const unsigned char* compressed, const int compressedSize, unsigned char* buffer,
                        const int maxBufferSize, int* bufferSize) override {
        if (compressedSize > maxBufferSize) return DT_FAILURE | DT_BUFFER_TOO_SMALL;
        memcpy(buffer, compressed, compressedSize);
        *bufferSize = compressedSize;
        return DT_SUCCESS;
    }
};

// Same mapping as the solo/tiled builds, plus the case's off-mesh links.
struct MeshProcess : public dtTileCacheMeshProcess {
    const OffMeshArrays* om = nullptr;
    void process(dtNavMeshCreateParams* params, unsigned char* polyAreas, unsigned short* polyFlags) override {
        for (int i = 0; i < params->polyCount; ++i) {
            if (polyAreas[i] == DT_TILECACHE_WALKABLE_AREA) polyAreas[i] = 0;
            polyFlags[i] = polyFlagsForArea(polyAreas[i]);
        }
        if (om) om->apply(*params);
    }
};

struct TileCacheState {
    RawCompressor comp;
    dtTileCacheAlloc alloc;
    MeshProcess proc;
    OffMeshArrays om;
    dtTileCacheParams params;
    dtTileCache* cache = nullptr;
    explicit TileCacheState(const Case& c) : om(c) {
        proc.om = &om;
        memset(&params, 0, sizeof(params));
    }
    ~TileCacheState() { dtFreeTileCache(cache); }
};

void freeTileCacheState(TileCacheState* tc) { delete tc; }

const int kMaxLayersPerTile = 32;     // as the demo
const int kExpectedLayersPerTile = 4; // as the demo

// The steps of dtTileCache::buildNavMeshTile for a layer without obstacles,
// replicated with the public builder functions so each stage can be recorded.
// navOut stays empty when the layer yields no polygons.
bool buildLayerTile(TileCacheState& st, dtCompressedTileRef ref, bool record, std::vector<unsigned char>& navOut) {
    navOut.clear();
    const dtCompressedTile* tile = st.cache->getTileByRef(ref);
    if (!tile || !tile->header) return false;
    const dtTileCacheLayerHeader* h = tile->header;
    char prefix[64];
    snprintf(prefix, sizeof(prefix), "layer_%d_%d_%d.", h->tx, h->ty, h->tlayer);
    const std::string pre(prefix);
    const dtTileCacheParams& tp = st.params;
    const int walkableClimbVx = (int)(tp.walkableClimb / tp.ch);
    dtTileCacheLayer* layer = nullptr;
    dtTileCacheContourSet* lcset = nullptr;
    dtTileCachePolyMesh* lmesh = nullptr;
    bool ok = false;
    if (dtStatusFailed(dtDecompressTileCacheLayer(&st.alloc, &st.comp, tile->data, tile->dataSize, &layer))) goto done;
    if (record) { Record r(pre + "layer_grid"); streamLayerGrid(r.s, *layer); }
    if (dtStatusFailed(dtBuildTileCacheRegions(&st.alloc, *layer, walkableClimbVx))) goto done;
    if (record) { Record r(pre + "layer_regions"); streamLayerRegions(r.s, *layer); r.counts["regions"] = layer->regCount; }
    lcset = dtAllocTileCacheContourSet(&st.alloc);
    if (!lcset || dtStatusFailed(dtBuildTileCacheContours(&st.alloc, *layer, walkableClimbVx, tp.maxSimplificationError, *lcset)))
        goto done;
    if (record) {
        Record r(pre + "layer_contours");
        long long nv;
        streamLayerContours(r.s, *lcset, nv);
        r.counts["contours"] = lcset->nconts; r.counts["verts"] = nv;
    }
    lmesh = dtAllocTileCachePolyMesh(&st.alloc);
    if (!lmesh || dtStatusFailed(dtBuildTileCachePolyMesh(&st.alloc, *lcset, *lmesh))) goto done;
    if (lmesh->npolys) {
        dtNavMeshCreateParams params;
        memset(&params, 0, sizeof(params));
        params.verts = lmesh->verts; params.vertCount = lmesh->nverts;
        params.polys = lmesh->polys; params.polyAreas = lmesh->areas; params.polyFlags = lmesh->flags;
        params.polyCount = lmesh->npolys; params.nvp = DT_VERTS_PER_POLYGON;
        params.walkableHeight = tp.walkableHeight; params.walkableRadius = tp.walkableRadius;
        params.walkableClimb = tp.walkableClimb;
        params.tileX = h->tx; params.tileY = h->ty; params.tileLayer = h->tlayer;
        params.cs = tp.cs; params.ch = tp.ch; params.buildBvTree = false;
        dtVcopy(params.bmin, h->bmin); dtVcopy(params.bmax, h->bmax);
        st.proc.process(&params, lmesh->areas, lmesh->flags);
        if (record) {
            Record r(pre + "layer_poly_mesh");
            streamLayerPolyMesh(r.s, *lmesh);
            r.counts["polys"] = lmesh->npolys; r.counts["verts"] = lmesh->nverts;
        }
        unsigned char* navData = nullptr;
        int navDataSize = 0;
        if (!dtCreateNavMeshData(&params, &navData, &navDataSize)) goto done;
        navOut.assign(navData, navData + navDataSize);
        dtFree(navData);
    } else if (record) {
        Record r(pre + "layer_poly_mesh");
        streamLayerPolyMesh(r.s, *lmesh);
        r.counts["polys"] = 0; r.counts["verts"] = lmesh->nverts;
    }
    ok = true;
done:
    dtFreeTileCacheLayer(&st.alloc, layer);
    dtFreeTileCacheContourSet(&st.alloc, lcset);
    dtFreeTileCachePolyMesh(&st.alloc, lmesh);
    return ok;
}

// The navmesh rewrites first-link fields and the link section when it adds a
// tile; every other byte must equal the dtCreateNavMeshData output.
bool sameAsResident(const std::vector<unsigned char>& built, const dtMeshTile* tile) {
    if (!tile || !tile->header || tile->dataSize != (int)built.size()) return false;
    std::vector<unsigned char> res(tile->data, tile->data + tile->dataSize);
    dtMeshHeader h;
    memcpy(&h, res.data(), sizeof(h));
    const size_t headerSize = dtAlign4(sizeof(dtMeshHeader));
    const size_t vertsSize = dtAlign4(sizeof(float) * 3 * h.vertCount);
    const size_t polysSize = dtAlign4(sizeof(dtPoly) * h.polyCount);
    const size_t linksSize = dtAlign4(sizeof(dtLink) * h.maxLinkCount);
    if (headerSize + vertsSize + polysSize + linksSize > res.size()) return false;
    unsigned char* polys = res.data() + headerSize + vertsSize;
    for (int i = 0; i < h.polyCount; ++i) memset(polys + i * sizeof(dtPoly) + offsetof(dtPoly, firstLink), 0, sizeof(unsigned int));
    memset(polys + polysSize, 0, linksSize);
    return memcmp(res.data(), built.data(), built.size()) == 0;
}

bool bakeTileCache(const Case& c, const Geometry& g, bool record, Baked& out) {
    const rcConfig base = makeConfig(c);
    int gw = 0, gh = 0;
    rcCalcGridSize(g.bmin, g.bmax, c.cellSize, &gw, &gh);
    const int ts = c.tileSize;
    const int border = base.walkableRadius + 3;
    if (ts <= 0 || ts + 2 * border > 255) {
        fprintf(stderr, "tilecache: tile_size %d plus a %d-cell border on each side exceeds the 255-cell layer limit\n", ts, border);
        return false;
    }
    const int tw = (gw + ts - 1) / ts, th = (gh + ts - 1) / ts;
    const float tcs = ts * c.cellSize;
    const int tileBits = rcMin((int)dtIlog2(dtNextPow2((unsigned int)(tw * th * kExpectedLayersPerTile))), 14);

    dtNavMeshParams np;
    memset(&np, 0, sizeof(np));
    rcVcopy(np.orig, g.bmin);
    np.tileWidth = tcs; np.tileHeight = tcs;
    np.maxTiles = 1 << tileBits; np.maxPolys = 1 << (22 - tileBits);
    out.nav = dtAllocNavMesh();
    if (dtStatusFailed(out.nav->init(&np))) return false;

    out.tc = new TileCacheState(c);
    TileCacheState& st = *out.tc;
    dtTileCacheParams& tp = st.params;
    rcVcopy(tp.orig, g.bmin);
    tp.cs = c.cellSize; tp.ch = c.cellHeight;
    tp.width = ts; tp.height = ts;
    tp.walkableHeight = c.agentHeight; tp.walkableRadius = c.agentRadius; tp.walkableClimb = c.agentMaxClimb;
    tp.maxSimplificationError = c.edgeMaxError;
    tp.maxTiles = tw * th * kExpectedLayersPerTile;
    tp.maxObstacles = 128;
    st.cache = dtAllocTileCache();
    if (!st.cache || dtStatusFailed(st.cache->init(&tp, &st.alloc, &st.comp, &st.proc))) return false;

    auto tileConfig = [&](int x, int y) { return tileConfigAt(base, g, ts, x, y); };
    const std::vector<std::vector<int>> buckets = bucketTriangles(g, base, tcs, tw, th, tileConfig);
    const int nverts = (int)g.verts.size() / 3;
    rcContext ctx(false);
    for (int y = 0; y < th; ++y)
        for (int x = 0; x < tw; ++x) {
            const rcConfig cfg = tileConfig(x, y);
            const std::vector<int>& tris = buckets[(size_t)y * tw + x];
            const int ntris = (int)tris.size() / 3;
            char prefixBuf[64];
            snprintf(prefixBuf, sizeof(prefixBuf), "tile_%d_%d.", x, y);
            const std::string prefix(prefixBuf);
            if (record) { Record r(prefix + "config"); streamConfig(r.s, cfg); }
            rcHeightfield* hf = rcAllocHeightfield();
            rcCompactHeightfield* chf = nullptr;
            rcHeightfieldLayerSet* lset = nullptr;
            std::vector<unsigned char> areas(ntris, 0);
            bool ok = false;
            if (!rcCreateHeightfield(&ctx, *hf, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch)) goto tileDone;
            rcMarkWalkableTriangles(&ctx, cfg.walkableSlopeAngle, g.verts.data(), nverts, tris.data(), ntris, areas.data());
            if (record) {
                Record r(prefix + "tri_areas");
                r.counts["walkable"] = 0;
                r.s.u32((uint32_t)areas.size());
                for (unsigned char a : areas) { r.s.u8(a); if (a) r.counts["walkable"]++; }
            }
            if (ntris && !rcRasterizeTriangles(&ctx, g.verts.data(), nverts, tris.data(), areas.data(), ntris, *hf, cfg.walkableClimb))
                goto tileDone;
            if (record) { Record r(prefix + "heightfield_initial"); long long n; streamHeightfield(r.s, *hf, n); r.counts["spans"] = n; }
            if (c.filterLowHanging) rcFilterLowHangingWalkableObstacles(&ctx, cfg.walkableClimb, *hf);
            if (c.filterLedge) rcFilterLedgeSpans(&ctx, cfg.walkableHeight, cfg.walkableClimb, *hf);
            if (c.filterWalkableLowHeight) rcFilterWalkableLowHeightSpans(&ctx, cfg.walkableHeight, *hf);
            if (record) { Record r(prefix + "heightfield_filtered"); long long n; streamHeightfield(r.s, *hf, n); r.counts["spans"] = n; }
            chf = rcAllocCompactHeightfield();
            if (!rcBuildCompactHeightfield(&ctx, cfg.walkableHeight, cfg.walkableClimb, *hf, *chf)) goto tileDone;
            if (record) { Record r(prefix + "compact_initial"); streamCompact(r.s, *chf, false, false); r.counts["spans"] = chf->spanCount; }
            if (!rcErodeWalkableArea(&ctx, cfg.walkableRadius, *chf)) goto tileDone;
            if (record) { Record r(prefix + "compact_eroded"); streamCompact(r.s, *chf, false, false); }
            if (markAreaVolumes(&ctx, c, *chf) && record) { Record r(prefix + "compact_marked"); streamCompact(r.s, *chf, false, false); }
            lset = rcAllocHeightfieldLayerSet();
            if (!lset || !rcBuildHeightfieldLayers(&ctx, *chf, cfg.borderSize, cfg.walkableHeight, *lset)) goto tileDone;
            if (record) { Record r(prefix + "heightfield_layers"); streamLayers(r.s, *lset); r.counts["layers"] = lset->nlayers; }
            ok = true;
            for (int i = 0; i < rcMin(lset->nlayers, kMaxLayersPerTile) && ok; ++i) {
                const rcHeightfieldLayer& l = lset->layers[i];
                // Zeroed first: the header's padding bytes are copied into the layer data.
                dtTileCacheLayerHeader header;
                memset(&header, 0, sizeof(header));
                header.magic = DT_TILECACHE_MAGIC; header.version = DT_TILECACHE_VERSION;
                header.tx = x; header.ty = y; header.tlayer = i;
                dtVcopy(header.bmin, l.bmin); dtVcopy(header.bmax, l.bmax);
                header.width = (unsigned char)l.width; header.height = (unsigned char)l.height;
                header.minx = (unsigned char)l.minx; header.maxx = (unsigned char)l.maxx;
                header.miny = (unsigned char)l.miny; header.maxy = (unsigned char)l.maxy;
                header.hmin = (unsigned short)l.hmin; header.hmax = (unsigned short)l.hmax;
                unsigned char* data = nullptr;
                int dataSize = 0;
                if (dtStatusFailed(dtBuildTileCacheLayer(&st.comp, &header, l.heights, l.areas, l.cons, &data, &dataSize))) {
                    ok = false;
                    break;
                }
                if (record) {
                    char lname[64];
                    snprintf(lname, sizeof(lname), "layer_%d_%d_%d.layer_data", x, y, i);
                    Record r(lname);
                    r.s.u32((uint32_t)dataSize);
                    r.s.put(data, (size_t)dataSize);
                }
                if (dtStatusFailed(st.cache->addTile(data, dataSize, DT_COMPRESSEDTILE_FREE_DATA, 0))) {
                    dtFree(data);
                    ok = false;
                }
            }
        tileDone:
            rcFreeHeightField(hf);
            rcFreeCompactHeightfield(chf);
            rcFreeHeightfieldLayerSet(lset);
            if (!ok) return false;
        }

    // Navmesh tiles, row-major, layers in the cache's order for the location
    // (the order buildNavMeshTilesAt adds them in).
    for (int y = 0; y < th; ++y)
        for (int x = 0; x < tw; ++x) {
            dtCompressedTileRef refs[kMaxLayersPerTile];
            const int n = st.cache->getTilesAt(x, y, refs, kMaxLayersPerTile);
            std::vector<std::vector<unsigned char>> built((size_t)n);
            for (int i = 0; i < n; ++i)
                if (!buildLayerTile(st, refs[i], record, built[i])) return false;
            if (dtStatusFailed(st.cache->buildNavMeshTilesAt(x, y, out.nav))) return false;
            for (int i = 0; i < n; ++i) {
                if (built[i].empty()) continue;
                const int layer = st.cache->getTileByRef(refs[i])->header->tlayer;
                if (!sameAsResident(built[i], out.nav->getTileAt(x, y, layer))) {
                    fprintf(stderr, "tilecache: replicated build of layer %d,%d,%d differs from the tile cache's\n", x, y, layer);
                    return false;
                }
                out.tileOrder.push_back({x, y, layer});
                out.tileBytes.push_back(built[i]);
            }
        }
    return true;
}

// ---------------------------------------------------------------------------
// Detour queries. Points come from findRandomPoint driven by a fixed xorshift32,
// so the query set is a pure function of the navmesh and the seed.

uint32_t g_rng = 1;
float frand() {
    g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5;
    return (float)(g_rng >> 8) * (1.0f / 16777216.0f);
}

void recordNavmesh(const Baked& b) {
    Record r("navmesh_tiles");
    r.counts["tiles"] = (long long)b.tileBytes.size();
    long long polys = 0;
    r.s.u32((uint32_t)b.tileBytes.size());
    for (size_t i = 0; i < b.tileBytes.size(); ++i) {
        r.s.i32(b.tileOrder[i].x); r.s.i32(b.tileOrder[i].y); r.s.i32(b.tileOrder[i].layer);
        r.s.u32((uint32_t)b.tileBytes[i].size());
        r.s.put(b.tileBytes[i].data(), b.tileBytes[i].size()); // upstream tile bytes, host endianness (LE)
        const dtMeshHeader* h = (const dtMeshHeader*)b.tileBytes[i].data();
        polys += h->polyCount;
    }
    r.counts["polys"] = polys;
}

// The second query filter: excludes flag 0x10 (area 4, see polyFlagsForArea)
// and gives areas 1..7 costs above 1.
void makeCostFilter(dtQueryFilter& f) {
    f.setIncludeFlags(0xffff);
    f.setExcludeFlags(0x10);
    for (int a = 1; a < 8; ++a) f.setAreaCost(a, 1.0f + 1.5f * (float)a);
}

// One query set over the navmesh. Every record is prefixed (empty for the
// default dtQueryFilter, "filtered." for makeCostFilter). Records whose payload
// has one entry per pair or per point start with that entry count (u32).
void recordQueries(const Case& c, dtNavMesh* nav, const dtQueryFilter& filter, const std::string& pre) {
    dtNavMeshQuery* q = dtAllocNavMeshQuery();
    q->init(nav, 2048);
    g_rng = c.seed ? c.seed : 1;
    const int n = c.queries;
    std::vector<dtPolyRef> refs(n + 1);
    std::vector<float> pts((n + 1) * 3);
    {
        Record r(pre + "q_random_point");
        r.s.u32((uint32_t)(n + 1));
        for (int i = 0; i <= n; ++i) {
            dtStatus st = q->findRandomPoint(&filter, frand, &refs[i], &pts[i * 3]);
            r.s.u32(st); r.s.ref(refs[i]); r.s.v3(&pts[i * 3]);
        }
        r.counts["points"] = n + 1;
    }
    const float ext[3] = {2.0f, 4.0f, 2.0f};
    {
        Record r(pre + "q_nearest_poly");
        Record rq(pre + "q_query_polygons");
        r.s.u32((uint32_t)(n + 1));
        rq.s.u32((uint32_t)(n + 1));
        for (int i = 0; i <= n; ++i) {
            float probe[3] = {pts[i * 3] + 0.37f, pts[i * 3 + 1] + 1.0f, pts[i * 3 + 2] - 0.21f};
            dtPolyRef ref = 0; float np[3] = {0, 0, 0}; bool over = false;
            dtStatus st = q->findNearestPoly(probe, ext, &filter, &ref, np, &over);
            r.s.u32(st); r.s.ref(ref); r.s.v3(np); r.s.u8(over);
            dtPolyRef polys[64]; int npolys = 0;
            dtStatus sq = q->queryPolygons(probe, ext, &filter, polys, &npolys, 64);
            rq.s.u32(sq); rq.s.i32(npolys); for (int k = 0; k < npolys; ++k) rq.s.ref(polys[k]);
        }
    }
    const int MAXP = 256;
    std::vector<dtPolyRef> path(MAXP), path2(MAXP), path3(MAXP), sref(MAXP);
    std::vector<float> spts(MAXP * 3);
    std::vector<unsigned char> sflags(MAXP);
    const int NSET = 15;
    const char* names[NSET] = {"q_find_path", "q_straight_path", "q_straight_path_all_crossings",
                               "q_straight_path_area_crossings", "q_sliced_path", "q_partial_sliced_path",
                               "q_raycast", "q_raycast_plain", "q_distance_to_wall", "q_move_along_surface",
                               "q_polys_around_circle", "q_polys_around_shape", "q_local_neighbourhood",
                               "q_poly_wall_segments", "q_closest_point_on_poly"};
    std::vector<Record*> recs;
    for (int k = 0; k < NSET; ++k) recs.push_back(new Record(pre + names[k]));
    Record& rp = *recs[0]; Record& rsl = *recs[4]; Record& rpart = *recs[5];
    Record& rr = *recs[6]; Record& rrp = *recs[7]; Record& rw = *recs[8]; Record& rm = *recs[9];
    Record& rcirc = *recs[10]; Record& rshape = *recs[11]; Record& rloc = *recs[12]; Record& rwall = *recs[13];
    Record& rcp = *recs[14];
    Record rcpb(pre + "q_closest_point_on_poly_boundary"), rph(pre + "q_poly_height"),
        rrc(pre + "q_random_point_around_circle");
    for (Record* r : recs) r->s.u32((uint32_t)n);
    rcpb.s.u32((uint32_t)n); rph.s.u32((uint32_t)n); rrc.s.u32((uint32_t)n);
    long long slicedMismatch = 0, pathPolys = 0;
    for (int i = 0; i < n; ++i) {
        const float* a = &pts[i * 3]; const float* e = &pts[(i + 1) * 3];
        int np = 0;
        dtStatus st = q->findPath(refs[i], refs[i + 1], a, e, &filter, path.data(), &np, MAXP);
        rp.s.u32(st); rp.s.i32(np); for (int k = 0; k < np; ++k) rp.s.ref(path[k]);
        pathPolys += np;
        const int opts[3] = {0, DT_STRAIGHTPATH_ALL_CROSSINGS, DT_STRAIGHTPATH_AREA_CROSSINGS};
        for (int o = 0; o < 3; ++o) {
            Record& rec = *recs[1 + o];
            int ns = 0;
            dtStatus st2 = q->findStraightPath(a, e, path.data(), np, spts.data(), sflags.data(), sref.data(), &ns, MAXP, opts[o]);
            rec.s.u32(st2); rec.s.i32(ns);
            for (int k = 0; k < ns; ++k) { rec.s.v3(&spts[k * 3]); rec.s.u8(sflags[k]); rec.s.ref(sref[k]); }
        }
        // Sliced search, 8 iterations per update; the record holds the sliced result
        // and the mismatch count against the unsliced one.
        int np2 = 0, done = 0;
        dtStatus ss = q->initSlicedFindPath(refs[i], refs[i + 1], a, e, &filter, 0);
        while (dtStatusInProgress(ss)) ss = q->updateSlicedFindPath(8, &done);
        ss = q->finalizeSlicedFindPath(path2.data(), &np2, MAXP);
        rsl.s.u32(ss); rsl.s.i32(np2); for (int k = 0; k < np2; ++k) rsl.s.ref(path2[k]);
        if (np2 != np || memcmp(path.data(), path2.data(), sizeof(dtPolyRef) * np) != 0) ++slicedMismatch;

        // Partial sliced search: one update of 4 iterations, then finalize against
        // the unsliced corridor as the existing path.
        {
            int np3 = 0, done3 = 0;
            dtStatus s0 = q->initSlicedFindPath(refs[i], refs[i + 1], a, e, &filter, 0);
            dtStatus s1 = dtStatusInProgress(s0) ? q->updateSlicedFindPath(4, &done3) : s0;
            dtStatus s2 = q->finalizeSlicedFindPathPartial(path.data(), np, path3.data(), &np3, MAXP);
            rpart.s.u32(s0); rpart.s.u32(s1); rpart.s.i32(done3); rpart.s.u32(s2); rpart.s.i32(np3);
            for (int k = 0; k < np3; ++k) rpart.s.ref(path3[k]);
        }

        dtRaycastHit hit;
        memset(&hit, 0, sizeof(hit));
        hit.path = path2.data(); hit.maxPath = MAXP;
        dtStatus sr = q->raycast(refs[i], a, e, &filter, DT_RAYCAST_USE_COSTS, &hit);
        rr.s.u32(sr); rr.s.f32(hit.t); rr.s.v3(hit.hitNormal); rr.s.i32(hit.hitEdgeIndex); rr.s.i32(hit.pathCount);
        for (int k = 0; k < hit.pathCount; ++k) rr.s.ref(path2[k]);
        rr.s.f32(hit.pathCost);

        {
            float t = 0, hn[3] = {0, 0, 0};
            int nr = 0;
            dtStatus s = q->raycast(refs[i], a, e, &filter, &t, hn, path3.data(), &nr, MAXP);
            rrp.s.u32(s); rrp.s.f32(t); rrp.s.v3(hn); rrp.s.i32(nr);
            for (int k = 0; k < nr; ++k) rrp.s.ref(path3[k]);
        }

        float hd = 0, hp[3] = {0, 0, 0}, hn[3] = {0, 0, 0};
        dtStatus sw = q->findDistanceToWall(refs[i], a, 5.0f, &filter, &hd, hp, hn);
        rw.s.u32(sw); rw.s.f32(hd); rw.s.v3(hp); rw.s.v3(hn);

        float res[3] = {0, 0, 0}; dtPolyRef vis[16]; int nvis = 0;
        dtStatus sm = q->moveAlongSurface(refs[i], a, e, &filter, res, vis, &nvis, 16);
        rm.s.u32(sm); rm.s.v3(res); rm.s.i32(nvis); for (int k = 0; k < nvis; ++k) rm.s.ref(vis[k]);

        const int MAXR = 64;
        dtPolyRef rref[MAXR], rpar[MAXR];
        float rcost[MAXR];
        {
            int nres = 0;
            dtStatus s = q->findPolysAroundCircle(refs[i], a, 4.0f, &filter, rref, rpar, rcost, &nres, MAXR);
            rcirc.s.u32(s); rcirc.s.i32(nres);
            for (int k = 0; k < nres; ++k) { rcirc.s.ref(rref[k]); rcirc.s.ref(rpar[k]); rcirc.s.f32(rcost[k]); }
        }
        {
            // Axis-aligned 4 x 4 square around the start point, counter-clockwise seen from +y.
            const float shape[12] = {a[0] - 2.0f, a[1], a[2] - 2.0f, a[0] - 2.0f, a[1], a[2] + 2.0f,
                                     a[0] + 2.0f, a[1], a[2] + 2.0f, a[0] + 2.0f, a[1], a[2] - 2.0f};
            int nres = 0;
            dtStatus s = q->findPolysAroundShape(refs[i], shape, 4, &filter, rref, rpar, rcost, &nres, MAXR);
            rshape.s.u32(s); rshape.s.i32(nres);
            for (int k = 0; k < nres; ++k) { rshape.s.ref(rref[k]); rshape.s.ref(rpar[k]); rshape.s.f32(rcost[k]); }
        }
        {
            int nres = 0;
            dtStatus s = q->findLocalNeighbourhood(refs[i], a, 3.0f, &filter, rref, rpar, &nres, 32);
            rloc.s.u32(s); rloc.s.i32(nres);
            for (int k = 0; k < nres; ++k) { rloc.s.ref(rref[k]); rloc.s.ref(rpar[k]); }
        }
        {
            float segs[MAXR * 6];
            dtPolyRef segRefs[MAXR];
            int nseg = 0;
            dtStatus s = q->getPolyWallSegments(refs[i], &filter, segs, segRefs, &nseg, MAXR);
            rwall.s.u32(s); rwall.s.i32(nseg);
            for (int k = 0; k < nseg; ++k) { rwall.s.v3(&segs[k * 6]); rwall.s.v3(&segs[k * 6 + 3]); rwall.s.ref(segRefs[k]); }
        }
        {
            // The pair's end point projected on the start polygon.
            float cp[3] = {0, 0, 0};
            bool over = false;
            dtStatus s = q->closestPointOnPoly(refs[i], e, cp, &over);
            rcp.s.u32(s); rcp.s.v3(cp); rcp.s.u8(over);
            float cb[3] = {0, 0, 0};
            dtStatus sb = q->closestPointOnPolyBoundary(refs[i], e, cb);
            rcpb.s.u32(sb); rcpb.s.v3(cb);
        }
        {
            float ha = 0, he = 0;
            dtStatus sa = q->getPolyHeight(refs[i], a, &ha);
            dtStatus se = q->getPolyHeight(refs[i], e, &he);
            rph.s.u32(sa); rph.s.f32(ha); rph.s.u32(se); rph.s.f32(he);
        }
        {
            dtPolyRef rref1 = 0;
            float rp1[3] = {0, 0, 0};
            dtStatus s = q->findRandomPointAroundCircle(refs[i], a, 3.0f, &filter, frand, &rref1, rp1);
            rrc.s.u32(s); rrc.s.ref(rref1); rrc.s.v3(rp1);
        }
    }
    rp.counts["pairs"] = n; rp.counts["path_polys"] = pathPolys;
    rsl.counts["sliced_vs_unsliced_mismatches"] = slicedMismatch;
    for (Record* r : recs) delete r; // framed in declaration order
    dtFreeNavMeshQuery(q);
}

// Crowd variants (reference/RECORDS.md). Each run builds a fresh crowd over
// `nav`, places agents with findRandomPoint from a fixed seed and records, per
// tick, every active agent.
enum CrowdVariant {
    CROWD_DEFAULT,       // crowd_trajectories: avoidance type 3 with dtCrowd's default parameters
    CROWD_QUALITY,       // crowd_quality_<q>: the demo's four avoidance presets
    CROWD_MOVE_VELOCITY, // crowd_move_velocity: velocity requests, then target resets
    CROWD_REMOVE_READD,  // crowd_remove_readd: agents removed at T/3, new agents added at 2T/3
    CROWD_TILE_SWAP,     // crowd_tile_swap: the tile under agent 0 removed and re-added at T/2
};

void addCrowdAgent(dtCrowd* crowd, dtNavMeshQuery* q, const dtQueryFilter& filter, const dtCrowdAgentParams& ap) {
    dtPolyRef r0 = 0, r1 = 0;
    float p0[3] = {0, 0, 0}, p1[3] = {0, 0, 0};
    q->findRandomPoint(&filter, frand, &r0, p0);
    q->findRandomPoint(&filter, frand, &r1, p1);
    const int idx = crowd->addAgent(p0, &ap);
    if (idx >= 0) crowd->requestMoveTarget(idx, r1, p1);
}

void recordCrowd(const Case& c, dtNavMesh* nav, const Baked& b, CrowdVariant variant, int quality) {
    if (c.crowdAgents <= 0 || c.crowdTicks <= 0) return;
    dtNavMeshQuery* q = dtAllocNavMeshQuery();
    q->init(nav, 2048);
    dtQueryFilter filter;
    dtCrowd* crowd = dtAllocCrowd();
    crowd->init(c.crowdAgents, c.agentRadius, nav);
    if (variant == CROWD_QUALITY) {
        // Upstream demo presets (RecastDemo Tool_Crowd.cpp): low, medium, good, high.
        static const unsigned char divs[4] = {5, 5, 7, 7}, rings[4] = {2, 2, 2, 3}, depth[4] = {1, 2, 3, 3};
        dtObstacleAvoidanceParams p;
        memcpy(&p, crowd->getObstacleAvoidanceParams(0), sizeof(p));
        for (int k = 0; k < 4; ++k) {
            p.velBias = 0.5f; p.adaptiveDivs = divs[k]; p.adaptiveRings = rings[k]; p.adaptiveDepth = depth[k];
            crowd->setObstacleAvoidanceParams(k, &p);
        }
    }
    g_rng = (c.seed ^ 0x5bd1e995u) ? (c.seed ^ 0x5bd1e995u) : 1;
    dtCrowdAgentParams ap;
    memset(&ap, 0, sizeof(ap));
    ap.radius = c.agentRadius; ap.height = c.agentHeight;
    ap.maxAcceleration = 8.0f; ap.maxSpeed = 3.5f;
    ap.collisionQueryRange = ap.radius * 12.0f; ap.pathOptimizationRange = ap.radius * 30.0f;
    ap.separationWeight = 2.0f;
    ap.updateFlags = DT_CROWD_ANTICIPATE_TURNS | DT_CROWD_OPTIMIZE_VIS | DT_CROWD_OPTIMIZE_TOPO |
                     DT_CROWD_OBSTACLE_AVOIDANCE | DT_CROWD_SEPARATION;
    ap.obstacleAvoidanceType = (unsigned char)(variant == CROWD_QUALITY ? quality : 3);
    for (int i = 0; i < c.crowdAgents; ++i) addCrowdAgent(crowd, q, filter, ap);

    std::string name = "crowd_trajectories";
    if (variant == CROWD_QUALITY) name = "crowd_quality_" + std::to_string(quality);
    else if (variant == CROWD_MOVE_VELOCITY) name = "crowd_move_velocity";
    else if (variant == CROWD_REMOVE_READD) name = "crowd_remove_readd";
    else if (variant == CROWD_TILE_SWAP) name = "crowd_tile_swap";
    Record r(name);
    r.counts["agents"] = c.crowdAgents; r.counts["ticks"] = c.crowdTicks;
    r.s.u32((uint32_t)c.crowdTicks);

    if (variant == CROWD_MOVE_VELOCITY) {
        // Odd agents switch from their target to a velocity toward it at half speed.
        for (int i = 1; i < crowd->getAgentCount(); i += 2) {
            const dtCrowdAgent* ag = crowd->getAgent(i);
            if (!ag->active) continue;
            float d[3];
            dtVsub(d, ag->targetPos, ag->npos);
            d[1] = 0;
            const float len = dtMathSqrtf(d[0] * d[0] + d[2] * d[2]);
            float v[3] = {0, 0, 0};
            if (len > 0.0001f) dtVscale(v, d, 0.5f * ap.maxSpeed / len);
            crowd->requestMoveVelocity(i, v);
        }
    }
    const float dt = 1.0f / 30.0f;
    for (int t = 0; t < c.crowdTicks; ++t) {
        if (variant == CROWD_MOVE_VELOCITY && t == c.crowdTicks / 2) {
            for (int i = 0; i < crowd->getAgentCount(); i += 3) crowd->resetMoveTarget(i);
        }
        if (variant == CROWD_REMOVE_READD && t == c.crowdTicks / 3) {
            for (int i = 1; i < crowd->getAgentCount(); i += 4) crowd->removeAgent(i);
        }
        if (variant == CROWD_REMOVE_READD && t == (2 * c.crowdTicks) / 3) {
            for (int i = 1; i < crowd->getAgentCount(); i += 4) addCrowdAgent(crowd, q, filter, ap);
        }
        if (variant == CROWD_TILE_SWAP && t == c.crowdTicks / 2) {
            // Remove the tile under agent 0 and add the same dtCreateNavMeshData bytes back.
            const dtCrowdAgent* ag0 = crowd->getAgent(0);
            const dtMeshTile* tile = nullptr;
            const dtPoly* poly = nullptr;
            if (ag0->active && dtStatusSucceed(nav->getTileAndPolyByRef(ag0->corridor.getFirstPoly(), &tile, &poly))) {
                const int tx = tile->header->x, ty = tile->header->y, tl = tile->header->layer;
                for (size_t k = 0; k < b.tileOrder.size(); ++k) {
                    if (b.tileOrder[k].x != tx || b.tileOrder[k].y != ty || b.tileOrder[k].layer != tl) continue;
                    nav->removeTile(nav->getTileRefAt(tx, ty, tl), nullptr, nullptr);
                    unsigned char* data = (unsigned char*)dtAlloc(b.tileBytes[k].size(), DT_ALLOC_PERM);
                    memcpy(data, b.tileBytes[k].data(), b.tileBytes[k].size());
                    if (dtStatusFailed(nav->addTile(data, (int)b.tileBytes[k].size(), DT_TILE_FREE_DATA, 0, nullptr))) dtFree(data);
                    r.counts["swapped_tiles"] = 1;
                    break;
                }
            }
        }
        crowd->update(dt, nullptr);
        uint32_t active = 0;
        for (int i = 0; i < crowd->getAgentCount(); ++i) active += crowd->getAgent(i)->active ? 1 : 0;
        r.s.u32((uint32_t)t); r.s.u32(active);
        for (int i = 0; i < crowd->getAgentCount(); ++i) {
            const dtCrowdAgent* ag = crowd->getAgent(i);
            if (!ag->active) continue;
            r.s.u32((uint32_t)i); r.s.u8(ag->state); r.s.u8(ag->targetState);
            r.s.v3(ag->npos); r.s.v3(ag->vel); r.s.v3(ag->nvel);
            r.s.ref(ag->corridor.getFirstPoly()); r.s.i32(ag->ncorners);
            r.s.u32((uint32_t)ag->corridor.getPathCount());
            for (int k = 0; k < ag->corridor.getPathCount(); ++k) r.s.ref(ag->corridor.getPath()[k]);
        }
    }
    dtFreeCrowd(crowd);
    dtFreeNavMeshQuery(q);
}

// A fresh navmesh holding copies of the baked tiles, in the same add order.
dtNavMesh* cloneNavMesh(const Baked& b) {
    dtNavMesh* nav = dtAllocNavMesh();
    const dtNavMeshParams* params = b.nav->getParams();
    dtStatus st = nav->init(params);
    for (size_t k = 0; k < b.tileBytes.size() && dtStatusSucceed(st); ++k) {
        unsigned char* data = (unsigned char*)dtAlloc(b.tileBytes[k].size(), DT_ALLOC_PERM);
        memcpy(data, b.tileBytes[k].data(), b.tileBytes[k].size());
        st = nav->addTile(data, (int)b.tileBytes[k].size(), DT_TILE_FREE_DATA, 0, nullptr);
        if (dtStatusFailed(st)) dtFree(data);
    }
    if (dtStatusFailed(st)) { dtFreeNavMesh(nav); return nullptr; }
    return nav;
}

// Upstream's defined verdicts on malformed or conflicting tile data, on a fresh
// navmesh with the baked navmesh's parameters. Only inputs whose upstream
// behaviour is defined are used (header checks that run before any section is
// read, slot conflicts, invalid refs).
void recordMalformedTiles(const Baked& b) {
    if (b.tileBytes.empty()) return;
    Record r("malformed_tiles");
    const std::vector<unsigned char>& src = b.tileBytes[0];
    const int NV = 8;
    r.s.u32((uint32_t)NV);
    dtNavMesh* nav = dtAllocNavMesh();
    nav->init(b.nav->getParams());
    auto copyOf = [&](void (*mutate)(dtMeshHeader&)) {
        unsigned char* data = (unsigned char*)dtAlloc(src.size(), DT_ALLOC_PERM);
        memcpy(data, src.data(), src.size());
        if (mutate) {
            dtMeshHeader h;
            memcpy(&h, data, sizeof(h));
            mutate(h);
            memcpy(data, &h, sizeof(h));
        }
        return data;
    };
    auto add = [&](uint32_t code, unsigned char* data, dtTileRef* out) {
        dtTileRef ref = 0;
        dtStatus st = nav->addTile(data, (int)src.size(), DT_TILE_FREE_DATA, 0, &ref);
        if (dtStatusFailed(st)) dtFree(data);
        r.s.u32(code); r.s.u32(st); r.s.ref(ref);
        if (out) *out = ref;
    };
    dtTileRef valid = 0;
    add(1, copyOf(nullptr), &valid);                                                 // valid tile
    add(2, copyOf(nullptr), nullptr);                                                // same location again
    add(3, copyOf([](dtMeshHeader& h) { h.magic ^= 0x01000000; }), nullptr);         // wrong magic
    add(4, copyOf([](dtMeshHeader& h) { h.version = DT_NAVMESH_VERSION - 1; }), nullptr); // wrong version
    add(5, copyOf([](dtMeshHeader& h) { h.polyCount = 1 << 23; }), nullptr);         // more polys than ref bits
    {
        dtStatus st = nav->removeTile(0, nullptr, nullptr);                          // null ref
        r.s.u32(6); r.s.u32(st); r.s.ref(0);
    }
    {
        dtStatus st0 = nav->removeTile(valid, nullptr, nullptr);
        dtStatus st = nav->removeTile(valid, nullptr, nullptr);                      // stale ref
        r.s.u32(7); r.s.u32(st0); r.s.u32(st);
    }
    {
        // Re-adding into the freed slot with the stale ref as lastRef restores it.
        dtTileRef ref = 0;
        unsigned char* data = copyOf(nullptr);
        dtStatus st = nav->addTile(data, (int)src.size(), DT_TILE_FREE_DATA, valid, &ref);
        if (dtStatusFailed(st)) dtFree(data);
        r.s.u32(8); r.s.u32(st); r.s.ref(ref);
    }
    dtFreeNavMesh(nav);
}

// Tile-cache obstacle script (case `tc_obstacle` lines), on the tile cache and
// navmesh of a tilecache-mode bake.
void recordObstacles(const Case& c, Baked& b) {
    if (!b.tc || c.obstacleOps.empty()) return;
    dtTileCache* tc = b.tc->cache;
    Record r("tilecache_obstacles");
    r.s.u32((uint32_t)c.obstacleOps.size());
    std::vector<dtObstacleRef> added;
    std::vector<dtCompressedTileRef> touched; // since the last update, in first-touch order
    auto touch = [&](const dtTileCacheObstacle* ob) {
        float bmin[3], bmax[3];
        tc->getObstacleBounds(ob, bmin, bmax);
        dtCompressedTileRef res[64];
        int n = 0;
        tc->queryTiles(bmin, bmax, res, &n, 64);
        for (int k = 0; k < n; ++k)
            if (std::find(touched.begin(), touched.end(), res[k]) == touched.end()) touched.push_back(res[k]);
        return n;
    };
    auto findObstacle = [&](dtObstacleRef ref) -> const dtTileCacheObstacle* {
        for (int i = 0; i < tc->getObstacleCount(); ++i) {
            const dtTileCacheObstacle* ob = tc->getObstacle(i);
            if (ob->state != DT_OBSTACLE_EMPTY && tc->getObstacleRef(ob) == ref) return ob;
        }
        return nullptr;
    };
    long long updates = 0;
    for (const ObstacleOp& op : c.obstacleOps) {
        r.s.u8((uint32_t)op.op);
        for (float a : op.args) r.s.f32(a);
        if (op.op >= 1 && op.op <= 3) {
            dtObstacleRef ref = 0;
            dtStatus st;
            const float* a = op.args.data();
            if (op.op == 1) st = tc->addObstacle(a, a[3], a[4], &ref);
            else if (op.op == 2) st = tc->addBoxObstacle(a, a + 3, &ref);
            else st = tc->addBoxObstacle(a, a + 3, a[6], &ref);
            added.push_back(ref);
            const dtTileCacheObstacle* ob = dtStatusSucceed(st) ? findObstacle(ref) : nullptr;
            r.s.u32(st); r.s.u32(ref); r.s.u32(ob ? (uint32_t)touch(ob) : 0u);
        } else if (op.op == 4) {
            const int k = (int)op.args[0];
            const dtObstacleRef ref = (k >= 0 && k < (int)added.size()) ? added[k] : 0;
            const dtTileCacheObstacle* ob = ref ? findObstacle(ref) : nullptr;
            const uint32_t n = ob ? (uint32_t)touch(ob) : 0u;
            dtStatus st = tc->removeObstacle(ref);
            r.s.u32(ref); r.s.u32(st); r.s.u32(n);
        } else {
            // update until the cache reports up to date (bounded), then the
            // obstacle slots and the resident navmesh tiles of every touched tile.
            bool upToDate = false;
            dtStatus st = DT_SUCCESS;
            uint32_t iters = 0;
            while (!upToDate && iters < 4096) {
                st = tc->update(1.0f / 30.0f, b.nav, &upToDate);
                ++iters;
                if (dtStatusFailed(st)) break;
            }
            ++updates;
            r.s.u32(iters); r.s.u32(st); r.s.u8(upToDate ? 1 : 0);
            uint32_t used = 0;
            for (int i = 0; i < tc->getObstacleCount(); ++i) used += tc->getObstacle(i)->state != DT_OBSTACLE_EMPTY ? 1 : 0;
            r.s.u32(used);
            for (int i = 0; i < tc->getObstacleCount(); ++i) {
                const dtTileCacheObstacle* ob = tc->getObstacle(i);
                if (ob->state == DT_OBSTACLE_EMPTY) continue;
                r.s.u32((uint32_t)i); r.s.u32(tc->getObstacleRef(ob)); r.s.u8(ob->state); r.s.u8(ob->type);
                r.s.u16(ob->salt);
            }
            r.s.u32((uint32_t)touched.size());
            for (dtCompressedTileRef tref : touched) {
                const dtCompressedTile* ct = tc->getTileByRef(tref);
                const int tx = ct ? ct->header->tx : -1, ty = ct ? ct->header->ty : -1, tl = ct ? ct->header->tlayer : -1;
                r.s.i32(tx); r.s.i32(ty); r.s.i32(tl);
                const dtMeshTile* mt = ct ? b.nav->getTileAt(tx, ty, tl) : nullptr;
                if (mt && mt->header) {
                    r.s.u8(1); r.s.u32((uint32_t)mt->dataSize); r.s.put(mt->data, (size_t)mt->dataSize);
                } else {
                    r.s.u8(0);
                }
            }
            touched.clear();
        }
    }
    r.counts["ops"] = (long long)c.obstacleOps.size();
    r.counts["updates"] = updates;
}

// ---------------------------------------------------------------------------
// Procedural meshes. Coordinates are exact binary fractions printed with %.9g,
// so the OBJ text round-trips to the same floats in any correct parser.

struct ObjOut {
    std::vector<float> v;
    std::vector<int> f;
    int vert(float x, float y, float z) { v.push_back(x); v.push_back(y); v.push_back(z); return (int)v.size() / 3 - 1; }
    void tri(int a, int b, int c) { f.push_back(a); f.push_back(b); f.push_back(c); }
    void quad(int a, int b, int c, int d) { f.push_back(a); f.push_back(b); f.push_back(c); f.push_back(a); f.push_back(c); f.push_back(d); }
    // Axis-aligned box; top face winds counter-clockwise seen from +y (walkable).
    void box(float x0, float y0, float z0, float x1, float y1, float z1) {
        int a = vert(x0, y0, z0), b = vert(x1, y0, z0), c = vert(x1, y0, z1), d = vert(x0, y0, z1);
        int e = vert(x0, y1, z0), f2 = vert(x1, y1, z0), g = vert(x1, y1, z1), h = vert(x0, y1, z1);
        quad(e, h, g, f2); quad(a, b, c, d); quad(a, e, f2, b); quad(b, f2, g, c); quad(c, g, h, d); quad(d, h, e, a);
    }
    // Ground quad at height y; counter-clockwise from +y.
    void floor(float x0, float z0, float x1, float z1, float y) {
        int a = vert(x0, y, z0), b = vert(x1, y, z0), c = vert(x1, y, z1), d = vert(x0, y, z1);
        quad(a, d, c, b);
    }
    bool write(const char* path) const {
        FILE* fp = fopen(path, "wb");
        if (!fp) return false;
        for (size_t i = 0; i < v.size(); i += 3) fprintf(fp, "v %.9g %.9g %.9g\n", v[i], v[i + 1], v[i + 2]);
        for (size_t i = 0; i < f.size(); i += 3) fprintf(fp, "f %d %d %d\n", f[i] + 1, f[i + 1] + 1, f[i + 2] + 1);
        fclose(fp);
        return true;
    }
};

// Integer hash noise in [0, 1): deterministic on every platform.
float hashNoise(int x, int z, uint32_t seed) {
    uint32_t h = (uint32_t)x * 0x8da6b343u ^ (uint32_t)z * 0xd8163841u ^ seed * 0xcb1ab31fu;
    h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
    return (float)(h >> 8) * (1.0f / 16777216.0f);
}

void gridTerrain(ObjOut& o, int n, float step, float amp, uint32_t seed, int octaveCell) {
    std::vector<int> idx((n + 1) * (n + 1));
    for (int z = 0; z <= n; ++z)
        for (int x = 0; x <= n; ++x) {
            // Bilinear value noise on a coarse lattice plus a little fine noise.
            const int cx = x / octaveCell, cz = z / octaveCell;
            const float fx = (float)(x % octaveCell) / octaveCell, fz = (float)(z % octaveCell) / octaveCell;
            const float h00 = hashNoise(cx, cz, seed), h10 = hashNoise(cx + 1, cz, seed);
            const float h01 = hashNoise(cx, cz + 1, seed), h11 = hashNoise(cx + 1, cz + 1, seed);
            const float hb = (h00 * (1 - fx) + h10 * fx) * (1 - fz) + (h01 * (1 - fx) + h11 * fx) * fz;
            const float y = amp * hb + 0.0625f * amp * hashNoise(x, z, seed + 1);
            idx[z * (n + 1) + x] = o.vert(x * step, y, z * step);
        }
    for (int z = 0; z < n; ++z)
        for (int x = 0; x < n; ++x) {
            int a = idx[z * (n + 1) + x], b = idx[z * (n + 1) + x + 1];
            int c = idx[(z + 1) * (n + 1) + x + 1], d = idx[(z + 1) * (n + 1) + x];
            o.quad(a, d, c, b);
        }
}

int generate(const char* kind, const char* path) {
    ObjOut o;
    std::string k(kind);
    if (k == "stairs") {
        // Three flights; risers 0.125, 0.25 (climbable at climb 0.9) and 1.0 (not).
        o.floor(0, 0, 40, 40, 0);
        const float risers[3] = {0.125f, 0.25f, 1.0f};
        for (int s = 0; s < 3; ++s)
            for (int i = 0; i < 8; ++i)
                o.box(4.0f + s * 12.0f, 0, 4.0f + i * 0.5f, 10.0f + s * 12.0f, risers[s] * (i + 1), 40.0f);
    } else if (k == "slopes") {
        // Ramps at 30, 44, 45, 46 and 60 degrees around the default 45 degree limit.
        o.floor(-5, -5, 75, 25, 0);
        // rise = 12 * tan(angle), written as literals so no libm call enters the corpus.
        const float rises[5] = {6.92820323f, 11.5883284f, 12.0f, 12.4264507f, 20.7846097f};
        for (int i = 0; i < 5; ++i) {
            const float x0 = i * 15.0f, x1 = x0 + 8.0f, run = 12.0f;
            const float rise = rises[i];
            int a = o.vert(x0, 0, 0), b = o.vert(x1, 0, 0), c = o.vert(x1, rise, run), d = o.vert(x0, rise, run);
            o.quad(a, d, c, b);
        }
    } else if (k == "multilevel") {
        // Two floors connected by a ramp, plus a bridge over the lower floor.
        o.floor(0, 0, 40, 40, 0);
        o.box(0, 4.0f, 20, 40, 4.25f, 40);
        int a = o.vert(30, 0, 4), b = o.vert(36, 0, 4), c = o.vert(36, 4.25f, 20), d = o.vert(30, 4.25f, 20);
        o.quad(a, d, c, b);
        o.box(8, 2.5f, 0, 12, 2.75f, 20);
    } else if (k == "corridors") {
        // Parallel corridors of width 0.75 .. 2.5 between 2 m walls; agent radius 0.6 needs > 1.2 m.
        o.floor(0, 0, 60, 30, 0);
        float x = 2.0f;
        const float widths[8] = {0.75f, 1.0f, 1.25f, 1.5f, 1.75f, 2.0f, 2.25f, 2.5f};
        for (int i = 0; i < 8; ++i) { o.box(x, 0, 2, x + 1.0f, 2.0f, 28); x += 1.0f + widths[i]; }
        o.box(x, 0, 2, x + 1.0f, 2.0f, 28);
    } else if (k == "islands") {
        // Disconnected platforms at varied heights; one pair is meant for an off-mesh link.
        for (int i = 0; i < 6; ++i) {
            const float x = (i % 3) * 14.0f, z = (i / 3) * 14.0f;
            o.box(x, 0, z, x + 8.0f, 0.5f + 0.5f * i, z + 8.0f);
        }
    } else if (k == "terrain_small") {
        gridTerrain(o, 128, 1.0f, 12.0f, 7, 16);
    } else if (k == "terrain_large") {
        gridTerrain(o, 1024, 1.0f, 40.0f, 11, 64);
    } else if (k == "seams") {
        // Walls and steps placed on exact tile boundaries (32 cells * 0.3 = 9.6 m).
        o.floor(0, 0, 38.4f, 38.4f, 0);
        for (int i = 1; i < 4; ++i) {
            const float s = i * 9.6f;
            o.box(s - 0.25f, 0, 4.0f, s + 0.25f, 2.0f, 12.0f);
            o.box(4.0f, 0, s - 0.15f, 12.0f, 0.3f, s + 0.15f);
            o.box(20.0f, 0, s, 28.0f, 0.5f, s + 2.0f);
        }
    } else if (k == "degenerate") {
        // A walkable floor carrying degenerate triangles: zero-area (repeated
        // vertex, repeated index, collinear), duplicated faces, slivers, faces
        // below one cell, downward-facing faces and a vertical wall.
        o.floor(0, 0, 24, 24, 0);
        o.floor(4, 4, 8, 8, 0.5f);
        const int dup = (int)o.f.size();
        const int d0 = o.f[dup - 6], d1 = o.f[dup - 5], d2 = o.f[dup - 4];
        o.tri(d0, d1, d2); // duplicated face
        for (int i = 0; i < 8; ++i) {
            const float x = 2.0f + 2.5f * i, y = 0.25f;
            int a = o.vert(x, y, 12.0f), b = o.vert(x, y, 12.0f), c = o.vert(x, y, 12.0f);
            o.tri(a, b, c);                                    // three equal vertices
            o.tri(a, a, o.vert(x + 1.0f, y, 13.0f));           // repeated index
            o.tri(o.vert(x, y, 14.0f), o.vert(x + 0.5f, y, 14.5f), o.vert(x + 1.0f, y, 15.0f)); // collinear
            o.tri(o.vert(x, y, 16.0f), o.vert(x + 2.0f, y, 16.0f + 0.0009765625f), o.vert(x + 1.0f, y, 16.0f)); // sliver
            o.tri(o.vert(x, y, 18.0f), o.vert(x + 0.0625f, y, 18.0f), o.vert(x, y, 18.0625f)); // below one cell
            o.tri(o.vert(x, 1.0f, 20.0f), o.vert(x + 1.5f, 1.0f, 20.0f), o.vert(x, 1.0f, 21.5f)); // facing down
        }
        o.box(10.0f, 0, 2.0f, 10.0f, 2.0f, 8.0f);              // zero-thickness wall
        int w0 = o.vert(14, 0, 2), w1 = o.vert(14, 2, 2), w2 = o.vert(14, 2, 8), w3 = o.vert(14, 0, 8);
        o.quad(w0, w1, w2, w3);                                // vertical quad
    } else {
        fprintf(stderr, "unknown kind %s (stairs slopes multilevel corridors islands terrain_small terrain_large seams degenerate)\n", kind);
        return 2;
    }
    if (!o.write(path)) { fprintf(stderr, "cannot write %s\n", path); return 1; }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: nav_oracle bake|bench <case> [--dump DIR] [--repeat N] [--threads T] | gen <kind> <out.obj>\n");
        return 2;
    }
    const std::string cmd = argv[1];
    if (cmd == "gen") return argc == 4 ? generate(argv[2], argv[3]) : 2;
    int repeat = 1, threads = 1, queryRepeat = 0;
    const char* recordsPath = nullptr;
    for (int i = 3; i < argc; ++i) {
        if (!strcmp(argv[i], "--dump") && i + 1 < argc) g_dumpDir = argv[++i];
        else if (!strcmp(argv[i], "--records") && i + 1 < argc) recordsPath = argv[++i];
        else if (!strcmp(argv[i], "--repeat") && i + 1 < argc) repeat = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--query-repeat") && i + 1 < argc) queryRepeat = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--threads") && i + 1 < argc) threads = atoi(argv[++i]);
        else { fprintf(stderr, "unknown option %s\n", argv[i]); return 2; }
    }
    Case c;
    if (!parseCase(argv[2], c)) return 2;
    Geometry g;
    if (!loadObj(c.mesh, g)) return 1;

    if (cmd == "bake") {
        if (recordsPath && !openRecordsFile(recordsPath, argv[2])) return 1;
        bool ok = true;
        {
            recordGeometry(g);
            Baked b;
            if (!bake(c, g, true, 1, b)) { fprintf(stderr, "bake failed\n"); ok = false; }
            if (ok) {
                recordNavmesh(b);
                dtQueryFilter plain, costs;
                makeCostFilter(costs);
                recordQueries(c, b.nav, plain, "");
                recordQueries(c, b.nav, costs, "filtered.");
                recordCrowd(c, b.nav, b, CROWD_DEFAULT, 0);
                for (int qlt = 0; qlt < 4; ++qlt) recordCrowd(c, b.nav, b, CROWD_QUALITY, qlt);
                recordCrowd(c, b.nav, b, CROWD_MOVE_VELOCITY, 0);
                recordCrowd(c, b.nav, b, CROWD_REMOVE_READD, 0);
                // The tile swap mutates its navmesh, so it runs on a copy.
                if (c.crowdAgents > 0 && c.crowdTicks > 0) {
                    dtNavMesh* copy = cloneNavMesh(b);
                    if (!copy) { fprintf(stderr, "navmesh copy failed\n"); ok = false; }
                    else { recordCrowd(c, copy, b, CROWD_TILE_SWAP, 0); dtFreeNavMesh(copy); }
                }
                recordMalformedTiles(b);
                recordObstacles(c, b); // last: it rebuilds tiles of the baked navmesh
            }
        }
        if (!closeRecordsFile()) { fprintf(stderr, "writing %s failed\n", recordsPath); return 1; }
        return ok ? 0 : 1;
    }
    if (cmd == "bench") {
        // Bakes `repeat` times; per-bake cost is the difference between two runs
        // with different repeat counts, so loading and start-up cancel out.
        std::vector<double> ms;
        for (int i = 0; i < repeat; ++i) {
            Baked b;
            auto t0 = std::chrono::steady_clock::now();
            if (!bake(c, g, false, threads, b)) { fprintf(stderr, "bake failed\n"); return 1; }
            auto t1 = std::chrono::steady_clock::now();
            ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
        }
        std::sort(ms.begin(), ms.end());
        printf("{\"bench\":\"bake\",\"repeat\":%d,\"threads\":%d,\"min_ms\":%.3f,\"median_ms\":%.3f}\n", repeat, threads,
               ms.empty() ? 0.0 : ms.front(), ms.empty() ? 0.0 : ms[ms.size() / 2]);
        if (queryRepeat > 0) {
            // One more bake, then the record query set and the crowd run, each
            // `queryRepeat` times with printing suppressed.
            Baked b;
            if (!bake(c, g, false, threads, b)) { fprintf(stderr, "bake failed\n"); return 1; }
            g_quiet = true;
            std::vector<double> qms, cms;
            for (int i = 0; i < queryRepeat; ++i) {
                auto t0 = std::chrono::steady_clock::now();
                dtQueryFilter plain;
                recordQueries(c, b.nav, plain, "");
                auto t1 = std::chrono::steady_clock::now();
                // NAV_BENCH_NO_CROWD=1 times the query set alone (compared with the port).
                if (!getenv("NAV_BENCH_NO_CROWD")) recordCrowd(c, b.nav, b, CROWD_DEFAULT, 0);
                auto t2 = std::chrono::steady_clock::now();
                qms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
                cms.push_back(std::chrono::duration<double, std::milli>(t2 - t1).count());
            }
            g_quiet = false;
            std::sort(qms.begin(), qms.end());
            std::sort(cms.begin(), cms.end());
            printf("{\"bench\":\"queries\",\"repeat\":%d,\"pairs\":%d,\"min_ms\":%.3f,\"median_ms\":%.3f}\n", queryRepeat,
                   c.queries, qms.front(), qms[qms.size() / 2]);
            if (c.crowdAgents > 0)
                printf("{\"bench\":\"crowd\",\"repeat\":%d,\"agents\":%d,\"ticks\":%d,\"min_ms\":%.3f,\"median_ms\":%.3f}\n",
                       queryRepeat, c.crowdAgents, c.crowdTicks, cms.front(), cms[cms.size() / 2]);
        }
        return 0;
    }
    fprintf(stderr, "unknown command %s\n", cmd.c_str());
    return 2;
}
