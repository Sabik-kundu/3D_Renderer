/*--------------
How does this 3D renderer works: 

  OBJ File
      │
      ▼
  Model Loader
      │
      ▼
 Vertex Buffer
      │
      ▼
 Model Transform
      │
      ▼
 View Transform
      │
      ▼
 Projection Transform
      │
      ▼
 Clipping
      │
      ▼
 Perspective Divide
      │
      ▼
 Viewport Transform
      │
      ▼
 Triangle Rasterization
      │
      ▼
 Depth Test (Z Buffer)
      │
      ▼
 Fragment Shading
      │
      ▼
 Framebuffer
      │
      ▼
     Screen


Currently the models don't have there original colours
It gets coloured in a sequence of colors.
----------------*/


#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <windowsx.h>
#include <shellapi.h>
#include <commdlg.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <vector>
#include <string>

struct Vec2 { float u, v; };
struct Vec3 { float x, y, z; };
struct Vec4 { float x, y, z, w; };
struct Mat4 { float m[16]; };

static Vec3 vec3(float x, float y, float z) { return { x, y, z }; }
static Vec3 vec3_add(Vec3 a, Vec3 b) { return vec3(a.x + b.x, a.y + b.y, a.z + b.z); }
static Vec3 vec3_sub(Vec3 a, Vec3 b) { return vec3(a.x - b.x, a.y - b.y, a.z - b.z); }
static Vec3 vec3_mul(Vec3 a, float s) { return vec3(a.x * s, a.y * s, a.z * s); }
static float vec3_dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static Vec3 vec3_cross(Vec3 a, Vec3 b) {
    return vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
static float vec3_len(Vec3 a) { return sqrtf(vec3_dot(a, a)); }
static Vec3 vec3_norm(Vec3 a) {
    float l = vec3_len(a);
    if (l < 1e-8f) return vec3(0, 0, 0);
    return vec3_mul(a, 1.0f / l);
}

static Mat4 mat4_identity() {
    Mat4 r; memset(r.m, 0, sizeof(r.m));
    r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
    return r;
}

static Mat4 mat4_mul(const Mat4 &a, const Mat4 &b) {
    Mat4 r;
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++) {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++) sum += a.m[row * 4 + k] * b.m[k * 4 + col];
            r.m[row * 4 + col] = sum;
        }
    return r;
}

static Vec4 mat4_mul_vec4(const Mat4 &a, Vec4 v) {
    Vec4 r;
    r.x = a.m[0] * v.x + a.m[1] * v.y + a.m[2] * v.z + a.m[3] * v.w;
    r.y = a.m[4] * v.x + a.m[5] * v.y + a.m[6] * v.z + a.m[7] * v.w;
    r.z = a.m[8] * v.x + a.m[9] * v.y + a.m[10] * v.z + a.m[11] * v.w;
    r.w = a.m[12] * v.x + a.m[13] * v.y + a.m[14] * v.z + a.m[15] * v.w;
    return r;
}

static Mat4 mat4_translate(Vec3 t) {
    Mat4 r = mat4_identity();
    r.m[3] = t.x; r.m[7] = t.y; r.m[11] = t.z;
    return r;
}

static Mat4 mat4_rotate_y(float a) {
    Mat4 r = mat4_identity();
    float c = cosf(a), s = sinf(a);
    r.m[0] = c;  r.m[2] = s;
    r.m[8] = -s; r.m[10] = c;
    return r;
}

static Mat4 mat4_perspective(float fovYRad, float aspect, float nearZ, float farZ) {
    Mat4 r; memset(r.m, 0, sizeof(r.m));
    float f = 1.0f / tanf(fovYRad * 0.5f);
    r.m[0] = f / aspect;
    r.m[5] = f;
    r.m[10] = (farZ + nearZ) / (nearZ - farZ);
    r.m[11] = (2.0f * farZ * nearZ) / (nearZ - farZ);
    r.m[14] = -1.0f;
    return r;
}

static Mat4 mat4_lookat(Vec3 eye, Vec3 target, Vec3 up) {
    Vec3 f = vec3_norm(vec3_sub(target, eye));
    Vec3 s = vec3_norm(vec3_cross(f, up));
    Vec3 u = vec3_cross(s, f);
    Mat4 r = mat4_identity();
    r.m[0] = s.x;  r.m[1] = s.y;  r.m[2] = s.z;  r.m[3] = -vec3_dot(s, eye);
    r.m[4] = u.x;  r.m[5] = u.y;  r.m[6] = u.z;  r.m[7] = -vec3_dot(u, eye);
    r.m[8] = -f.x; r.m[9] = -f.y; r.m[10] = -f.z; r.m[11] = vec3_dot(f, eye);
    r.m[12] = 0; r.m[13] = 0; r.m[14] = 0; r.m[15] = 1.0f;
    return r;
}

struct Vertex { Vec3 pos; Vec3 normal; Vec2 uv; };

struct Mesh {
    std::vector<Vertex> verts;
};

struct ModelInstance {
    Mesh mesh;
    Vec3 position;
    float yaw = 0.0f;
    std::wstring path;
};

static int resolve_index(int idx, int count) {
    if (idx > 0) return idx - 1;
    if (idx < 0) return count + idx;
    return -1;
}

static void parse_face_token(char *tok, int *pi, int *ti, int *ni) {
    *pi = *ti = *ni = 0;
    char *slash1 = strchr(tok, '/');
    if (!slash1) { *pi = atoi(tok); return; }
    *slash1 = 0;
    *pi = atoi(tok);
    char *afterFirst = slash1 + 1;
    char *slash2 = strchr(afterFirst, '/');
    if (!slash2) { *ti = atoi(afterFirst); return; }
    *slash2 = 0;
    if (slash2 != afterFirst) *ti = atoi(afterFirst);
    *ni = atoi(slash2 + 1);
}

static bool load_obj(const std::wstring &path, Mesh &mesh) {
    FILE *f = _wfopen(path.c_str(), L"r");
    if (!f) return false;

    std::vector<Vec3> positions;
    std::vector<Vec2> uvs;
    std::vector<Vec3> normals;

    char line[1024];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == 'v' && line[1] == ' ') {
            Vec3 p;
            sscanf(line + 2, "%f %f %f", &p.x, &p.y, &p.z);
            positions.push_back(p);
        } else if (line[0] == 'v' && line[1] == 't') {
            Vec2 t;
            sscanf(line + 3, "%f %f", &t.u, &t.v);
            uvs.push_back(t);
        } else if (line[0] == 'v' && line[1] == 'n') {
            Vec3 n;
            sscanf(line + 3, "%f %f %f", &n.x, &n.y, &n.z);
            normals.push_back(n);
        } else if (line[0] == 'f' && line[1] == ' ') {
            char *tokens[64];
            int tokenCount = 0;
            char *tok = strtok(line + 2, " \t\r\n");
            while (tok && tokenCount < 64) {
                tokens[tokenCount++] = tok;
                tok = strtok(NULL, " \t\r\n");
            }
            if (tokenCount < 3) continue;

            int pi[64], ti[64], ni[64];
            for (int i = 0; i < tokenCount; i++) {
                int rp, rt, rn;
                parse_face_token(tokens[i], &rp, &rt, &rn);
                pi[i] = resolve_index(rp, (int)positions.size());
                ti[i] = rt != 0 ? resolve_index(rt, (int)uvs.size()) : -1;
                ni[i] = rn != 0 ? resolve_index(rn, (int)normals.size()) : -1;
            }

            for (int i = 1; i + 1 < tokenCount; i++) {
                int idx[3] = { 0, i, i + 1 };
                Vertex triVerts[3];
                for (int k = 0; k < 3; k++) {
                    int j = idx[k];
                    Vertex v;
                    v.pos = positions[pi[j]];
                    v.uv = ti[j] >= 0 ? uvs[ti[j]] : Vec2{ 0, 0 };
                    v.normal = ni[j] >= 0 ? normals[ni[j]] : vec3(0, 0, 0);
                    triVerts[k] = v;
                }
                bool needsFaceNormal = ni[idx[0]] < 0 || ni[idx[1]] < 0 || ni[idx[2]] < 0;
                if (needsFaceNormal) {
                    Vec3 e1 = vec3_sub(triVerts[1].pos, triVerts[0].pos);
                    Vec3 e2 = vec3_sub(triVerts[2].pos, triVerts[0].pos);
                    Vec3 faceNormal = vec3_norm(vec3_cross(e1, e2));
                    for (int k = 0; k < 3; k++) triVerts[k].normal = faceNormal;
                }
                for (int k = 0; k < 3; k++) mesh.verts.push_back(triVerts[k]);
            }
        }
    }

    fclose(f);
    return !mesh.verts.empty();
}

static void make_fallback_cube(Mesh &mesh) {
    Vec3 p[8] = {
        vec3(-1,-1,-1), vec3(1,-1,-1), vec3(1,1,-1), vec3(-1,1,-1),
        vec3(-1,-1,1),  vec3(1,-1,1),  vec3(1,1,1),  vec3(-1,1,1)
    };
    int faces[6][4] = {
        {0,1,2,3}, {5,4,7,6}, {4,0,3,7}, {1,5,6,2}, {3,2,6,7}, {4,5,1,0}
    };
    for (int f = 0; f < 6; f++) {
        Vec3 a = p[faces[f][0]], b = p[faces[f][1]], c = p[faces[f][2]], d = p[faces[f][3]];
        Vec3 n = vec3_norm(vec3_cross(vec3_sub(b, a), vec3_sub(c, a)));
        Vertex va{ a, n, {0,0} }, vb{ b, n, {1,0} }, vc{ c, n, {1,1} }, vd{ d, n, {0,1} };
        mesh.verts.push_back(va); mesh.verts.push_back(vb); mesh.verts.push_back(vc);
        mesh.verts.push_back(va); mesh.verts.push_back(vc); mesh.verts.push_back(vd);
    }
}

static void normalize_mesh(Mesh &mesh) {
    if (mesh.verts.empty()) return;
    Vec3 minB = mesh.verts[0].pos, maxB = mesh.verts[0].pos;
    for (auto &v : mesh.verts) {
        if (v.pos.x < minB.x) minB.x = v.pos.x;
        if (v.pos.y < minB.y) minB.y = v.pos.y;
        if (v.pos.z < minB.z) minB.z = v.pos.z;
        if (v.pos.x > maxB.x) maxB.x = v.pos.x;
        if (v.pos.y > maxB.y) maxB.y = v.pos.y;
        if (v.pos.z > maxB.z) maxB.z = v.pos.z;
    }
    Vec3 center = vec3_mul(vec3_add(minB, maxB), 0.5f);
    float radius = 1e-6f;
    for (auto &v : mesh.verts) {
        float d = vec3_len(vec3_sub(v.pos, center));
        if (d > radius) radius = d;
    }
    float invRadius = 1.0f / radius;
    for (auto &v : mesh.verts) {
        v.pos = vec3_mul(vec3_sub(v.pos, center), invRadius);
    }
}

struct ClipVertex { Vec4 clip; Vec3 worldNormal; Vec2 uv; };

static ClipVertex lerp_clip_vertex(const ClipVertex &a, const ClipVertex &b, float t) {
    ClipVertex r;
    r.clip.x = a.clip.x + (b.clip.x - a.clip.x) * t;
    r.clip.y = a.clip.y + (b.clip.y - a.clip.y) * t;
    r.clip.z = a.clip.z + (b.clip.z - a.clip.z) * t;
    r.clip.w = a.clip.w + (b.clip.w - a.clip.w) * t;
    r.worldNormal = vec3_add(a.worldNormal, vec3_mul(vec3_sub(b.worldNormal, a.worldNormal), t));
    r.uv.u = a.uv.u + (b.uv.u - a.uv.u) * t;
    r.uv.v = a.uv.v + (b.uv.v - a.uv.v) * t;
    return r;
}

static float near_plane_dist(const ClipVertex &v) { return v.clip.z + v.clip.w; }

static int clip_near(ClipVertex *in, int inCount, ClipVertex *out) {
    int outCount = 0;
    for (int i = 0; i < inCount; i++) {
        ClipVertex cur = in[i];
        ClipVertex prev = in[(i - 1 + inCount) % inCount];
        float curD = near_plane_dist(cur);
        float prevD = near_plane_dist(prev);
        bool curIn = curD >= 0.0f;
        bool prevIn = prevD >= 0.0f;
        if (curIn != prevIn) {
            float t = prevD / (prevD - curD);
            out[outCount++] = lerp_clip_vertex(prev, cur, t);
        }
        if (curIn) out[outCount++] = cur;
    }
    return outCount;
}

struct Framebuffer {
    std::vector<uint32_t> pixels;
    std::vector<float> depth;
    int width = 0, height = 0;
};

static void framebuffer_resize(Framebuffer &fb, int w, int h) {
    fb.width = w; fb.height = h;
    fb.pixels.assign((size_t)w * h, 0);
    fb.depth.assign((size_t)w * h, 1e30f);
}

static void framebuffer_clear(Framebuffer &fb, uint32_t color) {
    std::fill(fb.pixels.begin(), fb.pixels.end(), color);
    std::fill(fb.depth.begin(), fb.depth.end(), 1e30f);
}

struct ScreenVertex { float x, y, invW; Vec3 normal; Vec2 uv; };

static ScreenVertex to_screen(const ClipVertex &v, int width, int height) {
    float invW = 1.0f / v.clip.w;
    float ndcX = v.clip.x * invW;
    float ndcY = v.clip.y * invW;
    ScreenVertex sv;
    sv.x = (ndcX * 0.5f + 0.5f) * width;
    sv.y = (1.0f - (ndcY * 0.5f + 0.5f)) * height;
    sv.invW = invW;
    sv.normal = vec3_mul(v.worldNormal, invW);
    sv.uv.u = v.uv.u * invW;
    sv.uv.v = v.uv.v * invW;
    return sv;
}

static float ndc_depth(Vec4 clip) { return (clip.z / clip.w) * 0.5f + 0.5f; }

static void rasterize_triangle(Framebuffer &fb, ScreenVertex a, ScreenVertex b, ScreenVertex c,
                                float depthA, float depthB, float depthC, Vec3 lightDir,
                                uint32_t baseColor, bool selected) {
    float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if (fabsf(area) < 1e-6f) return;
    if (area > 0.0f) return; // Backface culling

    int minX = (int)floorf(fminf(a.x, fminf(b.x, c.x)));
    int maxX = (int)ceilf(fmaxf(a.x, fmaxf(b.x, c.x)));
    int minY = (int)floorf(fminf(a.y, fminf(b.y, c.y)));
    int maxY = (int)ceilf(fmaxf(a.y, fmaxf(b.y, c.y)));
    if (minX < 0) minX = 0;
    if (minY < 0) minY = 0;
    if (maxX >= fb.width) maxX = fb.width - 1;
    if (maxY >= fb.height) maxY = fb.height - 1;

    float invArea = 1.0f / area;
    uint32_t baseR = (baseColor >> 16) & 0xFF;
    uint32_t baseG = (baseColor >> 8) & 0xFF;
    uint32_t baseB = baseColor & 0xFF;

    // ----- PERFORMANCE IMPROVEMENT: Incremental Edge Functions -----
    // Instead of doing expensive multiplications per pixel, we step our edges incrementally.
    float dw0dx = a.y - b.y, dw0dy = b.x - a.x;
    float dw1dx = b.y - c.y, dw1dy = c.x - b.x;
    float dw2dx = c.y - a.y, dw2dy = a.x - c.x;

    float px = minX + 0.5f, py = minY + 0.5f;
    float w0_row = (b.x - a.x) * (py - a.y) - (b.y - a.y) * (px - a.x);
    float w1_row = (c.x - b.x) * (py - b.y) - (c.y - b.y) * (px - b.x);
    float w2_row = (a.x - c.x) * (py - c.y) - (a.y - c.y) * (px - c.x);

    for (int y = minY; y <= maxY; y++) {
        float w0 = w0_row;
        float w1 = w1_row;
        float w2 = w2_row;

        for (int x = minX; x <= maxX; x++) {
            // Because area < 0, edges inside triangle must be <= 0
            if (w0 <= 0 && w1 <= 0 && w2 <= 0) {
                float l0 = w1 * invArea;
                float l1 = w2 * invArea;
                float l2 = w0 * invArea;

                float depth = l0 * depthA + l1 * depthB + l2 * depthC;
                int pixelIdx = y * fb.width + x;
                if (depth < fb.depth[pixelIdx]) {

                    float invW = l0 * a.invW + l1 * b.invW + l2 * c.invW;
                    float invWSafe = invW != 0.0f ? 1.0f / invW : 0.0f;

                    // Compute World Normal
                    Vec3 normal;
                    normal.x = (l0 * a.normal.x + l1 * b.normal.x + l2 * c.normal.x) * invWSafe;
                    normal.y = (l0 * a.normal.y + l1 * b.normal.y + l2 * c.normal.y) * invWSafe;
                    normal.z = (l0 * a.normal.z + l1 * b.normal.z + l2 * c.normal.z) * invWSafe;
                    normal = vec3_norm(normal);

                    // Compute true UVs
                    float u = (l0 * a.uv.u + l1 * b.uv.u + l2 * c.uv.u) * invWSafe;
                    float v = (l0 * a.uv.v + l1 * b.uv.v + l2 * c.uv.v) * invWSafe;

                    // ----- EXTENSIBILITY IMPROVEMENT: Texturing -----
                    // A simple procedural checkerboard pattern utilizing your UV data
                    int checker = ((int)floorf(u * 10.0f) + (int)floorf(v * 10.0f)) % 2;
                    float texColorMult = (checker == 0) ? 1.0f : 0.6f; 

                    float diffuse = vec3_dot(normal, lightDir);
                    if (diffuse < 0.0f) diffuse = 0.0f;
                    float ambient = selected ? 0.4f : 0.15f;
                    float shade = ambient + diffuse * (1.0f - ambient);
                    if (shade > 1.0f) shade = 1.0f;

                    // Mix lighting with texture
                    int channel = (int)(shade * texColorMult * 255.0f);
                    uint32_t r = (baseR * channel) / 255;
                    uint32_t g = (baseG * channel) / 255;
                    uint32_t bch = (baseB * channel) / 255;
                    uint32_t color = (r << 16) | (g << 8) | bch;

                    fb.depth[pixelIdx] = depth;
                    fb.pixels[pixelIdx] = color;
                }
            }
            // Step X incrementally
            w0 += dw0dx; 
            w1 += dw1dx; 
            w2 += dw2dx;
        }
        // Step Y incrementally
        w0_row += dw0dy;
        w1_row += dw1dy;
        w2_row += dw2dy;
    }
}

static const uint32_t kPalette[8] = {
    0x5AAAE6, 0xE68A5A, 0x8AE65A, 0xE65AC8, 0xE6D65A, 0x5AE6D6, 0xC85AE6, 0xE65A5A
};

static void put_pixel_depth(Framebuffer &fb, int x, int y, float depth, uint32_t color) {
    if (x < 0 || y < 0 || x >= fb.width || y >= fb.height) return;
    int idx = y * fb.width + x;
    if (depth >= fb.depth[idx]) return;
    fb.depth[idx] = depth;
    fb.pixels[idx] = color;
}

static void draw_line_3d(Framebuffer &fb, Vec3 worldA, Vec3 worldB, const Mat4 &vp, uint32_t color) {
    Vec4 a = mat4_mul_vec4(vp, { worldA.x, worldA.y, worldA.z, 1.0f });
    Vec4 b = mat4_mul_vec4(vp, { worldB.x, worldB.y, worldB.z, 1.0f });

    float da = a.z + a.w;
    float db = b.z + b.w;
    bool aIn = da >= 0.0f, bIn = db >= 0.0f;
    if (!aIn && !bIn) return;
    if (aIn != bIn) {
        float t = da / (da - db);
        Vec4 mid;
        mid.x = a.x + (b.x - a.x) * t;
        mid.y = a.y + (b.y - a.y) * t;
        mid.z = a.z + (b.z - a.z) * t;
        mid.w = a.w + (b.w - a.w) * t;
        if (!aIn) a = mid; else b = mid;
    }

    float invWa = 1.0f / a.w, invWb = 1.0f / b.w;
    float sx0 = (a.x * invWa * 0.5f + 0.5f) * fb.width;
    float sy0 = (1.0f - (a.y * invWa * 0.5f + 0.5f)) * fb.height;
    float d0 = a.z * invWa * 0.5f + 0.5f;
    float sx1 = (b.x * invWb * 0.5f + 0.5f) * fb.width;
    float sy1 = (1.0f - (b.y * invWb * 0.5f + 0.5f)) * fb.height;
    float d1 = b.z * invWb * 0.5f + 0.5f;

    int steps = (int)fmaxf(fabsf(sx1 - sx0), fabsf(sy1 - sy0));
    if (steps < 1) steps = 1;
    for (int i = 0; i <= steps; i++) {
        float t = (float)i / (float)steps;
        int x = (int)(sx0 + (sx1 - sx0) * t + 0.5f);
        int y = (int)(sy0 + (sy1 - sy0) * t + 0.5f);
        float depth = d0 + (d1 - d0) * t;
        put_pixel_depth(fb, x, y, depth, color);
    }
}

static void render_grid(Framebuffer &fb, const Mat4 &vp) {
    const int extent = 12;
    for (int i = -extent; i <= extent; i++) {
        uint32_t color;
        if (i == 0) color = 0x555570;
        else if (i % 5 == 0) color = 0x35354A;
        else color = 0x22222E;
        draw_line_3d(fb, vec3((float)i, 0, (float)-extent), vec3((float)i, 0, (float)extent), vp, color);
        draw_line_3d(fb, vec3((float)-extent, 0, (float)i), vec3((float)extent, 0, (float)i), vp, color);
    }
    draw_line_3d(fb, vec3(-(float)extent, 0, 0), vec3((float)extent, 0, 0), vp, 0x8A3A3A);
    draw_line_3d(fb, vec3(0, 0, -(float)extent), vec3(0, 0, (float)extent), vp, 0x3A5A8A);
}

static void render_model(Framebuffer &fb, const ModelInstance &model, const Mat4 &view, const Mat4 &proj,
                          Vec3 lightDir, uint32_t color, bool selected) {
    Mat4 modelMat = mat4_mul(mat4_translate(model.position), mat4_rotate_y(model.yaw));
    Mat4 mvp = mat4_mul(proj, mat4_mul(view, modelMat));

    for (size_t i = 0; i + 2 < model.mesh.verts.size(); i += 3) {
        ClipVertex cv[3];
        for (int k = 0; k < 3; k++) {
            const Vertex &v = model.mesh.verts[i + k];
            Vec4 clip = mat4_mul_vec4(mvp, { v.pos.x, v.pos.y, v.pos.z, 1.0f });
            Vec4 worldNormal4 = mat4_mul_vec4(modelMat, { v.normal.x, v.normal.y, v.normal.z, 0.0f });
            cv[k].clip = clip;
            cv[k].worldNormal = vec3_norm(vec3(worldNormal4.x, worldNormal4.y, worldNormal4.z));
            cv[k].uv = v.uv;
        }

        ClipVertex clipped[8];
        int clippedCount = clip_near(cv, 3, clipped);
        if (clippedCount < 3) continue;

        for (int t = 1; t + 1 < clippedCount; t++) {
            ClipVertex &pa = clipped[0];
            ClipVertex &pb = clipped[t];
            ClipVertex &pc = clipped[t + 1];

            ScreenVertex sa = to_screen(pa, fb.width, fb.height);
            ScreenVertex sb = to_screen(pb, fb.width, fb.height);
            ScreenVertex sc = to_screen(pc, fb.width, fb.height);

            float da = ndc_depth(pa.clip);
            float db = ndc_depth(pb.clip);
            float dc = ndc_depth(pc.clip);

            rasterize_triangle(fb, sa, sb, sc, da, db, dc, lightDir, color, selected);
        }
    }
}

struct OrbitCamera {
    float yaw = 0.6f;
    float pitch = 0.35f;
    float distance = 8.0f;
    Vec3 target = vec3(0, 0, 0);
};

static Vec3 orbit_eye(const OrbitCamera &cam) {
    Vec3 dir = vec3(cosf(cam.pitch) * sinf(cam.yaw), sinf(cam.pitch), cosf(cam.pitch) * cosf(cam.yaw));
    return vec3_add(cam.target, vec3_mul(dir, cam.distance));
}

static std::vector<ModelInstance> g_models;
static OrbitCamera g_cam;
static Framebuffer g_fb;
static BITMAPINFO g_bmi;
static bool g_running = true;
static bool g_orbiting = false;
static bool g_panning = false;
static int g_lastMouseX = 0, g_lastMouseY = 0;
static HWND g_hwnd;
static int g_selected = 0;
static float g_lightAzimuth = 0.f;
static float g_lightElevation = 0.f;
static bool g_showGrid = true;
static HFONT g_uiFont;

static Vec3 light_direction() {
    return vec3_norm(vec3(cosf(g_lightElevation) * sinf(g_lightAzimuth),
                           sinf(g_lightElevation),
                           cosf(g_lightElevation) * cosf(g_lightAzimuth)));
}

static COLORREF palette_to_colorref(uint32_t c) {
    return RGB((c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF);
}

static std::wstring filename_only(const std::wstring &path) {
    size_t pos = path.find_last_of(L"\\/");
    return pos == std::wstring::npos ? path : path.substr(pos + 1);
}

static void clamp_selection() {
    int count = (int)g_models.size();
    if (g_selected >= count) g_selected = count - 1;
    if (g_selected < 0 && count > 0) g_selected = 0;
}

static Vec3 next_spawn_position() {
    float spacing = 3.0f;
    return vec3((float)g_models.size() * spacing, 0, 0);
}

static void update_title() {
    wchar_t buf[256];
    swprintf(buf, 256, L"Software OBJ Viewer  |  %d model(s)  |  O: open  drag&drop files  LMB: orbit  RMB: pan  wheel: zoom  C: clear  Backspace: remove last",
             (int)g_models.size());
    SetWindowTextW(g_hwnd, buf);
}

static void add_model_from_path(const std::wstring &path) {
    ModelInstance model;
    model.path = path;
    if (!load_obj(path, model.mesh)) return;
    normalize_mesh(model.mesh);
    model.position = next_spawn_position();
    g_models.push_back(std::move(model));
    g_selected = (int)g_models.size() - 1;
    update_title();
}

static void open_file_dialog(HWND hwnd) {
    wchar_t fileBuf[16384];
    memset(fileBuf, 0, sizeof(fileBuf));

    OPENFILENAMEW ofn;
    memset(&ofn, 0, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = hwnd;
    ofn.lpstrFilter = L"OBJ Files (*.obj)\0*.obj\0All Files\0*.*\0";
    ofn.lpstrFile = fileBuf;
    ofn.nMaxFile = 16384;
    ofn.Flags = OFN_ALLOWMULTISELECT | OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;

    if (!GetOpenFileNameW(&ofn)) return;

    wchar_t *p = fileBuf;
    std::wstring dirOrFirst = p;
    p += dirOrFirst.size() + 1;

    if (*p == 0) {
        add_model_from_path(dirOrFirst);
    } else {
        while (*p) {
            std::wstring name = p;
            p += name.size() + 1;
            std::wstring full = dirOrFirst + L"\\" + name;
            add_model_from_path(full);
        }
    }
}

static void draw_ui(HDC hdc, int width, int height) {
    (void)height;
    HFONT oldFont = (HFONT)SelectObject(hdc, g_uiFont);
    int prevMode = SetBkMode(hdc, TRANSPARENT);

    int panelW = 300;
    int panelH = 40 + (int)g_models.size() * 20 + 112;
    RECT panelRect = { 10, 10, 10 + panelW, 10 + panelH };
    HBRUSH panelBrush = CreateSolidBrush(RGB(12, 12, 18));
    FillRect(hdc, &panelRect, panelBrush);
    DeleteObject(panelBrush);

    int y = 18;
    SetTextColor(hdc, RGB(230, 230, 230));
    TextOutW(hdc, 18, y, L"Models  (Tab / Shift+Tab to select)", 36);
    y += 22;

    for (int i = 0; i < (int)g_models.size(); i++) {
        uint32_t pc = kPalette[i % 8];
        RECT swatch = { 18, y + 3, 30, y + 15 };
        HBRUSH swatchBrush = CreateSolidBrush(palette_to_colorref(pc));
        FillRect(hdc, &swatch, swatchBrush);
        DeleteObject(swatchBrush);

        std::wstring name = filename_only(g_models[i].path);
        if (name.empty()) name = L"cube";
        std::wstring line = (i == g_selected ? L"> " : L"  ") + name;
        SetTextColor(hdc, i == g_selected ? RGB(255, 230, 140) : RGB(210, 210, 210));
        TextOutW(hdc, 38, y, line.c_str(), (int)line.size());
        y += 20;
    }

    y += 6;
    wchar_t buf[160];
    if (g_selected >= 0 && g_selected < (int)g_models.size()) {
        Vec3 p = g_models[g_selected].position;
        float yawDeg = g_models[g_selected].yaw * 180.0f / 3.14159265f;
        swprintf(buf, 160, L"Pos  %.2f, %.2f, %.2f   Yaw %.0f deg", p.x, p.y, p.z, yawDeg);
        SetTextColor(hdc, RGB(200, 220, 255));
        TextOutW(hdc, 18, y, buf, (int)wcslen(buf));
        y += 20;
    }

    swprintf(buf, 160, L"Light  Azimuth %.0f deg   Elevation %.0f deg",
             g_lightAzimuth * 180.0f / 3.14159265f, g_lightElevation * 180.0f / 3.14159265f);
    SetTextColor(hdc, RGB(255, 220, 140));
    TextOutW(hdc, 18, y, buf, (int)wcslen(buf));
    y += 20;

    swprintf(buf, 160, L"Grid: %s  (V to toggle)", g_showGrid ? L"ON" : L"OFF");
    SetTextColor(hdc, RGB(160, 200, 160));
    TextOutW(hdc, 18, y, buf, (int)wcslen(buf));
    y += 26;

    SetTextColor(hdc, RGB(140, 140, 150));
    const wchar_t *hints[] = {
        L"WASD/QE move   Z/X rotate   G reset",
        L"Arrows: light direction",
        L"O open   drag&drop files   C clear   Bksp remove",
        L"LMB orbit   RMB pan   wheel zoom   R reset cam"
    };
    for (int i = 0; i < 4; i++) {
        TextOutW(hdc, 18, y, hints[i], (int)wcslen(hints[i]));
        y += 18;
    }

    int cx = width - 90, cy = 90, radius = 55;
    HPEN circlePen = CreatePen(PS_SOLID, 1, RGB(90, 90, 100));
    HPEN oldPen = (HPEN)SelectObject(hdc, circlePen);
    HBRUSH oldBrush = (HBRUSH)SelectObject(hdc, GetStockObject(NULL_BRUSH));
    Ellipse(hdc, cx - radius, cy - radius, cx + radius, cy + radius);

    float horiz = cosf(g_lightElevation);
    int tx = cx + (int)(sinf(g_lightAzimuth) * radius * horiz);
    int ty = cy - (int)(cosf(g_lightAzimuth) * radius * horiz);
    HPEN raypen = CreatePen(PS_SOLID, 2, RGB(255, 210, 90));
    SelectObject(hdc, raypen);
    MoveToEx(hdc, cx, cy, NULL);
    LineTo(hdc, tx, ty);
    HBRUSH sunBrush = CreateSolidBrush(RGB(255, 220, 110));
    SelectObject(hdc, sunBrush);
    Ellipse(hdc, tx - 5, ty - 5, tx + 5, ty + 5);
    SetTextColor(hdc, RGB(140, 140, 150));
    TextOutW(hdc, cx - 40, cy + radius + 6, L"light dir", 9);

    SelectObject(hdc, oldPen);
    SelectObject(hdc, oldBrush);
    DeleteObject(circlePen);
    DeleteObject(raypen);
    DeleteObject(sunBrush);

    SetBkMode(hdc, prevMode);
    SelectObject(hdc, oldFont);
}

LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
        case WM_DESTROY:
            g_running = false;
            PostQuitMessage(0);
            return 0;
        case WM_KEYDOWN:
            if (wp == VK_ESCAPE) { g_running = false; PostQuitMessage(0); }
            else if (wp == 'O') open_file_dialog(hwnd);
            else if (wp == 'C') { g_models.clear(); g_selected = -1; update_title(); }
            else if (wp == VK_BACK) { if (!g_models.empty()) { g_models.pop_back(); clamp_selection(); update_title(); } }
            else if (wp == 'R') { g_cam = OrbitCamera{}; }
            else if (wp == 'V') { g_showGrid = !g_showGrid; }
            else if (wp == VK_TAB) {
                if (!g_models.empty()) {
                    bool shiftHeld = (GetKeyState(VK_SHIFT) & 0x8000) != 0;
                    int count = (int)g_models.size();
                    g_selected = ((g_selected + (shiftHeld ? -1 : 1)) % count + count) % count;
                }
            }
            else if (wp == 'G') {
                if (g_selected >= 0 && g_selected < (int)g_models.size()) {
                    g_models[g_selected].position = vec3(0, 0, 0);
                    g_models[g_selected].yaw = 0.0f;
                }
            }
            return 0;
        case WM_LBUTTONDOWN:
            g_orbiting = true;
            g_lastMouseX = GET_X_LPARAM(lp);
            g_lastMouseY = GET_Y_LPARAM(lp);
            SetCapture(hwnd);
            return 0;
        case WM_LBUTTONUP:
            g_orbiting = false;
            ReleaseCapture();
            return 0;
        case WM_RBUTTONDOWN:
            g_panning = true;
            g_lastMouseX = GET_X_LPARAM(lp);
            g_lastMouseY = GET_Y_LPARAM(lp);
            SetCapture(hwnd);
            return 0;
        case WM_RBUTTONUP:
            g_panning = false;
            ReleaseCapture();
            return 0;
        case WM_MOUSEMOVE: {
            int x = GET_X_LPARAM(lp), y = GET_Y_LPARAM(lp);
            int dx = x - g_lastMouseX, dy = y - g_lastMouseY;
            g_lastMouseX = x; g_lastMouseY = y;
            if (g_orbiting) {
                g_cam.yaw += dx * 0.006f;
                g_cam.pitch += -dy * 0.006f;
                if (g_cam.pitch > 1.5f) g_cam.pitch = 1.5f;
                if (g_cam.pitch < -1.5f) g_cam.pitch = -1.5f;
            }
            if (g_panning) {
                Vec3 eye = orbit_eye(g_cam);
                Vec3 forward = vec3_norm(vec3_sub(g_cam.target, eye));
                Vec3 right = vec3_norm(vec3_cross(forward, vec3(0, 1, 0)));
                Vec3 up = vec3_cross(right, forward);
                float panScale = g_cam.distance * 0.0018f;
                g_cam.target = vec3_sub(g_cam.target, vec3_mul(right, dx * panScale));
                g_cam.target = vec3_add(g_cam.target, vec3_mul(up, dy * panScale));
            }
            return 0;
        }
        case WM_MOUSEWHEEL: {
            float delta = (float)GET_WHEEL_DELTA_WPARAM(wp);
            g_cam.distance *= powf(0.9f, delta / 120.0f);
            if (g_cam.distance < 0.5f) g_cam.distance = 0.5f;
            if (g_cam.distance > 500.0f) g_cam.distance = 500.0f;
            return 0;
        }
        case WM_DROPFILES: {
            HDROP hDrop = (HDROP)wp;
            UINT count = DragQueryFileW(hDrop, 0xFFFFFFFF, NULL, 0);
            for (UINT i = 0; i < count; i++) {
                wchar_t buf[MAX_PATH];
                DragQueryFileW(hDrop, i, buf, MAX_PATH);
                add_model_from_path(buf);
            }
            DragFinish(hDrop);
            return 0;
        }
        case WM_SIZE: {
            int w = LOWORD(lp), h = HIWORD(lp);
            if (w > 0 && h > 0) {
                framebuffer_resize(g_fb, w, h);
                memset(&g_bmi, 0, sizeof(g_bmi));
                g_bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
                g_bmi.bmiHeader.biWidth = w;
                g_bmi.bmiHeader.biHeight = -h;
                g_bmi.bmiHeader.biPlanes = 1;
                g_bmi.bmiHeader.biBitCount = 32;
                g_bmi.bmiHeader.biCompression = BI_RGB;
            }
            return 0;
        }
    }
    return DefWindowProc(hwnd, msg, wp, lp);
}

int WINAPI WinMain(HINSTANCE hInst, HINSTANCE hPrev, LPSTR cmdLine, int showCmd) {
    (void)hPrev; (void)cmdLine;

    int width = 1280, height = 800;

    WNDCLASSW wc = {};
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInst;
    wc.lpszClassName = L"ObjViewerWndClass";
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    RegisterClassW(&wc);

    RECT rect = { 0, 0, width, height };
    AdjustWindowRect(&rect, WS_OVERLAPPEDWINDOW, FALSE);

    g_hwnd = CreateWindowW(
        wc.lpszClassName, L"Software OBJ Viewer",
        WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT,
        rect.right - rect.left, rect.bottom - rect.top,
        NULL, NULL, hInst, NULL
    );
    ShowWindow(g_hwnd, showCmd);
    DragAcceptFiles(g_hwnd, TRUE);

    framebuffer_resize(g_fb, width, height);
    memset(&g_bmi, 0, sizeof(g_bmi));
    g_bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    g_bmi.bmiHeader.biWidth = width;
    g_bmi.bmiHeader.biHeight = -height;
    g_bmi.bmiHeader.biPlanes = 1;
    g_bmi.bmiHeader.biBitCount = 32;
    g_bmi.bmiHeader.biCompression = BI_RGB;

    int argc = 0;
    LPWSTR *argv = CommandLineToArgvW(GetCommandLineW(), &argc);
    if (argv) {
        for (int i = 1; i < argc; i++) add_model_from_path(argv[i]);
        LocalFree(argv);
    }

    if (g_models.empty()) {
        ModelInstance fallback;
        make_fallback_cube(fallback.mesh);
        normalize_mesh(fallback.mesh);
        fallback.position = vec3(0, 0, 0);
        g_models.push_back(std::move(fallback));
        g_selected = 0;
    }

    Vec3 initLight = vec3_norm(vec3(0.5f, 0.8f, 0.4f));
    g_lightElevation = asinf(initLight.y);
    g_lightAzimuth = atan2f(initLight.x, initLight.z);

    g_uiFont = CreateFontW(16, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
                            OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY,
                            DEFAULT_PITCH | FF_DONTCARE, L"Consolas");

    update_title();

    LARGE_INTEGER freq, prevTime;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&prevTime);

    HDC hdc = GetDC(g_hwnd);

    while (g_running) {
        MSG msg;
        while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) g_running = false;
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        if (!g_running) break;

        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);
        float dt = (float)(now.QuadPart - prevTime.QuadPart) / (float)freq.QuadPart;
        prevTime = now;
        if (dt > 0.1f) dt = 0.1f;

        if (g_fb.width <= 0 || g_fb.height <= 0) { Sleep(10); continue; }

        Vec3 eye = orbit_eye(g_cam);

        bool focused = GetForegroundWindow() == g_hwnd;
        if (focused) {
            float moveSpeed = 2.5f, rotSpeed = 1.6f, lightSpeed = 1.2f;
            
            // 'A' when pressed it is made sure the selected object move right! 
            Vec3 camForward = vec3_norm(vec3_sub(g_cam.target, eye));
            Vec3 camRight = vec3_norm(vec3_cross(camForward, vec3(0, 1, 0)));
            camForward.y = 0; camForward = vec3_norm(camForward); // up/down movement is made sure!
            camRight.y = 0; camRight = vec3_norm(camRight);
            
            if (g_selected >= 0 && g_selected < (int)g_models.size()) {
                ModelInstance &sel = g_models[g_selected];
                if (GetAsyncKeyState('W') & 0x8000) sel.position = vec3_add(sel.position, vec3_mul(camForward, moveSpeed * dt));
                if (GetAsyncKeyState('S') & 0x8000) sel.position = vec3_sub(sel.position, vec3_mul(camForward, moveSpeed * dt));
                if (GetAsyncKeyState('A') & 0x8000) sel.position = vec3_sub(sel.position, vec3_mul(camRight, moveSpeed * dt)); // Move Left
                if (GetAsyncKeyState('D') & 0x8000) sel.position = vec3_add(sel.position, vec3_mul(camRight, moveSpeed * dt)); // Move Right
                if (GetAsyncKeyState('Q') & 0x8000) sel.position.y -= moveSpeed * dt;
                if (GetAsyncKeyState('E') & 0x8000) sel.position.y += moveSpeed * dt;
                if (GetAsyncKeyState('Z') & 0x8000) sel.yaw -= rotSpeed * dt;
                if (GetAsyncKeyState('X') & 0x8000) sel.yaw += rotSpeed * dt;
            }
            if (GetAsyncKeyState(VK_LEFT) & 0x8000) g_lightAzimuth -= lightSpeed * dt;
            if (GetAsyncKeyState(VK_RIGHT) & 0x8000) g_lightAzimuth += lightSpeed * dt;
            if (GetAsyncKeyState(VK_UP) & 0x8000) g_lightElevation += lightSpeed * dt;
            if (GetAsyncKeyState(VK_DOWN) & 0x8000) g_lightElevation -= lightSpeed * dt;
            if (g_lightElevation > 1.55f) g_lightElevation = 1.55f;
            if (g_lightElevation < -1.55f) g_lightElevation = -1.55f;
        }

        Mat4 view = mat4_lookat(eye, g_cam.target, vec3(0, 1, 0));
        Mat4 proj = mat4_perspective(60.0f * 3.14159265f / 180.0f,
                                      (float)g_fb.width / (float)g_fb.height, 0.05f, 500.0f);
        Vec3 lightDir = light_direction();

        framebuffer_clear(g_fb, 0x101018);
        if (g_showGrid) {
            Mat4 vp = mat4_mul(proj, view);
            render_grid(g_fb, vp);
        }
        for (size_t i = 0; i < g_models.size(); i++) {
            uint32_t color = kPalette[i % 8];
            render_model(g_fb, g_models[i], view, proj, lightDir, color, (int)i == g_selected);
        }

        StretchDIBits(hdc, 0, 0, g_fb.width, g_fb.height, 0, 0, g_fb.width, g_fb.height,
                      g_fb.pixels.data(), &g_bmi, DIB_RGB_COLORS, SRCCOPY);

        draw_ui(hdc, g_fb.width, g_fb.height);

        Sleep(1);
    }

    ReleaseDC(g_hwnd, hdc);
    DeleteObject(g_uiFont);
    return 0;
}
