/* tag_detector.cpp - 板端 AprilTag 36h11 检测
 *
 * 简化版检测器, 针对"单标签、近正对、可控补光"的纠偏场景:
 *  1) 积分图自适应阈值二值化 (暗=1)
 *  2) 8 邻域连通域 + 矩统计, 面积/长宽比预筛
 *  3) 主轴对角方向极值定位 4 角点, 边缘射线采样 + 直线拟合求亚像素角点
 *  4) 4 点单应 -> 环形黑白电平平面拟合 -> 采样 36 数据位 -> 码表汉明匹配
 * 内存: 积分图/二值/标签/并查集约 0.7MB, 优先 PSRAM 分配, 只分配一次复用。
 */
#include <Arduino.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "esp_heap_caps.h"
#include "config.h"
#include "tag_detector.h"
#include "tag36h11_table.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DET_MAX_COMPS     4096   /* 连通域统计上限 (噪声场景碎域多, 需留余量) */
#define DET_MAX_CANDS     8      /* 最多尝试解码的候选数 */

typedef struct {
    int32_t area;
    int16_t minx, maxx, miny, maxy;
    int64_t sx, sy, sxx, syy, sxy;
} CompStat;

/* 候选连通域: 4096 个 x 8B = 32KB, 必须放 PSRAM, 不能放栈上 */

/* ================= 工作区 (按尺寸惰性分配复用) ================= */
typedef struct {
    int      w, h;
    uint32_t *ii;      /* (w+1)*(h+1) 积分图 */
    uint8_t  *bin;     /* w*h 二值 */
    uint16_t *lab;     /* w*h 连通域标签 */
    uint16_t *parent;  /* 并查集 */
    CompStat *stats;   /* 暗域统计 */
    uint16_t (*wbbox)[4]; /* 白域包围盒 (填洞用) */
    Cand     *cands;   /* 候选数组 (PSRAM) */
} Scratch;

static Scratch g_sc;
static int g_ncomps = 0;

static void *psram_malloc(size_t n) {
    void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) p = malloc(n);
    return p;
}

static bool scratch_ensure(int w, int h) {
    if (g_sc.w == w && g_sc.h == h) return true;
    free(g_sc.ii); free(g_sc.bin); free(g_sc.lab); free(g_sc.parent);
    free(g_sc.stats); free(g_sc.wbbox); free(g_sc.cands);
    memset(&g_sc, 0, sizeof(g_sc));
    g_sc.ii     = (uint32_t *)psram_malloc((size_t)(w + 1) * (h + 1) * 4);
    g_sc.bin    = (uint8_t *)psram_malloc((size_t)w * h);
    g_sc.lab    = (uint16_t *)psram_malloc((size_t)w * h * 2);
    g_sc.parent = (uint16_t *)psram_malloc((size_t)w * h * 2);
    g_sc.stats  = (CompStat *)psram_malloc((size_t)DET_MAX_COMPS * sizeof(CompStat));
    g_sc.wbbox  = (uint16_t (*)[4])psram_malloc((size_t)DET_MAX_COMPS * 4 * 2);
    g_sc.cands  = (Cand *)psram_malloc((size_t)DET_MAX_COMPS * sizeof(Cand));
    if (!g_sc.ii || !g_sc.bin || !g_sc.lab || !g_sc.parent ||
        !g_sc.stats || !g_sc.wbbox || !g_sc.cands) {
        free(g_sc.ii); free(g_sc.bin); free(g_sc.lab); free(g_sc.parent);
        free(g_sc.stats); free(g_sc.wbbox); free(g_sc.cands);
        memset(&g_sc, 0, sizeof(g_sc));
        return false;
    }
    memset(g_sc.parent, 0, (size_t)w * h * 2);   /* 保证 label 0 的 parent 也是 0 */
    g_sc.w = w; g_sc.h = h;
    return true;
}

/* ================= 双线性灰度采样 (越界返回 -1) ================= */
static float samp_bilinear(const uint8_t *im, int w, int h, int stride, float px, float py) {
    if (px < 0 || py < 0 || px > w - 1 || py > h - 1) return -1.0f;
    int x0 = (int)px, y0 = (int)py;
    int x1 = x0 + 1 < w ? x0 + 1 : x0;
    int y1 = y0 + 1 < h ? y0 + 1 : y0;
    float fx = px - x0, fy = py - y0;
    float g00 = im[y0 * stride + x0], g10 = im[y0 * stride + x1];
    float g01 = im[y1 * stride + x0], g11 = im[y1 * stride + x1];
    return g00 * (1 - fx) * (1 - fy) + g10 * fx * (1 - fy)
         + g01 * (1 - fx) * fy       + g11 * fx * fy;
}

/* ================= 积分图 + 自适应阈值 ================= */
static void threshold_adaptive(const uint8_t *im, int w, int h, int stride) {
    const int win = DET_ADAPT_WIN, half = win / 2;
    uint32_t *ii = g_sc.ii;
    memset(ii, 0, (size_t)(w + 1) * 4);
    for (int y = 0; y < h; y++) {
        const uint8_t *row = im + (size_t)y * stride;
        uint32_t *iirow = ii + (size_t)(y + 1) * (w + 1);
        uint32_t *iiprev = ii + (size_t)y * (w + 1);
        uint32_t run = 0;
        for (int x = 0; x < w; x++) {
            run += row[x];
            iirow[x + 1] = iiprev[x + 1] + run;
        }
    }
    for (int y = 0; y < h; y++) {
        int y0 = y - half, y1 = y + half;
        if (y0 < 0) y0 = 0;
        if (y1 >= h) y1 = h - 1;
        uint8_t *brow = g_sc.bin + (size_t)y * w;
        const uint8_t *grow = im + (size_t)y * stride;
        for (int x = 0; x < w; x++) {
            int x0 = x - half, x1 = x + half;
            if (x0 < 0) x0 = 0;
            if (x1 >= w) x1 = w - 1;
            uint32_t area = (uint32_t)(x1 - x0 + 1) * (y1 - y0 + 1);
            uint32_t sum = ii[(size_t)(y1 + 1) * (w + 1) + (x1 + 1)]
                         - ii[(size_t)(y0) * (w + 1) + (x1 + 1)]
                         - ii[(size_t)(y1 + 1) * (w + 1) + (x0)]
                         + ii[(size_t)(y0) * (w + 1) + (x0)];
            float mean = (float)sum / area;
            brow[x] = (grow[x] < mean - DET_ADAPT_BIAS) ? 1 : 0;
        }
    }
}

/* ================= 并查集 ================= */
static uint16_t uf_find(uint16_t *p, uint16_t a) {
    while (p[a] != a) { p[a] = p[p[a]]; a = p[a]; }
    return a;
}

/* ================= 封闭白区填洞 =================
 * 自适应阈值窗口(15px)小于标签特征厚度(黑框边长/8*1, edge>=120px 时 >15px)时,
 * 大片黑色的内部局部均值≈自身, 不会被判暗 -> 黑框被掏空成细带, 连通域断裂。
 * 处理: 对"白"做连通域标记, 不接触图像边界的白区一律判为洞 -> 填黑。
 * 效果: 标签变成实心黑方块 (对角点提取更稳), 解码仍直接采灰度原图, 不受影响。 */
static void fill_holes(int w, int h) {
    uint16_t *lab = g_sc.lab, *parent = g_sc.parent;
    uint16_t (*wbbox)[4] = g_sc.wbbox;
    uint16_t next = 1;

    for (int y = 0; y < h; y++) {
        uint8_t *brow = g_sc.bin + (size_t)y * w;
        uint16_t *lrow = lab + (size_t)y * w;
        uint16_t *lup = (y > 0) ? lab + (size_t)(y - 1) * w : NULL;
        for (int x = 0; x < w; x++) {
            if (brow[x]) { lrow[x] = 0; continue; }   /* 黑色不属于白区 */
            uint16_t m = 0;
            /* 白区用 4 邻域: 暗格网的对角搭桥不漏, 封闭性才成立 */
            if (x > 0 && lrow[x - 1]) m = lrow[x - 1];
            if (y > 0 && lup[x] && (!m || lup[x] < m)) m = lup[x];
            if (!m) {
                if (next >= DET_MAX_COMPS) { lrow[x] = 0; continue; }
                m = next++;
                parent[m] = m;
                wbbox[m][0] = wbbox[m][1] = x;
                wbbox[m][2] = wbbox[m][3] = y;
            } else {
                m = uf_find(parent, m);
            }
            lrow[x] = m;
            if (x > 0 && lrow[x - 1] && lrow[x - 1] != m) {
                uint16_t a = uf_find(parent, lrow[x - 1]);
                if (a != m) parent[a] = m;
            }
            if (y > 0 && lup[x] && lup[x] != m) {
                uint16_t a = uf_find(parent, lup[x]);
                if (a != m) parent[a] = m;
            }
        }
    }

    /* 不触边的白区 -> 填黑 (洞) */
    for (int y = 0; y < h; y++) {
        uint8_t *brow = g_sc.bin + (size_t)y * w;
        uint16_t *lrow = lab + (size_t)y * w;
        for (int x = 0; x < w; x++) {
            if (brow[x]) continue;
            uint16_t l = lrow[x];
            if (l == 0) continue;                    /* 未标注 = 背景 */
            uint16_t r = uf_find(parent, l);
            if (r >= DET_MAX_COMPS) continue;        /* 超上限按背景处理 */
            if (wbbox[r][0] > 0 || wbbox[r][1] < w - 1 ||
                wbbox[r][2] > 0 || wbbox[r][3] < h - 1) continue;
            brow[x] = 1;
        }
    }
}

/* ================= 连通域 (8 邻域, union-find) ================= */
static void label_components(int w, int h) {
    uint16_t *lab = g_sc.lab, *parent = g_sc.parent;
    uint16_t next = 1;

    for (int y = 0; y < h; y++) {
        uint8_t *brow = g_sc.bin + (size_t)y * w;
        uint16_t *lrow = lab + (size_t)y * w;
        uint16_t *lup = (y > 0) ? lab + (size_t)(y - 1) * w : NULL;
        for (int x = 0; x < w; x++) {
            if (!brow[x]) { lrow[x] = 0; continue; }
            uint16_t m = 0;
            if (x > 0 && lrow[x - 1]) m = lrow[x - 1];
            if (y > 0) {
                if (lup[x] && (!m || lup[x] < m)) m = lup[x];
                if (x > 0 && lup[x - 1] && (!m || lup[x - 1] < m)) m = lup[x - 1];
                if (x + 1 < w && lup[x + 1] && (!m || lup[x + 1] < m)) m = lup[x + 1];
            }
            if (!m) {
                if (next >= 65535) { lrow[x] = 0; continue; }
                m = next++;
                parent[m] = m;
                if (m < DET_MAX_COMPS) {
                    CompStat *st = &g_sc.stats[m];
                    st->minx = st->maxx = x; st->miny = st->maxy = y;
                }
            } else {
                m = uf_find(parent, m);
            }
            lrow[x] = m;
            if (x > 0 && lrow[x - 1] && lrow[x - 1] != m) {
                uint16_t a = uf_find(parent, lrow[x - 1]);
                if (a != m) parent[a] = m;
            }
            if (y > 0) {
                uint16_t ns[3] = {(uint16_t)lup[x], (uint16_t)((x > 0) ? lup[x - 1] : 0),
                                  (uint16_t)((x + 1 < w) ? lup[x + 1] : 0)};
                for (int k = 0; k < 3; k++) {
                    if (ns[k] && ns[k] != m) {
                        uint16_t a = uf_find(parent, ns[k]);
                        if (a != m) parent[a] = m;
                    }
                }
            }
        }
    }

    /* 累加统计 (路径压缩到根) */
    g_ncomps = next - 1;
    memset(g_sc.stats, 0, (size_t)DET_MAX_COMPS * sizeof(CompStat));
    for (int y = 0; y < h; y++) {
        uint16_t *lrow = lab + (size_t)y * w;
        for (int x = 0; x < w; x++) {
            uint16_t l = lrow[x];
            if (!l) continue;
            uint16_t r = uf_find(parent, l);
            lrow[x] = r;
            if (r >= DET_MAX_COMPS) continue;   /* 超上限的域不统计 */
            CompStat *st = &g_sc.stats[r];
            int64_t ix = x, iy = y;
            if (st->area == 0) { st->minx = st->maxx = x; st->miny = st->maxy = y; }
            st->area++;
            st->sx += ix; st->sy += iy;
            st->sxx += ix * ix; st->syy += iy * iy; st->sxy += ix * iy;
            if (x < st->minx) st->minx = x;
            if (x > st->maxx) st->maxx = x;
            if (y < st->miny) st->miny = y;
            if (y > st->maxy) st->maxy = y;
        }
    }
}

/* ================= 3x3 与 8x8 线性求解 ================= */
static bool solve3(double A[3][4]) {
    for (int i = 0; i < 3; i++) {
        int piv = i;
        for (int k = i + 1; k < 3; k++)
            if (fabs(A[k][i]) > fabs(A[piv][i])) piv = k;
        if (fabs(A[piv][i]) < 1e-12) return false;
        if (piv != i) for (int j = i; j < 4; j++) { double t = A[i][j]; A[i][j] = A[piv][j]; A[piv][j] = t; }
        for (int k = i + 1; k < 3; k++) {
            double f = A[k][i] / A[i][i];
            for (int j = i; j < 4; j++) A[k][j] -= f * A[i][j];
        }
    }
    for (int i = 2; i >= 0; i--) {
        double s = A[i][3];
        for (int j = i + 1; j < 3; j++) s -= A[i][j] * A[j][3];
        A[i][3] = s / A[i][i];
    }
    return true;
}

/* 4 点对应 (tag坐标 [-1,1]² -> 图像) 解单应 H, 使 px = H00 xt+H01 yt+H02 / (H20 xt+H21 yt+1) */
static bool homography_solve(const float A[4][2], double H[3][3]) {
    static const float TC[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    double M[8][9] = {{0}};
    for (int i = 0; i < 4; i++) {
        double xt = TC[i][0], yt = TC[i][1];
        double px = A[i][0], py = A[i][1];
        double *r1 = &M[2 * i][0], *r2 = &M[2 * i + 1][0];
        r1[0] = xt; r1[1] = yt; r1[2] = 1; r1[6] = -px * xt; r1[7] = -px * yt; r1[8] = px;
        r2[3] = xt; r2[4] = yt; r2[5] = 1; r2[6] = -py * xt; r2[7] = -py * yt; r2[8] = py;
    }
    for (int i = 0; i < 8; i++) {           /* Gauss 消元 (部分主元) */
        int piv = i;
        for (int k = i + 1; k < 8; k++)
            if (fabs(M[k][i]) > fabs(M[piv][i])) piv = k;
        if (fabs(M[piv][i]) < 1e-12) return false;
        if (piv != i) for (int j = i; j < 9; j++) { double t = M[i][j]; M[i][j] = M[piv][j]; M[piv][j] = t; }
        for (int k = i + 1; k < 8; k++) {
            double f = M[k][i] / M[i][i];
            for (int j = i; j < 9; j++) M[k][j] -= f * M[i][j];
        }
    }
    double x[8];
    for (int i = 7; i >= 0; i--) {
        double s = M[i][8];
        for (int j = i + 1; j < 8; j++) s -= M[i][j] * x[j];
        x[i] = s / M[i][i];
    }
    H[0][0] = x[0]; H[0][1] = x[1]; H[0][2] = x[2];
    H[1][0] = x[3]; H[1][1] = x[4]; H[1][2] = x[5];
    H[2][0] = x[6]; H[2][1] = x[7]; H[2][2] = 1;
    return true;
}

static void h_project(const double H[3][3], double xt, double yt, double *px, double *py) {
    double d = H[2][0] * xt + H[2][1] * yt + 1.0;
    *px = (H[0][0] * xt + H[0][1] * yt + H[0][2]) / d;
    *py = (H[1][0] * xt + H[1][1] * yt + H[1][2]) / d;
}

/* ================= 灰度电平平面拟合 ================= */
typedef struct { double sx, sy, sxx, syy, sxy, sv, sxv, syv; int n; } GrayModel;

static void gm_add(GrayModel *g, double x, double y, double v) {
    g->n++; g->sx += x; g->sy += y; g->sxx += x * x; g->syy += y * y;
    g->sxy += x * y; g->sv += v; g->sxv += x * v; g->syv += y * v;
}

/* 解 v = a + b x + c y; 求解结果存 a,b,c, 失败返回 false */
static bool gm_solve(const GrayModel *g, double *a, double *b, double *c) {
    if (g->n < 6) return false;
    double A[3][4];
    A[0][0] = g->n;    A[0][1] = g->sx;  A[0][2] = g->sy;  A[0][3] = g->sv;
    A[1][0] = g->sx;   A[1][1] = g->sxx; A[1][2] = g->sxy; A[1][3] = g->sxv;
    A[2][0] = g->sy;   A[2][1] = g->sxy; A[2][2] = g->syy; A[2][3] = g->syv;
    if (!solve3(A)) return false;
    *a = A[0][3]; *b = A[1][3]; *c = A[2][3];
    return true;
}

static double gm_eval(double a, double b, double c, double x, double y) { return a + b * x + c * y; }

/* ================= 解码一次 (给定 TL/TR/BR/BL 角点赋值) ================= */
static int decode_attempt(const uint8_t *im, int w, int h, int stride,
                          const float A[4][2], TagObservation *out) {
    double H[3][3];
    if (!homography_solve(A, H)) return 255;

    const int WAB = TAG36H11_WIDTH_AT_BORDER;
    /* 黑白电平采样线 (同官方): 白在黑框外 0.5 格, 黑在框内 0.5 格 */
    GrayModel gw, gb;
    memset(&gw, 0, sizeof(gw)); memset(&gb, 0, sizeof(gb));
    for (int i = 0; i < 8; i++) {
        const double off = 0.5, span = (double)i + 0.5;
        const double coords[8][2] = {
            {-off, span}, {WAB + off, span},          /* 左右白列 */
            {off, span},  {WAB - off, span},          /* 左右黑列 */
            {span, -off}, {span, WAB + off},          /* 上下白行 */
            {span, off},  {span, WAB - off},          /* 上下黑行 */
        };
        for (int k = 0; k < 8; k++) {
            double cx = coords[k][0], cy = coords[k][1];
            double xt = 2.0 * cx / WAB - 1.0, yt = 2.0 * cy / WAB - 1.0;
            double px, py;
            h_project(H, xt, yt, &px, &py);
            float v = samp_bilinear(im, w, h, stride, (float)px, (float)py);
            if (v < 0) continue;
            bool is_white = (k < 2 || (k >= 4 && k < 6));
            gm_add(is_white ? &gw : &gb, xt, yt, v);
        }
    }
    double wa, wb, wc, ba, bb, bc;
    if (!gm_solve(&gw, &wa, &wb, &wc) || !gm_solve(&gb, &ba, &bb, &bc)) return 255;
    double contrast = gm_eval(wa, wb, wc, 0, 0) - gm_eval(ba, bb, bc, 0, 0);
    if (contrast < DET_CONTRAST_MIN) return 255;

    /* 采样 36 数据位, MSB 在前 (官方螺旋序) */
    uint64_t rcode = 0;
    double margin_sum = 0;
    for (int i = 0; i < TAG36H11_NBITS; i++) {
        double cx = TAG36H11_BIT_X[i] + 0.5, cy = TAG36H11_BIT_Y[i] + 0.5;
        double xt = 2.0 * cx / WAB - 1.0, yt = 2.0 * cy / WAB - 1.0;
        double px, py;
        h_project(H, xt, yt, &px, &py);
        float v = samp_bilinear(im, w, h, stride, (float)px, (float)py);
        if (v < 0) return 255;                       /* 位采样出图 -> 放弃 */
        double thresh = 0.5 * (gm_eval(wa, wb, wc, xt, yt) + gm_eval(ba, bb, bc, xt, yt));
        margin_sum += fabs(v - thresh);
        rcode = (rcode << 1) | (uint64_t)(v > thresh ? 1 : 0);
    }

    /* 码表匹配 (含容错位) */
    int best_h = 100, best_id = -1;
    for (int i = 0; i < TAG36H11_NCODES; i++) {
        if (TAG36H11_CODES[i] == rcode) { best_h = 0; best_id = i; break; }
    }
    if (best_h != 0) {
        for (int i = 0; i < TAG36H11_NCODES; i++) {
            int d = __builtin_popcountll(TAG36H11_CODES[i] ^ rcode);
            if (d < best_h) { best_h = d; best_id = i; if (d <= DET_MAX_HAMMING) break; }
        }
    }
    if (best_h > DET_MAX_HAMMING || best_id < 0) return 255;

    /* 角点约定: 官方位网格朝向与 cv2 渲染的图案相差恒定 180°
     * (见 视觉/code/pc_check/check_convention.py 的验证)。
     * 输出角点移 2 位, 使 c0..c3 = cv2 渲染图案的 左上/右上/右下/左下,
     * theta/phi/arc/axial 的符号与 Python 工具链 (tag_utils.py) 完全一致。 */
    float S[4][2];
    for (int i = 0; i < 4; i++) {
        S[i][0] = A[(i + 2) % 4][0];
        S[i][1] = A[(i + 2) % 4][1];
    }

    /* 与 tag_utils.py 一致: c0左上 c1右上, theta = c0->c1 方向角 */
    float e01 = hypotf(S[1][0] - S[0][0], S[1][1] - S[0][1]);
    float e12 = hypotf(S[2][0] - S[1][0], S[2][1] - S[1][1]);
    float e23 = hypotf(S[3][0] - S[2][0], S[3][1] - S[2][1]);
    float e30 = hypotf(S[0][0] - S[3][0], S[0][1] - S[3][1]);

    memset(out, 0, sizeof(*out));
    out->ok = true;
    out->id = best_id;
    out->hamming = best_h;
    out->u = (S[0][0] + S[1][0] + S[2][0] + S[3][0]) * 0.25f;
    out->v = (S[0][1] + S[1][1] + S[2][1] + S[3][1]) * 0.25f;
    out->theta = atan2f(S[1][1] - S[0][1], S[1][0] - S[0][0]);
    out->edge_a = 0.5f * (e01 + e23);
    out->edge_p = 0.5f * (e12 + e30);
    out->margin = (float)(margin_sum / TAG36H11_NBITS);
    out->contrast = (float)contrast;
    memcpy(out->corners, S, sizeof(out->corners));
    return best_h;
}

/* ================= 角点精修 (边缘射线 + 直线拟合 + 求交) ================= */
typedef struct { double x, y, dx, dy; bool valid; } Line2D;

static bool fit_side_line(const uint8_t *im, int w, int h, int stride,
                          const float PA[2], const float PB[2],
                          const float centroid[2], Line2D *ln) {
    double sx = PB[0] - PA[0], sy = PB[1] - PA[1];
    double len = hypot(sx, sy);
    if (len < 8) return false;
    double dx = sx / len, dy = sy / len;
    double nx = -dy, ny = dx;
    /* 法向指向远离形心一侧 (黑框外侧) */
    double mx = 0.5 * (PA[0] + PB[0]) - centroid[0];
    double my = 0.5 * (PA[1] + PB[1]) - centroid[1];
    if (nx * mx + ny * my < 0) { nx = -nx; ny = -ny; }

    double pts[9][2];
    int npts = 0;
    const int RAY = 5;
    for (int k = 0; k < 9; k++) {
        double t = 0.2 + k * 0.075;
        double bx = PA[0] + t * sx, by = PA[1] + t * sy;
        float g[2 * RAY + 1];
        for (int s = 0; s <= 2 * RAY; s++)
            g[s] = samp_bilinear(im, w, h, stride,
                                 (float)(bx + (RAY - s) * nx), (float)(by + (RAY - s) * ny));
        float gmax = -1e9f, gmin = 1e9f;
        for (int s = 0; s <= 2 * RAY; s++) {
            if (g[s] < 0) { gmax = -1; break; }
            if (g[s] > gmax) gmax = g[s];
            if (g[s] < gmin) gmin = g[s];
        }
        if (gmax < 0 || gmax - gmin < DET_CONTRAST_MIN) continue;
        float T = 0.5f * (gmax + gmin);
        /* s=0 对应最外侧, 向内找首次变暗, 线性插值亚像素 */
        for (int s = 0; s < 2 * RAY; s++) {
            if (g[s] >= T && g[s + 1] < T) {
                float frac = (g[s] - T) / (g[s] - g[s + 1]);
                float sd = (RAY - s) - frac;      /* 距中心的外法向像素数 */
                pts[npts][0] = bx + sd * nx;
                pts[npts][1] = by + sd * ny;
                npts++;
                break;
            }
        }
    }
    if (npts < 5) return false;

    double cx = 0, cy = 0;
    for (int i = 0; i < npts; i++) { cx += pts[i][0]; cy += pts[i][1]; }
    cx /= npts; cy /= npts;
    double mxx = 0, myy = 0, mxy = 0;
    for (int i = 0; i < npts; i++) {
        double ax = pts[i][0] - cx, ay = pts[i][1] - cy;
        mxx += ax * ax; myy += ay * ay; mxy += ax * ay;
    }
    double th = 0.5 * atan2(2 * mxy, mxx - myy);
    ln->x = cx; ln->y = cy;
    ln->dx = cos(th); ln->dy = sin(th);
    ln->valid = true;
    return true;
}

static bool line_intersect(const Line2D *l1, const Line2D *l2, float out[2]) {
    double det = l1->dx * l2->dy - l2->dx * l1->dy;
    if (fabs(det) < 1e-9) return false;
    double t = ((l2->x - l1->x) * l2->dy - (l2->y - l1->y) * l2->dx) / det;
    out[0] = (float)(l1->x + t * l1->dx);
    out[1] = (float)(l1->y + t * l1->dy);
    return true;
}

static void refine_corners(const uint8_t *im, int w, int h, int stride,
                           const float P[4][2], const float centroid[2], float R[4][2]) {
    Line2D lines[4];
    for (int i = 0; i < 4; i++) {
        const float *PA = P[i], *PB = P[(i + 1) % 4];
        lines[i].valid = false;
        fit_side_line(im, w, h, stride, PA, PB, centroid, &lines[i]);
    }
    for (int i = 0; i < 4; i++) {
        R[i][0] = P[i][0]; R[i][1] = P[i][1];      /* 默认退回原角点 */
        int a = (i + 3) % 4, b = i;                /* 角 i = side(a) 与 side(b) 交点 */
        if (lines[a].valid && lines[b].valid)
            line_intersect(&lines[a], &lines[b], R[i]);
    }
}

/* ================= 候选 -> 角点 -> 解码 ================= */
typedef struct { int id; int32_t area; } Cand;

static int cand_cmp(const void *a, const void *b) {
    return ((const Cand *)b)->area - ((const Cand *)a)->area;
}

static bool comp_extremes(const CompStat *st, uint16_t id, int w, int h, float P[4][2]) {
    /* 主轴方向 u, 对角方向 d1 = u+v, d2 = u-v; 4 角点 = d1/d2 的极值点 */
    double cx = (double)st->sx / st->area, cy = (double)st->sy / st->area;
    double mu20 = st->sxx / st->area - cx * cx;
    double mu02 = st->syy / st->area - cy * cy;
    double mu11 = st->sxy / st->area - cx * cy;
    double th = 0.5 * atan2(2 * mu11, mu20 - mu02);
    double ux = cos(th), uy = sin(th);
    double d1x = ux - uy, d1y = ux + uy;     /* u + v (v = (-uy, ux)) */
    double d2x = ux + uy, d2y = uy - ux;

    double b1 = -1e18, b2 = 1e18, b3 = -1e18, b4 = 1e18;
    int p1 = -1, p2 = -1, p3 = -1, p4 = -1;
    for (int y = st->miny; y <= st->maxy; y++) {
        const uint16_t *lrow = g_sc.lab + (size_t)y * w;
        for (int x = st->minx; x <= st->maxx; x++) {
            if (lrow[x] != id) continue;
            double v1 = x * d1x + y * d1y;
            double v2 = x * d2x + y * d2y;
            if (v1 > b1) { b1 = v1; p1 = y * w + x; }
            if (v1 < b2) { b2 = v1; p2 = y * w + x; }
            if (v2 > b3) { b3 = v2; p3 = y * w + x; }
            if (v2 < b4) { b4 = v2; p4 = y * w + x; }
        }
    }
    if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0) return false;
    P[0][0] = p1 % w; P[0][1] = p1 / w;
    P[1][0] = p2 % w; P[1][1] = p2 / w;
    P[2][0] = p3 % w; P[2][1] = p3 / w;
    P[3][0] = p4 % w; P[3][1] = p4 / w;
    return true;
}

/* ================= 对外入口 ================= */
bool TagDetector_Detect(const uint8_t *gray, int w, int h, int stride, TagObservation *out) {
    return TagDetector_DetectEx(gray, w, h, stride, out, NULL);
}

bool TagDetector_DetectEx(const uint8_t *gray, int w, int h, int stride,
                          TagObservation *out, TagDiag *diag) {
    memset(out, 0, sizeof(*out));
    out->ok = false;
    if (diag) {
        memset(diag, 0, sizeof(*diag));
        diag->best_hamming = 255;
        diag->stage = 1;
    }
    if (!gray || w <= 0 || h <= 0) return false;
    if (!scratch_ensure(w, h)) return false;

    threshold_adaptive(gray, w, h, stride);
    fill_holes(w, h);
    label_components(w, h);
    if (diag) diag->ncomps = g_ncomps;
    if (g_ncomps == 0) return false;

    /* 按面积取前几个候选 */
    Cand *cands = g_sc.cands;
    if (!cands) return false;
    int ncand = 0;
    for (int i = 1; i < DET_MAX_COMPS && i < 65535; i++) {
        if (g_sc.stats[i].area > 0) {
            cands[ncand].id = i;
            cands[ncand].area = g_sc.stats[i].area;
            ncand++;
        }
    }
    if (ncand == 0) return false;
    qsort(cands, ncand, sizeof(Cand), cand_cmp);
    if (diag) {
        diag->ncand = ncand;
        const CompStat *top = &g_sc.stats[cands[0].id];
        diag->cand_bbox[0] = top->minx;
        diag->cand_bbox[1] = top->miny;
        diag->cand_bbox[2] = top->maxx - top->minx + 1;
        diag->cand_bbox[3] = top->maxy - top->miny + 1;
        diag->cand_area = (int)top->area;
        int bw0 = top->maxx - top->minx + 1, bh0 = top->maxy - top->miny + 1;
        diag->cand_fill = (bw0 > 0 && bh0 > 0) ? (float)top->area / (float)(bw0 * bh0) : 0.0f;
        diag->stage = 2;
    }
    if (ncand > DET_MAX_CANDS) ncand = DET_MAX_CANDS;

    TagObservation best;
    int best_h = 255;

    for (int ci = 0; ci < ncand && best_h > DET_MAX_HAMMING; ci++) {
        const CompStat *st = &g_sc.stats[cands[ci].id];
        float bw = st->maxx - st->minx + 1, bh = st->maxy - st->miny + 1;
        /* 预筛: 旋转方形黑框的包围盒/面积特征 */
#ifdef DET_DEBUG
        fprintf(stderr, "[cand %d] id=%d area=%d bbox=(%.0f,%.0f) fill=%.2f\n",
                ci, cands[ci].id, (int)st->area, bw, bh,
                (double)st->area / (bw * bh));
#endif
        if (bw < 0.7f * DET_EDGE_PX_MIN || bh < 0.7f * DET_EDGE_PX_MIN) {
            if (diag) diag->drop_small++;
            continue;
        }
        if (bw > 1.6f * DET_EDGE_PX_MAX || bh > 1.6f * DET_EDGE_PX_MAX) {
            if (diag) diag->drop_large++;
            continue;
        }
        double fill = (double)st->area / (bw * bh);
        /* 下限 0.15: 环带形态(黑框+暗位连不齐)约 0.2+, 细碎轮廓线 <0.1;
         * 上限 0.99: 填洞后是实心方块 (正对时 fill≈0.98) */
        if (fill < 0.15 || fill > 0.99) {
            if (diag) diag->drop_fill++;
            continue;
        }
        double ar = bw / bh;
        if (ar < 0.4 || ar > 2.5) {
            if (diag) diag->drop_aspect++;
            continue;
        }
        if (diag) diag->decode_tried++;

        float P[4][2];
        float centroid[2] = {(float)((double)st->sx / st->area), (float)((double)st->sy / st->area)};
        if (!comp_extremes(st, cands[ci].id, w, h, P)) continue;

        /* 角点按绕形心方位角排序, 得到一致环序 */
        float ang[4];
        for (int i = 0; i < 4; i++)
            ang[i] = atan2f(P[i][1] - centroid[1], P[i][0] - centroid[0]);
        for (int i = 0; i < 3; i++)
            for (int j = i + 1; j < 4; j++)
                if (ang[j] < ang[i]) {
                    float t = ang[i]; ang[i] = ang[j]; ang[j] = t;
                    float tmp[2] = {P[i][0], P[i][1]};
                    P[i][0] = P[j][0]; P[i][1] = P[j][1];
                    P[j][0] = tmp[0]; P[j][1] = tmp[1];
                }

        float R[4][2];
        refine_corners(gray, w, h, stride, P, centroid, R);

        /* 2 种绕向 x 4 个起始角 = 8 种角点赋值, 取最小汉明 */
        const float *base[2][4] = {
            {R[0], R[1], R[2], R[3]},
            {R[0], R[3], R[2], R[1]},
        };
        for (int ori = 0; ori < 2 && best_h > DET_MAX_HAMMING; ori++) {
            for (int r = 0; r < 4 && best_h > DET_MAX_HAMMING; r++) {
                float A[4][2];
                for (int i = 0; i < 4; i++) {
                    const float *src = base[ori][(r + i) % 4];
                    A[i][0] = src[0]; A[i][1] = src[1];
                }
                TagObservation o;
                int ham = decode_attempt(gray, w, h, stride, A, &o);
                if (diag && ham < diag->best_hamming) {
                    diag->best_hamming = ham;
                    diag->best_contrast = o.contrast;
                    diag->best_margin = o.margin;
                }
                if (ham < best_h) { best_h = ham; best = o; }
            }
        }
    }

    if (best_h > DET_MAX_HAMMING) {
        if (diag) diag->stage = (diag->decode_tried > 0) ? 3 : 2;
        return false;
    }
    if (diag) diag->stage = 0;
    *out = best;
    return true;
}
