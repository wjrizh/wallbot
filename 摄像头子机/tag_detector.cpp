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
#define DET_MAX_CANDS     24     /* 杂乱背景时面积大的块会占位, 多试几个 */

typedef struct {
    int32_t area;
    int16_t minx, maxx, miny, maxy;
    int64_t sx, sy, sxx, syy, sxy;
} CompStat;

/* 候选连通域: 4096 个 x 8B = 32KB, 必须放 PSRAM, 不能放栈上 */

typedef struct { int id; int32_t area; } Cand;
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
    uint8_t  *half;    /* 半分辨率灰度 (连通域加速) */
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
    free(g_sc.stats); free(g_sc.wbbox); free(g_sc.cands); free(g_sc.half);
    memset(&g_sc, 0, sizeof(g_sc));
    g_sc.ii     = (uint32_t *)psram_malloc((size_t)(w + 1) * (h + 1) * 4);
    g_sc.bin    = (uint8_t *)psram_malloc((size_t)w * h);
    g_sc.lab    = (uint16_t *)psram_malloc((size_t)w * h * 2);
    g_sc.parent = (uint16_t *)psram_malloc((size_t)w * h * 2);
    g_sc.stats  = (CompStat *)psram_malloc((size_t)DET_MAX_COMPS * sizeof(CompStat));
    g_sc.wbbox  = (uint16_t (*)[4])psram_malloc((size_t)DET_MAX_COMPS * 4 * 2);
    g_sc.cands  = (Cand *)psram_malloc((size_t)DET_MAX_COMPS * sizeof(Cand));
    g_sc.half   = (uint8_t *)psram_malloc((size_t)w * h);
    if (!g_sc.ii || !g_sc.bin || !g_sc.lab || !g_sc.parent ||
        !g_sc.stats || !g_sc.wbbox || !g_sc.cands || !g_sc.half) {
        free(g_sc.ii); free(g_sc.bin); free(g_sc.lab); free(g_sc.parent);
        free(g_sc.stats); free(g_sc.wbbox); free(g_sc.cands); free(g_sc.half);
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
static void wbbox_union(uint16_t (*wbbox)[4], uint16_t dst, uint16_t src) {
    if (wbbox[src][0] < wbbox[dst][0]) wbbox[dst][0] = wbbox[src][0];
    if (wbbox[src][1] > wbbox[dst][1]) wbbox[dst][1] = wbbox[src][1];
    if (wbbox[src][2] < wbbox[dst][2]) wbbox[dst][2] = wbbox[src][2];
    if (wbbox[src][3] > wbbox[dst][3]) wbbox[dst][3] = wbbox[src][3];
}

static void wbbox_expand(uint16_t (*wbbox)[4], uint16_t id, int x, int y) {
    if (x < wbbox[id][0]) wbbox[id][0] = (uint16_t)x;
    if (x > wbbox[id][1]) wbbox[id][1] = (uint16_t)x;
    if (y < wbbox[id][2]) wbbox[id][2] = (uint16_t)y;
    if (y > wbbox[id][3]) wbbox[id][3] = (uint16_t)y;
}

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
                wbbox[m][0] = wbbox[m][1] = (uint16_t)x;
                wbbox[m][2] = wbbox[m][3] = (uint16_t)y;
            } else {
                m = uf_find(parent, m);
                wbbox_expand(wbbox, m, x, y);
            }
            lrow[x] = m;
            if (x > 0 && lrow[x - 1] && lrow[x - 1] != m) {
                uint16_t a = uf_find(parent, lrow[x - 1]);
                if (a != m) {
                    parent[a] = m;
                    wbbox_union(wbbox, m, a);
                }
            }
            if (y > 0 && lup[x] && lup[x] != m) {
                uint16_t a = uf_find(parent, lup[x]);
                if (a != m) {
                    parent[a] = m;
                    wbbox_union(wbbox, m, a);
                }
            }
        }
    }

    /* 不触边的白区 -> 填黑 (洞): 须四边都不碰图像边界 */
    for (int y = 0; y < h; y++) {
        uint8_t *brow = g_sc.bin + (size_t)y * w;
        uint16_t *lrow = lab + (size_t)y * w;
        for (int x = 0; x < w; x++) {
            if (brow[x]) continue;
            uint16_t l = lrow[x];
            if (l == 0) continue;                    /* 未标注 = 背景 */
            uint16_t r = uf_find(parent, l);
            if (r >= DET_MAX_COMPS) continue;        /* 超上限按背景处理 */
            if (!(wbbox[r][0] > 0 && wbbox[r][1] < w - 1 &&
                  wbbox[r][2] > 0 && wbbox[r][3] < h - 1)) continue;
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

    /* Phase A1: 连通域强制全分辨率 (对标 quad_decimate=1); 角点精修/采码仍用原灰度 */
    const int scale = 1;
    const int cw = w / scale, ch = h / scale;
    if (!scratch_ensure(cw, ch)) return false;

    const uint8_t *cc_gray = gray;
    int cc_stride = stride;
    if (scale == 2) {
        for (int y = 0; y < ch; y++) {
            const uint8_t *r0 = gray + (size_t)(y * 2) * stride;
            const uint8_t *r1 = gray + (size_t)(y * 2 + 1) * stride;
            uint8_t *dst = g_sc.half + (size_t)y * cw;
            for (int x = 0; x < cw; x++) {
                int s = r0[x * 2] + r0[x * 2 + 1] + r1[x * 2] + r1[x * 2 + 1];
                dst[x] = (uint8_t)(s >> 2);
            }
        }
        cc_gray = g_sc.half;
        cc_stride = cw;
    }

    threshold_adaptive(cc_gray, cw, ch, cc_stride);
    fill_holes(cw, ch);
    label_components(cw, ch);
    if (diag) diag->ncomps = g_ncomps;
    if (g_ncomps == 0) return false;

    /* 先按方形/尺寸筛, 再按面积排序。杂乱场景里大黑块不能挤掉真正的标签。 */
    Cand *cands = g_sc.cands;
    if (!cands) return false;
    int ncand = 0;
    for (int i = 1; i < DET_MAX_COMPS && i < 65535; i++) {
        const CompStat *st = &g_sc.stats[i];
        if (st->area <= 0) continue;
        float bw = st->maxx - st->minx + 1, bh = st->maxy - st->miny + 1;
        if (bw < 0.7f * DET_EDGE_PX_MIN || bh < 0.7f * DET_EDGE_PX_MIN) {
            if (diag) diag->drop_small++;
            continue;
        }
        if (bw > 1.6f * DET_EDGE_PX_MAX || bh > 1.6f * DET_EDGE_PX_MAX) {
            if (diag) diag->drop_large++;
            continue;
        }
        double fill = (double)st->area / (bw * bh);
        if (fill < 0.15) {
            if (diag) diag->drop_fill++;
            continue;
        }
        double ar = bw / bh;
        if (ar < 0.4 || ar > 2.5) {
            if (diag) diag->drop_aspect++;
            continue;
        }
        cands[ncand].id = i;
        cands[ncand].area = st->area;
        ncand++;
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
         * 填洞后轴对齐实心方块 fill 可达 1.0, 不再设上限 */
        if (fill < 0.15) {
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
        if (!comp_extremes(st, cands[ci].id, cw, ch, P)) continue;
        if (scale == 2) {
            for (int i = 0; i < 4; i++) { P[i][0] *= 2.0f; P[i][1] *= 2.0f; }
            centroid[0] *= 2.0f; centroid[1] *= 2.0f;
        }

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

/* ================= 小车俯视双腰形孔 =================
 * 通孔有深度: 孔心亮度随后面的东西/光照变, 不能当实心黑块。
 * 做法: 估 PLA 灰度 → 车体=接近 PLA 的像素 → 填封闭空洞 → 空洞=孔
 * (孔心比 PLA 亮或暗都行, 只要被塑料围住)。
 * 中点 (u,v), 连线 theta, 孔心距 edge_a。 */
#define CH_MAX_HOLES 32
#define CH_MIN_AREA  10
#define CH_MAX_AREA  8000

static float s_lock_u = -1.0f, s_lock_v = -1.0f;
static uint32_t s_lock_ms = 0;

typedef struct {
    float cx, cy, major, minor, axis, fill, aspect, len;
    int32_t area;
    int16_t minx, miny, maxx, maxy;
} HoleCand;

static void invert_bin(int w, int h) {
    size_t n = (size_t)w * (size_t)h;
    uint8_t *b = g_sc.bin;
    for (size_t i = 0; i < n; i++) b[i] = (uint8_t)(1 - b[i]);
}

static float ang_udiff(float a, float b) {
    float d = a - b;
    while (d > (float)M_PI * 0.5f) d -= (float)M_PI;
    while (d < -(float)M_PI * 0.5f) d += (float)M_PI;
    return fabsf(d);
}

static void hole_geom(const CompStat *st, HoleCand *h) {
    double n = (double)st->area;
    double mx = st->sx / n, my = st->sy / n;
    double cxx = st->sxx / n - mx * mx;
    double cyy = st->syy / n - my * my;
    double cxy = st->sxy / n - mx * my;
    double tr = cxx + cyy;
    double tmp = sqrt((cxx - cyy) * (cxx - cyy) + 4.0 * cxy * cxy);
    double l1 = 0.5 * (tr + tmp);
    double l2 = 0.5 * (tr - tmp);
    if (l2 < 0) l2 = 0;
    h->cx = (float)mx;
    h->cy = (float)my;
    h->major = 2.0f * sqrtf((float)l1 + 1e-6f);
    h->minor = 2.0f * sqrtf((float)l2 + 1e-6f);
    h->axis = 0.5f * (float)atan2(2.0 * cxy, cxx - cyy);
    float bw = (float)(st->maxx - st->minx + 1);
    float bh = (float)(st->maxy - st->miny + 1);
    h->len = (bw > bh) ? bw : bh;
    h->fill = (float)((double)st->area / (bw * bh));
    h->aspect = (bw > bh) ? (bw / bh) : (bh / bw);
    h->area = st->area;
    h->minx = st->minx; h->maxx = st->maxx;
    h->miny = st->miny; h->maxy = st->maxy;
}

static int collect_holes(int w, int h, HoleCand *hs, int maxn, TagDiag *diag) {
    (void)w; (void)h;
    int n = 0;
    for (int i = 1; i < DET_MAX_COMPS; i++) {
        const CompStat *st = &g_sc.stats[i];
        if (st->area < CH_MIN_AREA) {
            if (st->area > 0 && diag) diag->drop_small++;
            continue;
        }
        if (st->area > CH_MAX_AREA) {
            if (diag) diag->drop_large++;
            continue;
        }
        float bw = (float)(st->maxx - st->minx + 1);
        float bh = (float)(st->maxy - st->miny + 1);
        float long_s = (bw > bh) ? bw : bh;
        float short_s = (bw > bh) ? bh : bw;
        if (long_s < 5.0f) {
            if (diag) diag->drop_small++;
            continue;
        }
        if (long_s > 160.0f) {
            if (diag) diag->drop_large++;
            continue;
        }
        double fill = (double)st->area / (bw * bh);
        if (fill < 0.28 || fill > 0.94) {
            if (diag) diag->drop_fill++;
            continue;
        }
        double ar = (double)long_s / (short_s > 1.0f ? short_s : 1.0f);
        if (ar < 1.25 || ar > 5.0) {
            if (diag) diag->drop_aspect++;
            continue;
        }
        if (n >= maxn) break;
        hole_geom(st, &hs[n]);
        n++;
    }
    return n;
}

static bool pair_holes(const HoleCand *hs, int n, int cw, int ch, float scale,
                       int *bi, int *bj, float *bsc) {
    *bi = *bj = -1;
    *bsc = -1e9f;
    const float max_align = 35.0f * (float)M_PI / 180.0f;
    const float img_lim = 0.42f * (cw < ch ? (float)cw : (float)ch);
    const bool have_lock = (s_lock_u >= 0.0f) && (millis() - s_lock_ms < 2000);
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            float ar = (hs[i].area > hs[j].area)
                ? (float)hs[i].area / (float)hs[j].area
                : (float)hs[j].area / (float)hs[i].area;
            if (ar > 1.8f) continue;
            float dx = hs[j].cx - hs[i].cx;
            float dy = hs[j].cy - hs[i].cy;
            float dist = sqrtf(dx * dx + dy * dy);
            if (dist < 6.0f || dist > img_lim) continue;
            float mean_len = 0.5f * (hs[i].len + hs[j].len);
            if (mean_len < 1.0f) continue;
            float ratio = dist / mean_len;
            /* 孔长约 20mm、心距约 45mm -> ratio ~ 2.0~2.5 */
            if (ratio < 1.35f || ratio > 3.6f) continue;
            float pair_th = atan2f(dy, dx);
            if (ang_udiff(hs[i].axis, pair_th) > max_align) continue;
            if (ang_udiff(hs[j].axis, pair_th) > max_align) continue;
            float sc = (1.8f - ar) * 4.0f
                + (1.0f - fabsf(ratio - 2.2f)) * 2.0f
                + 40.0f / (dist + 8.0f);
            if (have_lock) {
                float mx = 0.5f * (hs[i].cx + hs[j].cx) * scale;
                float my = 0.5f * (hs[i].cy + hs[j].cy) * scale;
                float du = mx - s_lock_u, dv = my - s_lock_v;
                sc += 6.0f / (1.0f + sqrtf(du * du + dv * dv) / 30.0f);
            }
            if (sc > *bsc) { *bsc = sc; *bi = i; *bj = j; }
        }
    }
    return *bi >= 0;
}

static float gray_at(const uint8_t *gray, int w, int h, int stride, float x, float y) {
    int ix = (int)(x + 0.5f), iy = (int)(y + 0.5f);
    if (ix < 0 || iy < 0 || ix >= w || iy >= h) return -1.0f;
    return (float)gray[(size_t)iy * stride + ix];
}

/* 孔外一圈 PLA 的灰度, 用来认车体而不是桌面 */
static float sample_pla(const uint8_t *gray, int w, int h, int stride,
                        const HoleCand *a, const HoleCand *b, float scale) {
    float sum = 0;
    int n = 0;
    const HoleCand *hs[2] = {a, b};
    const float dir[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{0.7f,0.7f},{-0.7f,0.7f},{0.7f,-0.7f},{-0.7f,-0.7f}};
    for (int k = 0; k < 2; k++) {
        float r = (hs[k]->minor * 0.9f + 3.0f) * scale;
        float cx = hs[k]->cx * scale, cy = hs[k]->cy * scale;
        for (int d = 0; d < 8; d++) {
            float g = gray_at(gray, w, h, stride, cx + dir[d][0] * r, cy + dir[d][1] * r);
            if (g >= 0) { sum += g; n++; }
        }
    }
    return (n > 0) ? (sum / n) : 128.0f;
}

static float bar_body(const uint8_t *gray, int w, int h, int stride,
                      float cx, float cy, float px, float py, float pla, float scan) {
    int hits = 0, n = (int)(scan + 0.5f);
    if (n < 6) n = 6;
    for (int t = -n; t <= n; t++) {
        float g = gray_at(gray, w, h, stride, cx + px * t, cy + py * t);
        if (g >= 0 && fabsf(g - pla) < 38.0f) hits++;
    }
    return (float)hits;
}

static void fill_obs_from_pair(const HoleCand *a, const HoleCand *b,
                               float scale, const uint8_t *gray, int w, int h, int stride,
                               TagObservation *out) {
    float cx0 = a->cx * scale, cy0 = a->cy * scale;
    float cx1 = b->cx * scale, cy1 = b->cy * scale;
    float mx = 0.5f * (cx0 + cx1), my = 0.5f * (cy0 + cy1);
    float dx = cx1 - cx0, dy = cy1 - cy0;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist < 1.0f) dist = 1.0f;
    float ux = dx / dist, uy = dy / dist;
    float px = -uy, py = ux;

    /* 180°: 车头有两个耳, 比管端宽。在双孔外侧各扫一条垂线, 更像 PLA 灰度的那边是头。 */
    float pla = sample_pla(gray, w, h, stride, a, b, scale);
    float reach0 = (0.75f * a->len + 4.0f) * scale;
    float reach1 = (0.75f * b->len + 4.0f) * scale;
    float scan = dist * 0.45f;
    float w0 = bar_body(gray, w, h, stride, cx0 - ux * reach0, cy0 - uy * reach0, px, py, pla, scan);
    float w1 = bar_body(gray, w, h, stride, cx1 + ux * reach1, cy1 + uy * reach1, px, py, pla, scan);
    if (w0 > w1 + 3.0f) {
        ux = -ux; uy = -uy;
        px = -px; py = -py;
    }

    float hw = 0.35f * 0.5f * (a->minor + b->minor) * scale + 4.0f;
    float hl = 0.5f * dist + 0.25f * (a->len + b->len) * scale;
    out->ok = true;
    out->id = 0;
    out->hamming = 0;
    out->u = mx;
    out->v = my;
    out->theta = atan2f(uy, ux);
    out->edge_a = dist;
    out->edge_p = dist;
    out->contrast = 40.0f;
    out->margin = 10.0f;
    out->corners[0][0] = mx - ux * hl - px * hw; out->corners[0][1] = my - uy * hl - py * hw;
    out->corners[1][0] = mx + ux * hl - px * hw; out->corners[1][1] = my + uy * hl - py * hw;
    out->corners[2][0] = mx + ux * hl + px * hw; out->corners[2][1] = my + uy * hl + py * hw;
    out->corners[3][0] = mx - ux * hl + px * hw; out->corners[3][1] = my - uy * hl + py * hw;

    s_lock_u = mx;
    s_lock_v = my;
    s_lock_ms = millis();
}

/* 通孔: 孔心(后面的东西)和孔边(PLA)灰度差要够, 亮暗方向不限 */
static int keep_depth_holes(HoleCand *hs, int n, const uint8_t *im, int w, int h, int stride) {
    int m = 0;
    for (int i = 0; i < n; i++) {
        float cx = hs[i].cx, cy = hs[i].cy;
        float gin = 0;
        int ni = 0;
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                float g = gray_at(im, w, h, stride, cx + dx, cy + dy);
                if (g >= 0) { gin += g; ni++; }
            }
        }
        if (ni) gin /= (float)ni;
        float cax = cosf(hs[i].axis), say = sinf(hs[i].axis);
        float px = -say, py = cax;
        float r = hs[i].minor * 0.85f + 2.5f;
        float rim = 0;
        int nr = 0;
        float dir[4][2] = {{cax, say}, {-cax, -say}, {px, py}, {-px, -py}};
        for (int k = 0; k < 4; k++) {
            float g = gray_at(im, w, h, stride, cx + dir[k][0] * r, cy + dir[k][1] * r);
            if (g >= 0) { rim += g; nr++; }
        }
        if (nr) rim /= (float)nr;
        if (fabsf(gin - rim) >= 10.0f) hs[m++] = hs[i];
    }
    return (m >= 2) ? m : n;
}

bool ChassisDetector_DetectEx(const uint8_t *gray, int w, int h, int stride,
                              TagObservation *out, TagDiag *diag) {
    memset(out, 0, sizeof(*out));
    out->ok = false;
    if (diag) {
        memset(diag, 0, sizeof(*diag));
        diag->best_hamming = 255;
        diag->stage = 1;
    }
    if (!gray || w <= 0 || h <= 0) return false;

    const int scale = (w >= 640 && h >= 480) ? 2 : 1;
    const int cw = w / scale, ch = h / scale;
    if (!scratch_ensure(cw, ch)) return false;

    const uint8_t *cc_gray = gray;
    int cc_stride = stride;
    if (scale == 2) {
        for (int y = 0; y < ch; y++) {
            const uint8_t *r0 = gray + (size_t)(y * 2) * stride;
            const uint8_t *r1 = gray + (size_t)(y * 2 + 1) * stride;
            uint8_t *dst = g_sc.half + (size_t)y * cw;
            for (int x = 0; x < cw; x++) {
                int s = r0[x * 2] + r0[x * 2 + 1] + r1[x * 2] + r1[x * 2 + 1];
                dst[x] = (uint8_t)(s >> 2);
            }
        }
        cc_gray = g_sc.half;
        cc_stride = cw;
    }

    HoleCand hs[CH_MAX_HOLES];
    HoleCand best_a, best_b;
    float best_sc = -1e9f;
    int best_n = 0;
    bool got = false;

    /* 画面中心直方图估 PLA 灰。通孔内部不管亮暗, 只要跟塑料差一截、又被围住。 */
    int hist[32];
    memset(hist, 0, sizeof(hist));
    int x0 = cw / 5, x1 = cw - cw / 5, y0 = ch / 5, y1 = ch - ch / 5;
    for (int y = y0; y < y1; y++) {
        const uint8_t *row = cc_gray + (size_t)y * cc_stride;
        for (int x = x0; x < x1; x++) hist[row[x] >> 3]++;
    }
    int peak = 12, peakn = 0;
    for (int b = 5; b <= 28; b++) {
        if (hist[b] > peakn) { peakn = hist[b]; peak = b; }
    }
    const float pla = (float)(peak * 8 + 4);
    const float pla_tol = 36.0f;
    uint8_t *bin = g_sc.bin;
    for (int y = 0; y < ch; y++) {
        const uint8_t *row = cc_gray + (size_t)y * cc_stride;
        uint8_t *brow = bin + (size_t)y * cw;
        for (int x = 0; x < cw; x++) {
            float g = (float)row[x];
            brow[x] = (fabsf(g - pla) <= pla_tol) ? 1 : 0;
        }
    }
    /* 把车体 mask 暂存在积分图前 w*h 格 (fill_holes 不用 ii) */
    {
        uint32_t *stash = g_sc.ii;
        size_t n = (size_t)cw * (size_t)ch;
        for (size_t i = 0; i < n; i++) stash[i] = bin[i];
    }
    fill_holes(cw, ch);
    {
        uint32_t *stash = g_sc.ii;
        size_t n = (size_t)cw * (size_t)ch;
        for (size_t i = 0; i < n; i++)
            bin[i] = (bin[i] && stash[i] == 0) ? 1 : 0;
    }
    label_components(cw, ch);
    if (diag) diag->ncomps = g_ncomps;
    {
        int n = collect_holes(cw, ch, hs, CH_MAX_HOLES, diag);
        n = keep_depth_holes(hs, n, cc_gray, cw, ch, cc_stride);
        best_n = n;
        if (diag) diag->decode_tried = n;
        int bi = -1, bj = -1;
        float sc = -1e9f;
        if (n >= 2 && pair_holes(hs, n, cw, ch, (float)scale, &bi, &bj, &sc)) {
            best_sc = sc;
            best_a = hs[bi];
            best_b = hs[bj];
            got = true;
        }
    }

    /* 车体分割失败时, 暗/亮都试, 并先填封闭环, 环心亮暗无所谓 */
    if (!got) {
        threshold_adaptive(cc_gray, cw, ch, cc_stride);
        for (int pol = 0; pol < 2; pol++) {
            if (pol == 1) invert_bin(cw, ch);
            fill_holes(cw, ch);
            label_components(cw, ch);
            if (diag) diag->ncomps = g_ncomps;
            int n = collect_holes(cw, ch, hs, CH_MAX_HOLES, diag);
            n = keep_depth_holes(hs, n, cc_gray, cw, ch, cc_stride);
            if (n > best_n) best_n = n;
            if (diag) diag->decode_tried += n;
            int bi = -1, bj = -1;
            float sc = -1e9f;
            if (n >= 2 && pair_holes(hs, n, cw, ch, (float)scale, &bi, &bj, &sc) && sc > best_sc) {
                best_sc = sc;
                best_a = hs[bi];
                best_b = hs[bj];
                got = true;
            }
        }
    }
    if (diag) diag->ncand = best_n;
    if (!got) {
        if (diag) diag->stage = (best_n > 0) ? 3 : 2;
        return false;
    }

    if (diag) {
        int minx = best_a.minx < best_b.minx ? best_a.minx : best_b.minx;
        int miny = best_a.miny < best_b.miny ? best_a.miny : best_b.miny;
        int maxx = best_a.maxx > best_b.maxx ? best_a.maxx : best_b.maxx;
        int maxy = best_a.maxy > best_b.maxy ? best_a.maxy : best_b.maxy;
        diag->cand_bbox[0] = minx * scale;
        diag->cand_bbox[1] = miny * scale;
        diag->cand_bbox[2] = (maxx - minx + 1) * scale;
        diag->cand_bbox[3] = (maxy - miny + 1) * scale;
        diag->cand_area = (best_a.area + best_b.area) * scale * scale;
        diag->cand_fill = 0.5f * (best_a.fill + best_b.fill);
        diag->stage = 0;
        diag->best_hamming = 0;
        diag->best_contrast = best_sc;
    }
    fill_obs_from_pair(&best_a, &best_b, (float)scale, gray, w, h, stride, out);
    return true;
}


/* ================= 双码融合 (两孔位各一枚 tag36h11) =================
 * 复用单码连通域/解码流水线, 但不早停: 扫完全部候选, 收集 TAG_ID_A/B。
 * 融合: 中点 (u,v), 小ID->大ID 连线角 theta, 中心距 edge_a (= edge_p)。
 */
static void fuse_dual_obs(const TagObservation *a, const TagObservation *b,
                          TagObservation *out) {
    const TagObservation *lo = (a->id <= b->id) ? a : b;
    const TagObservation *hi = (a->id <= b->id) ? b : a;
    float dx = hi->u - lo->u, dy = hi->v - lo->v;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist < 1.0f) dist = 1.0f;
    float ux = dx / dist, uy = dy / dist;
    float px = -uy, py = ux;
    float half_w = 0.25f * (lo->edge_a + hi->edge_a);
    if (half_w < 4.0f) half_w = 4.0f;
    float half_l = 0.5f * dist;

    memset(out, 0, sizeof(*out));
    out->ok = true;
    out->id = lo->id;
    out->hamming = lo->hamming + hi->hamming;
    out->u = 0.5f * (lo->u + hi->u);
    out->v = 0.5f * (lo->v + hi->v);
    out->theta = atan2f(dy, dx);
    out->edge_a = dist;
    out->edge_p = dist;   /* 双码不做边长收缩, phi_shape~=0 */
    out->contrast = 0.5f * (lo->contrast + hi->contrast);
    out->margin = 0.5f * (lo->margin + hi->margin);
    out->corners[0][0] = out->u - ux * half_l - px * half_w;
    out->corners[0][1] = out->v - uy * half_l - py * half_w;
    out->corners[1][0] = out->u + ux * half_l - px * half_w;
    out->corners[1][1] = out->v + uy * half_l - py * half_w;
    out->corners[2][0] = out->u + ux * half_l + px * half_w;
    out->corners[2][1] = out->v + uy * half_l + py * half_w;
    out->corners[3][0] = out->u - ux * half_l + px * half_w;
    out->corners[3][1] = out->v - uy * half_l + py * half_w;
}

bool DualTagDetector_DetectEx(const uint8_t *gray, int w, int h, int stride,
                              TagObservation *out, TagDiag *diag) {
    memset(out, 0, sizeof(*out));
    out->ok = false;
    if (diag) {
        memset(diag, 0, sizeof(*diag));
        diag->best_hamming = 255;
        diag->stage = 1;
        diag->tags_found = 0;
        diag->found_id_a = -1;
        diag->found_id_b = -1;
    }
    if (!gray || w <= 0 || h <= 0) return false;
    if (TAG_ID_A == TAG_ID_B) return false;

    /* Phase A1: 连通域强制全分辨率; 角点精修/采码仍用原灰度 */
    const int scale = 1;
    const int cw = w / scale, ch = h / scale;
    if (!scratch_ensure(cw, ch)) return false;

    const uint8_t *cc_gray = gray;
    int cc_stride = stride;
    if (scale == 2) {
        for (int y = 0; y < ch; y++) {
            const uint8_t *r0 = gray + (size_t)(y * 2) * stride;
            const uint8_t *r1 = gray + (size_t)(y * 2 + 1) * stride;
            uint8_t *dst = g_sc.half + (size_t)y * cw;
            for (int x = 0; x < cw; x++) {
                int s = r0[x * 2] + r0[x * 2 + 1] + r1[x * 2] + r1[x * 2 + 1];
                dst[x] = (uint8_t)(s >> 2);
            }
        }
        cc_gray = g_sc.half;
        cc_stride = cw;
    }

    threshold_adaptive(cc_gray, cw, ch, cc_stride);
    fill_holes(cw, ch);
    label_components(cw, ch);
    if (diag) diag->ncomps = g_ncomps;
    if (g_ncomps == 0) return false;

    Cand *cands = g_sc.cands;
    if (!cands) return false;
    int ncand = 0;
    for (int i = 1; i < DET_MAX_COMPS && i < 65535; i++) {
        const CompStat *st = &g_sc.stats[i];
        if (st->area <= 0) continue;
        float bw = st->maxx - st->minx + 1, bh = st->maxy - st->miny + 1;
        if (bw < 0.7f * DET_EDGE_PX_MIN || bh < 0.7f * DET_EDGE_PX_MIN) {
            if (diag) diag->drop_small++;
            continue;
        }
        if (bw > 1.6f * DET_EDGE_PX_MAX || bh > 1.6f * DET_EDGE_PX_MAX) {
            if (diag) diag->drop_large++;
            continue;
        }
        double fill = (double)st->area / (bw * bh);
        if (fill < 0.15) { if (diag) diag->drop_fill++; continue; }
        double ar = bw / bh;
        if (ar < 0.4 || ar > 2.5) { if (diag) diag->drop_aspect++; continue; }
        cands[ncand].id = i;
        cands[ncand].area = st->area;
        ncand++;
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

    TagObservation got_a, got_b;
    bool have_a = false, have_b = false;
    memset(&got_a, 0, sizeof(got_a));
    memset(&got_b, 0, sizeof(got_b));

    for (int ci = 0; ci < ncand; ci++) {
        const CompStat *st = &g_sc.stats[cands[ci].id];
        float bw = st->maxx - st->minx + 1, bh = st->maxy - st->miny + 1;
        if (bw < 0.7f * DET_EDGE_PX_MIN || bh < 0.7f * DET_EDGE_PX_MIN) continue;
        if (bw > 1.6f * DET_EDGE_PX_MAX || bh > 1.6f * DET_EDGE_PX_MAX) continue;
        double fill = (double)st->area / (bw * bh);
        if (fill < 0.15) continue;
        double ar = bw / bh;
        if (ar < 0.4 || ar > 2.5) continue;
        if (diag) diag->decode_tried++;

        float P[4][2];
        float centroid[2] = {(float)((double)st->sx / st->area),
                             (float)((double)st->sy / st->area)};
        if (!comp_extremes(st, cands[ci].id, cw, ch, P)) continue;
        if (scale == 2) {
            for (int i = 0; i < 4; i++) { P[i][0] *= 2.0f; P[i][1] *= 2.0f; }
            centroid[0] *= 2.0f; centroid[1] *= 2.0f;
        }

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

        const float *base[2][4] = {
            {R[0], R[1], R[2], R[3]},
            {R[0], R[3], R[2], R[1]},
        };
        TagObservation best_o;
        int best_h = 255;
        memset(&best_o, 0, sizeof(best_o));
        for (int ori = 0; ori < 2; ori++) {
            for (int r = 0; r < 4; r++) {
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
                if (ham < best_h) { best_h = ham; best_o = o; }
            }
        }
        if (best_h > DET_MAX_HAMMING) continue;

        if (best_o.id == TAG_ID_A) {
            if (!have_a || best_h < got_a.hamming) { got_a = best_o; have_a = true; }
        } else if (best_o.id == TAG_ID_B) {
            if (!have_b || best_h < got_b.hamming) { got_b = best_o; have_b = true; }
        }
        if (have_a && have_b) break;
    }

    int nfound = (have_a ? 1 : 0) + (have_b ? 1 : 0);
    if (diag) {
        diag->tags_found = nfound;
        diag->found_id_a = have_a ? TAG_ID_A : -1;
        diag->found_id_b = have_b ? TAG_ID_B : -1;
    }
    if (nfound < 2) {
        if (diag) diag->stage = (diag->decode_tried > 0) ? 3 : 2;
        return false;
    }
    if (diag) diag->stage = 0;
    fuse_dual_obs(&got_a, &got_b, out);
    return true;
}

bool Landmark_DetectEx(const uint8_t *gray, int w, int h, int stride,
                       TagObservation *out, TagDiag *diag) {
#if LANDMARK_MODE == LANDMARK_DUAL_TAG
    return DualTagDetector_DetectEx(gray, w, h, stride, out, diag);
#elif LANDMARK_MODE == LANDMARK_CHASSIS
    return ChassisDetector_DetectEx(gray, w, h, stride, out, diag);
#else
    return TagDetector_DetectEx(gray, w, h, stride, out, diag);
#endif
}
