/* read_check.c: standalone reader of the v4 route library (routes.bin, map.bin; layouts in binary_v4.py).
 * Loads both with plain fread, prints the counts, re-does the hull sweep of every route against the hard
 * bit mask (bare 7.4 x 3.6 m hull and grown by 0.3 m, lattice every 0.125 m, at every sample along the
 * direction to the next sample), and prints the least clearance per file:
 *   routes.bin: least stored per-sample clearance byte (0.1 m steps);
 *   map.bin:    least clearance byte of the 0.5 m cell under any route sample.
 * usage: read_check.exe <routes.bin> <map.bin>
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static FILE *fp;
static void rd(void *p, size_t n)
{
    if (fread(p, 1, n, fp) != n) {
        fprintf(stderr, "short read\n");
        exit(1);
    }
}
static uint8_t u8(void) { uint8_t v; rd(&v, 1); return v; }
static uint16_t u16(void) { uint16_t v; rd(&v, 2); return v; }
static uint32_t u32(void) { uint32_t v; rd(&v, 4); return v; }
static float f32(void) { float v; rd(&v, 4); return v; }

typedef struct {
    float x0, z0, cell, cell05;
    uint32_t cols, rows, cols05, rows05, nb;
    uint8_t *bits, *clr05;
} map_t;

static int hard_at(const map_t *m, double x, double z)
{
    long c = (long)floor((x - m->x0) / m->cell), r = (long)floor((z - m->z0) / m->cell);
    if (c < 0) c = 0;
    if (r < 0) r = 0;
    if (c >= (long)m->cols) c = m->cols - 1;
    if (r >= (long)m->rows) r = m->rows - 1;
    return (m->bits[(size_t)r * m->nb + (c >> 3)] >> (c & 7)) & 1;
}

/* number of samples whose hull rectangle (half extents hl, hw) contains a hard cell */
static long sweep(const map_t *m, const float *xz, uint32_t n, double hl, double hw)
{
    long hits = 0;
    for (uint32_t i = 0; i < n; i++) {
        uint32_t a = i + 1 < n ? i : (n > 1 ? n - 2 : 0), b = i + 1 < n ? i + 1 : (n > 1 ? n - 1 : 0);
        double h = atan2((double)xz[2 * b + 1] - xz[2 * a + 1], (double)xz[2 * b] - xz[2 * a]);
        double hx = cos(h), hz = sin(h);
        int hit = 0;
        for (int iu = 0; !hit; iu++) {
            double u = -hl + iu * 0.125;
            if (u > hl + 1e-6) break;
            for (int iv = 0; !hit; iv++) {
                double v = -hw + iv * 0.125;
                if (v > hw + 1e-6) break;
                hit = hard_at(m, xz[2 * i] + u * hx - v * hz, xz[2 * i + 1] + u * hz + v * hx);
            }
        }
        hits += hit;
    }
    return hits;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: read_check <routes.bin> <map.bin>\n");
        return 2;
    }
    /* map.bin */
    map_t m;
    char magic[9] = {0};
    fp = fopen(argv[2], "rb");
    if (!fp) return 1;
    rd(magic, 8);
    if (strcmp(magic, "WTMAP004")) { fprintf(stderr, "bad map magic\n"); return 1; }
    m.x0 = f32(); m.z0 = f32(); m.cell = f32(); m.cell05 = f32();
    m.cols = u32(); m.rows = u32(); m.cols05 = u32(); m.rows05 = u32();
    m.nb = (m.cols + 7) / 8;
    m.bits = malloc((size_t)m.rows * m.nb);
    m.clr05 = malloc((size_t)m.rows05 * m.cols05);
    rd(m.bits, (size_t)m.rows * m.nb);
    rd(m.clr05, (size_t)m.rows05 * m.cols05);
    fclose(fp);
    long nhard = 0;
    for (size_t i = 0; i < (size_t)m.rows * m.nb; i++)
        for (int k = 0; k < 8; k++) nhard += (m.bits[i] >> k) & 1;
    printf("map.bin: %u x %u cells of %.2f m from (%.2f, %.2f), %ld hard; clearance %u x %u at %.2f m\n",
           m.cols, m.rows, m.cell, m.x0, m.z0, nhard, m.cols05, m.rows05, m.cell05);
    /* routes.bin */
    fp = fopen(argv[1], "rb");
    if (!fp) return 1;
    rd(magic, 8);
    if (strcmp(magic, "WTRTE004")) { fprintf(stderr, "bad routes magic\n"); return 1; }
    uint32_t ver = u32(), npo = u32(), nt = u32(), ns = u32(), nr = u32();
    char buf[64];
    for (uint32_t i = 0; i < npo; i++) { rd(buf, 16); f32(); f32(); f32(); }
    uint32_t nzone = 0, npost[3] = {0, 0, 0}, noroom = 0, pkind[5] = {0, 0, 0, 0, 0};
    for (uint32_t i = 0; i < nt; i++) {
        rd(buf, 16);
        uint8_t kind = u8(), army = u8(), pk = u8(), standing = u8();
        for (int k = 0; k < 8; k++) f32();
        if (kind == 0) nzone++;
        else { npost[army % 3]++; pkind[pk % 5]++; noroom += !standing; }
    }
    for (uint32_t i = 0; i < ns; i++) { u16(); u16(); f32(); f32(); f32(); }
    rd((char *)malloc((size_t)npo * nt), (size_t)npo * nt);
    long kinds[4] = {0, 0, 0, 0}, gears[4] = {0, 0, 0, 0}, desig = 0, tangent = 0, samples = 0, bare = 0, grown = 0;
    int min_rclr = 255, min_mclr = 255;
    double len_sum = 0;
    for (uint32_t i = 0; i < nr; i++) {
        rd(buf, 32);
        uint8_t kind = u8(), rank = u8(), flags = u8(), fb = u8();
        u16(); u16(); u16();
        uint8_t gear = u8();
        u8();
        float tot = f32(), drv = f32(), turn = f32(), len = f32();
        uint32_t n = u32();
        float *xz = malloc(sizeof(float) * 2 * n);
        uint8_t *clr = malloc(n);
        float *kap = malloc(sizeof(float) * n);
        rd(xz, sizeof(float) * 2 * n);
        rd(clr, n);
        rd(kap, sizeof(float) * n);
        (void)rank; (void)fb; (void)tot; (void)drv; (void)turn;
        kinds[kind & 3]++;
        gears[gear & 3]++;
        desig += (flags >> 1) & 1;
        tangent += flags & 1;
        samples += n;
        len_sum += len;
        for (uint32_t k = 0; k < n; k++) {
            if (clr[k] < min_rclr) min_rclr = clr[k];
            long c = (long)floor((xz[2 * k] - m.x0) / m.cell05), r = (long)floor((xz[2 * k + 1] - m.z0) / m.cell05);
            if (c >= 0 && r >= 0 && c < (long)m.cols05 && r < (long)m.rows05) {
                int v = m.clr05[(size_t)r * m.cols05 + c];
                if (v < min_mclr) min_mclr = v;
            }
        }
        bare += sweep(&m, xz, n, 3.7, 1.8);
        grown += sweep(&m, xz, n, 4.0, 2.1);
        free(xz); free(clr); free(kap);
    }
    long pos = ftell(fp);
    fseek(fp, 0, SEEK_END);
    long end = ftell(fp);
    fclose(fp);
    printf("routes.bin v%u: %u poses, %u targets (%u zones, %u posts army 1, %u posts army 2; choke %u, second crew %u, "
           "overwatch %u, gate %u; %u without standing room), %u slots\n",
           ver, npo, nt, nzone, npost[1], npost[2], pkind[1], pkind[2], pkind[3], pkind[4], noroom, ns);
    printf("routes: %u (spawn->zone %ld, spawn->post %ld, zone<->zone %ld, zone<->post %ld), %ld designated, "
           "%ld tangent starts, start turn gear none/forward/reverse/blocked %ld/%ld/%ld/%ld, %ld samples, %.0f m; "
           "file fully read: %s\n", nr, kinds[0], kinds[1], kinds[2], kinds[3], desig, tangent, gears[0], gears[1],
           gears[2], gears[3], samples, len_sum, pos == end ? "yes" : "NO");
    printf("hull sweep: bare 7.4 x 3.6 m: %ld samples touch hard; grown by 0.3 m: %ld samples\n", bare, grown);
    printf("least clearance: routes.bin %.1f m (byte %d), map.bin under route samples %.1f m (byte %d)\n",
           min_rclr / 10.0, min_rclr, min_mclr / 10.0, min_mclr);
    return 0;
}
