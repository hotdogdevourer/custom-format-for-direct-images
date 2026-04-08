/*
 * CFDE - Custom Format for Direct ImagEs
 * C89 implementation
 *
 * Header Layout (64 bytes total):
 *  Offset | Size | Name        | Description
 *  0      | 16   | Magic       | "CFDE" + CF DE B0 AD BE 09 0A BE AF E4 5B 00
 *  16     | 4    | Width       | big-endian, >0
 *  20     | 4    | Height      | big-endian, >0
 *  24     | 1    | ColorType   | 0=Gray, 2=RGB, 3=Indexed, 4=Gray+Alpha, 6=RGBA
 *  25     | 1    | BitDepth    | 1,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32
 *  26     | 1    | Compression | 0-5
 *  27     | 1    | Filter      | 0=adaptive
 *  28     | 1    | Interlace   | 0-255 (0=none, N=rearrange rows by stride N)
 *  29     | 4    | CRC         | CRC-32 checksum
 *  33     | 4    | BlockSize   | big-endian, >0
 *  37     | 8    | BlockCount  | big-endian, >0
 *  45     | 8    | FileSize    | big-endian, >0
 *  53     | 8    | Timestamp   | creation timestamp
 *  61     | 3    | Padding     | 0D 0A 00
 *  64+    | var  | Chunks      | pixel data chunks
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long  uint32_t;
typedef signed   long  int32_t;

typedef struct { uint32_t hi; uint32_t lo; } uint64_s;

#define CFDE_HEADER_SIZE 64

static const uint8_t CFDE_MAGIC[16] = {
    'C','F','D','E',
    0xCF, 0xDE, 0xB0, 0xAD,
    0xBE, 0x09, 0x0A, 0xBE,
    0xAF, 0xE4, 0x5B, 0x00
};

#define CT_GRAY       0
#define CT_RGB        2
#define CT_INDEXED    3
#define CT_GRAY_ALPHA 4
#define CT_RGBA       6

#define VIEW_MODE_NONE    0
#define VIEW_MODE_FULL    1
#define VIEW_MODE_ANSI2   2
#define VIEW_MODE_ANSI8   3
#define VIEW_MODE_ANSI16  4
#define VIEW_MODE_ANSI256 5
#define VIEW_MODE_GRAY    6

#define LAYOUT_NONE          0
#define LAYOUT_HALF_VERT     1
#define LAYOUT_FULL_STRETCHED 2
#define LAYOUT_ASPECT        3
#define FLUSH_NONE        0
#define FLUSH_BUFFER      1
#define FLUSH_LINE        2
#define FLUSH_PIXEL       3

#define COMP_NONE  0
#define COMP_MAX   5

#define CFDE_COMP_RLE     0x01
#define CFDE_COMP_DELTA   0x02
#define CFDE_COMP_LZ77    0x04
#define CFDE_COMP_HUFFMAN 0x08
#define CFDE_COMP_XOR     0x10

static uint32_t read_u32be(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] <<  8) |  (uint32_t)p[3];
}




static uint32_t crc32_table[256];
static int crc32_ready = 0;

static void crc32_init(void) {
    uint32_t i, j, c;
    if (crc32_ready) return;
    for (i = 0; i < 256; i++) {
        c = i;
        for (j = 0; j < 8; j++)
            c = (c & 1) ? (0xEDB88320UL ^ (c >> 1)) : (c >> 1);
        crc32_table[i] = c;
    }
    crc32_ready = 1;
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *buf, uint32_t len) {
    uint32_t i;
    crc32_init();
    crc ^= 0xFFFFFFFFUL;
    for (i = 0; i < len; i++)
        crc = crc32_table[(crc ^ buf[i]) & 0xFF] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFUL;
}


#define RLE_MIN_RUN  3
#define RLE_MAX_RUN  130
#define RLE_MAX_LIT  128

static uint8_t *cfde_rle_encode(const uint8_t *in, uint32_t in_len,
                                 uint32_t *out_len) {
    uint32_t cap = in_len * 2 + 32;
    uint8_t *out = (uint8_t*)malloc(cap);
    uint32_t ip = 0, op = 0;

    if (!out) return NULL;

    while (ip < in_len) {
        uint32_t run = 1;
        while (run < RLE_MAX_RUN && ip + run < in_len &&
               in[ip + run] == in[ip])
            run++;

        if (run >= (uint32_t)RLE_MIN_RUN) {
            if (op + 2 >= cap) {
                uint8_t *tmp;
                cap *= 2;
                tmp = (uint8_t*)realloc(out, cap);
                if (!tmp) { free(out); return NULL; }
                out = tmp;
            }
            out[op++] = (uint8_t)(0x80 | (uint8_t)(run - 1));
            out[op++] = in[ip];
            ip += run;
        } else {
            uint32_t lit = 0;
            uint32_t ls = ip;
            while (lit < RLE_MAX_LIT && ip < in_len) {
                uint32_t r2 = 1;
                while (r2 < RLE_MIN_RUN && ip + r2 < in_len &&
                       in[ip + r2] == in[ip])
                    r2++;
                if (r2 >= (uint32_t)RLE_MIN_RUN && lit > 0) break;
                lit++; ip++;
            }
            if (op + lit + 1 >= cap) {
                uint8_t *tmp;
                cap = cap * 2 + lit + 4;
                tmp = (uint8_t*)realloc(out, cap);
                if (!tmp) { free(out); return NULL; }
                out = tmp;
            }
            out[op++] = (uint8_t)(lit - 1);
            memcpy(out + op, in + ls, lit);
            op += lit;
        }
    }
    *out_len = op;
    return out;
}

static uint8_t *cfde_rle_decode(const uint8_t *in, uint32_t in_len,
                                 uint32_t *out_len) {
    uint32_t cap = in_len * 4 + 64;
    uint8_t *out = (uint8_t*)malloc(cap);
    uint32_t ip = 0, op = 0;

    if (!out) return NULL;

    while (ip < in_len) {
        uint8_t ctrl = in[ip++];
        if (ctrl & 0x80) {
            uint32_t run = (uint32_t)(ctrl & 0x7F) + 1;
            uint8_t val;
            if (ip >= in_len) break;
            val = in[ip++];
            if (op + run >= cap) {
                uint8_t *tmp;
                cap = cap * 2 + run + 4;
                tmp = (uint8_t*)realloc(out, cap);
                if (!tmp) { free(out); return NULL; }
                out = tmp;
            }
            memset(out + op, val, run);
            op += run;
        } else {
            uint32_t lit = (uint32_t)ctrl + 1;
            if (ip + lit > in_len) lit = in_len - ip;
            if (op + lit >= cap) {
                uint8_t *tmp;
                cap = cap * 2 + lit + 4;
                tmp = (uint8_t*)realloc(out, cap);
                if (!tmp) { free(out); return NULL; }
                out = tmp;
            }
            memcpy(out + op, in + ip, lit);
            op += lit; ip += lit;
        }
    }
    *out_len = op;
    return out;
}

static void cfde_delta_encode(uint8_t *buf, uint32_t len) {
    uint32_t i;
    uint8_t prev = 0, cur;
    for (i = 0; i < len; i++) {
        cur = buf[i];
        buf[i] = (uint8_t)((cur - prev) & 0xFF);
        prev = cur;
    }
}

static void cfde_delta_decode(uint8_t *buf, uint32_t len) {
    uint32_t i;
    uint8_t acc = 0;
    for (i = 0; i < len; i++) {
        acc = (uint8_t)((acc + buf[i]) & 0xFF);
        buf[i] = acc;
    }
}

static void cfde_xor_encode(uint8_t *buf, uint32_t len) {
    uint32_t i;
    for (i = len - 1; i >= 1; i--)
        buf[i] ^= buf[i - 1];
}

static void cfde_xor_decode(uint8_t *buf, uint32_t len) {
    uint32_t i;
    for (i = 1; i < len; i++)
        buf[i] ^= buf[i - 1];
}

#define LZ77_MIN_MATCH 3
#define LZ77_MAX_MATCH 130

static uint8_t *cfde_lz77_encode(const uint8_t *in, uint32_t in_len,
                                   uint32_t *out_len, uint32_t window) {
    uint32_t cap = in_len * 2 + 16;
    uint8_t *out = (uint8_t*)malloc(cap);
    uint32_t ip = 0, op = 0;

    if (!out) return NULL;
    if (window == 0) window = 4096;

    while (ip < in_len) {
        uint32_t best_len = 0, best_off = 0;
        uint32_t win_start = (ip > window) ? ip - window : 0;
        uint32_t j;

        for (j = win_start; j < ip; j++) {
            uint32_t mlen = 0;
            while (mlen < LZ77_MAX_MATCH && ip + mlen < in_len &&
                   in[j + mlen] == in[ip + mlen])
                mlen++;
            if (mlen > best_len) { best_len = mlen; best_off = ip - j; }
        }

        if (best_len >= (uint32_t)LZ77_MIN_MATCH) {
            if (op + 3 >= cap) {
                uint8_t *tmp;
                cap = cap * 2 + 8;
                tmp = (uint8_t*)realloc(out, cap);
                if (!tmp) { free(out); return NULL; }
                out = tmp;
            }
            out[op++] = (uint8_t)(0x80 | (uint8_t)(best_len - LZ77_MIN_MATCH));
            out[op++] = (uint8_t)((best_off - 1) >> 8);
            out[op++] = (uint8_t)((best_off - 1) & 0xFF);
            ip += best_len;
        } else {
            if (op + 2 >= cap) {
                uint8_t *tmp;
                cap = cap * 2 + 8;
                tmp = (uint8_t*)realloc(out, cap);
                if (!tmp) { free(out); return NULL; }
                out = tmp;
            }
            out[op++] = 0x00;
            out[op++] = in[ip++];
        }
    }
    *out_len = op;
    return out;
}

static uint8_t *cfde_lz77_decode(const uint8_t *in, uint32_t in_len,
                                   uint32_t *out_len) {
    uint32_t cap = in_len * 4 + 64;
    uint8_t *out = (uint8_t*)malloc(cap);
    uint32_t ip = 0, op = 0;

    if (!out) return NULL;

    while (ip < in_len) {
        uint8_t flag = in[ip++];
        if (flag & 0x80) {
            uint32_t mlen, offset, src, k;
            if (ip + 1 >= in_len) break;
            mlen   = (uint32_t)(flag & 0x7F) + LZ77_MIN_MATCH;
            offset = ((uint32_t)in[ip] << 8) | in[ip + 1];
            ip += 2;
            offset += 1;
            if (op < offset) break;
            src = op - offset;
            if (op + mlen >= cap) {
                uint8_t *tmp;
                cap = cap * 2 + mlen + 4;
                tmp = (uint8_t*)realloc(out, cap);
                if (!tmp) { free(out); return NULL; }
                out = tmp;
            }
            for (k = 0; k < mlen; k++)
                out[op++] = out[src + k];
        } else {
            if (ip >= in_len) break;
            if (op + 1 >= cap) {
                uint8_t *tmp;
                cap = cap * 2 + 4;
                tmp = (uint8_t*)realloc(out, cap);
                if (!tmp) { free(out); return NULL; }
                out = tmp;
            }
            out[op++] = in[ip++];
        }
    }
    *out_len = op;
    return out;
}

#define HPOOL_MAX 512
typedef struct { uint32_t freq; int sym; int left; int right; } HNode;

static int hpool_sz;
static HNode hpool[HPOOL_MAX];
static int heap_arr[HPOOL_MAX];
static int heap_sz2;
static uint8_t  huff_len2[256];
static uint32_t huff_code2[256];

static int hn_new2(uint32_t freq, int sym, int left, int right) {
    HNode *n = &hpool[hpool_sz];
    n->freq = freq; n->sym = sym; n->left = left; n->right = right;
    return hpool_sz++;
}

static void heap_push2(int idx) {
    int i = heap_sz2++;
    heap_arr[i] = idx;
    while (i > 0) {
        int par = (i - 1) / 2;
        if (hpool[heap_arr[par]].freq <= hpool[heap_arr[i]].freq) break;
        { int tmp = heap_arr[par]; heap_arr[par] = heap_arr[i]; heap_arr[i] = tmp; }
        i = par;
    }
}

static int heap_pop2(void) {
    int ret = heap_arr[0], i = 0;
    heap_arr[0] = heap_arr[--heap_sz2];
    for (;;) {
        int l = 2*i+1, r = 2*i+2, s = i;
        if (l < heap_sz2 && hpool[heap_arr[l]].freq < hpool[heap_arr[s]].freq) s = l;
        if (r < heap_sz2 && hpool[heap_arr[r]].freq < hpool[heap_arr[s]].freq) s = r;
        if (s == i) break;
        { int tmp = heap_arr[s]; heap_arr[s] = heap_arr[i]; heap_arr[i] = tmp; }
        i = s;
    }
    return ret;
}

static void assign_len2(int node, int depth) {
    if (hpool[node].sym >= 0) {
        huff_len2[hpool[node].sym] = (uint8_t)(depth > 0 ? depth : 1);
        return;
    }
    if (hpool[node].left  >= 0) assign_len2(hpool[node].left,  depth + 1);
    if (hpool[node].right >= 0) assign_len2(hpool[node].right, depth + 1);
}

static int sym_ord2[256];
static int cmp_sym2(const void *a, const void *b) {
    int ia = *(const int*)a, ib = *(const int*)b;
    if (huff_len2[ia] != huff_len2[ib]) return (int)huff_len2[ia] - (int)huff_len2[ib];
    return ia - ib;
}

static int build_huffman2(const uint8_t *data, uint32_t len) {
    uint32_t freq[256];
    int i;
    memset(freq, 0, sizeof(freq));
    memset(huff_len2, 0, sizeof(huff_len2));
    for (i = 0; i < (int)len; i++) freq[data[i]]++;
    hpool_sz = 0; heap_sz2 = 0;
    for (i = 0; i < 256; i++)
        if (freq[i]) heap_push2(hn_new2(freq[i], i, -1, -1));
    if (heap_sz2 == 0) return 0;
    if (heap_sz2 == 1) { huff_len2[hpool[heap_arr[0]].sym] = 1; }
    else {
        while (heap_sz2 > 1) {
            int a = heap_pop2(), b = heap_pop2();
            heap_push2(hn_new2(hpool[a].freq + hpool[b].freq, -1, a, b));
        }
        assign_len2(heap_arr[0], 0);
    }
    for (i = 0; i < 256; i++) if (huff_len2[i] > 16) huff_len2[i] = 16;
    for (i = 0; i < 256; i++) sym_ord2[i] = i;
    qsort(sym_ord2, 256, sizeof(int), cmp_sym2);
    {
        uint32_t code = 0; int prev = 0;
        for (i = 0; i < 256; i++) {
            int s = sym_ord2[i];
            if (!huff_len2[s]) continue;
            if (huff_len2[s] > prev) { code <<= (huff_len2[s] - prev); prev = huff_len2[s]; }
            huff_code2[s] = code++;
        }
    }
    { int n = 0; for (i = 0; i < 256; i++) if (huff_len2[i]) n++; return n; }
}

static uint8_t *cfde_huffman_encode(const uint8_t *in, uint32_t in_len,
                                     uint32_t *out_len) {
    int nsym, real_sym = 0, i;
    uint32_t cap, op;
    uint8_t *out;
    uint32_t bit_buf; int bit_cnt;

    nsym = build_huffman2(in, in_len);
    if (nsym <= 0) { *out_len = 0; return (uint8_t*)calloc(1,1); }
    for (i = 0; i < 256; i++) if (huff_len2[i]) real_sym++;
    nsym = real_sym;

    cap = 5 + (uint32_t)(nsym * 2) + in_len + 64;
    out = (uint8_t*)malloc(cap);
    if (!out) return NULL;
    op = 0;

    out[op++] = (uint8_t)(in_len >> 24);
    out[op++] = (uint8_t)(in_len >> 16);
    out[op++] = (uint8_t)(in_len >>  8);
    out[op++] = (uint8_t)(in_len      );
    out[op++] = (uint8_t)(nsym - 1);
    for (i = 0; i < 256; i++) {
        if (!huff_len2[i]) continue;
        out[op++] = (uint8_t)i;
        out[op++] = huff_len2[i];
    }

    bit_buf = 0; bit_cnt = 0;
    for (i = 0; i < (int)in_len; i++) {
        uint8_t sym = in[i];
        int clen = (int)huff_len2[sym];
        uint32_t code = huff_code2[sym];
        int b;
        for (b = clen - 1; b >= 0; b--) {
            bit_buf = (bit_buf << 1) | ((code >> b) & 1u);
            if (++bit_cnt == 8) {
                if (op >= cap) {
                    uint8_t *tmp; cap = cap * 2 + 16;
                    tmp = (uint8_t*)realloc(out, cap);
                    if (!tmp) { free(out); return NULL; }
                    out = tmp;
                }
                out[op++] = (uint8_t)bit_buf;
                bit_buf = 0; bit_cnt = 0;
            }
        }
    }
    if (bit_cnt > 0) {
        if (op >= cap) {
            uint8_t *tmp = (uint8_t*)realloc(out, cap + 4);
            if (!tmp) { free(out); return NULL; }
            out = tmp;
        }
        out[op++] = (uint8_t)(bit_buf << (8 - bit_cnt));
    }
    *out_len = op;
    return out;
}

static uint8_t *cfde_huffman_decode(const uint8_t *in, uint32_t in_len,
                                     uint32_t *out_len) {
    uint32_t orig_len, ip = 0, op = 0;
    int nsym, i, ndec = 0;
    uint8_t  dec_sym[256], dec_len[256];
    uint32_t dec_code[256];
    uint8_t *out;
    uint8_t  bit_buf; int bit_pos;
    uint32_t byte_ip;

    if (in_len < 5) return NULL;
    orig_len = ((uint32_t)in[0]<<24)|((uint32_t)in[1]<<16)|((uint32_t)in[2]<<8)|in[3];
    nsym = (int)in[4] + 1; ip = 5;

    for (i = 0; i < nsym && ip + 1 <= in_len; i++) {
        dec_sym[ndec] = in[ip++]; dec_len[ndec] = in[ip++]; ndec++;
    }
    {
        int j, k; uint8_t tl, ts;
        for (j = 0; j < ndec - 1; j++)
            for (k = j + 1; k < ndec; k++)
                if (dec_len[j] > dec_len[k] ||
                    (dec_len[j] == dec_len[k] && dec_sym[j] > dec_sym[k])) {
                    tl = dec_len[j]; dec_len[j] = dec_len[k]; dec_len[k] = tl;
                    ts = dec_sym[j]; dec_sym[j] = dec_sym[k]; dec_sym[k] = ts;
                }
    }
    { uint32_t code = 0; int prev = 0;
      for (i = 0; i < ndec; i++) {
          if (dec_len[i] > prev) { code <<= (dec_len[i] - prev); prev = dec_len[i]; }
          dec_code[i] = code++;
      }
    }

    out = (uint8_t*)malloc(orig_len + 1);
    if (!out) return NULL;

    byte_ip = (uint32_t)ip;
    bit_buf = (byte_ip < in_len) ? in[byte_ip++] : 0;
    bit_pos = 0;

    while (op < orig_len) {
        uint32_t accum = 0; int accum_len = 0, found = -1;
        while (found < 0) {
            int bit;
            if (bit_pos >= 8) {
                if (byte_ip >= in_len) break;
                bit_buf = in[byte_ip++]; bit_pos = 0;
            }
            bit = (bit_buf >> (7 - bit_pos)) & 1; bit_pos++;
            accum = (accum << 1) | (uint32_t)bit; accum_len++;
            for (i = 0; i < ndec; i++)
                if ((int)dec_len[i] == accum_len && dec_code[i] == accum) { found = i; break; }
            if (accum_len >= 16 && found < 0) break;
        }
        if (found < 0) break;
        out[op++] = dec_sym[found];
    }
    *out_len = op;
    return out;
}

static uint8_t *cfde_compress(const uint8_t *in, uint32_t in_len,
                               uint32_t *out_len, int level,
                               uint8_t *flags_out) {
    uint8_t *work, *tmp2;
    uint32_t work_len;
    uint8_t flags = 0;
    int do_rle, do_delta, do_xor, do_lz77, do_huffman;
    uint32_t lz_window;

    do_rle = do_delta = do_xor = do_lz77 = do_huffman = 0;
    lz_window = 0;

    switch (level) {
    case 0:                                                         break;
    case 1: do_rle = 1;                                             break;
    case 2: do_delta = 1; do_rle = 1;                              break;
    case 3: do_delta = 1; do_rle = 1; do_lz77 = 1; lz_window = 1024; break;
    case 4: do_delta = 1; do_xor = 1; do_rle = 1; do_lz77 = 1; lz_window = 8192; break;
    default:
        do_delta = 1; do_xor = 1; do_rle = 1;
        do_lz77 = 1; lz_window = 32768; do_huffman = 1;            break;
    }

    *flags_out = 0;

    work = (uint8_t*)malloc(in_len ? in_len : 1);
    if (!work) return NULL;
    if (in_len) memcpy(work, in, in_len);
    work_len = in_len;

    if (do_delta) { cfde_delta_encode(work, work_len); flags |= CFDE_COMP_DELTA; }
    if (do_xor)   { cfde_xor_encode(work, work_len);   flags |= CFDE_COMP_XOR;   }

    if (do_rle) {
        uint32_t rlen = 0;
        uint8_t *rout = cfde_rle_encode(work, work_len, &rlen);
        if (rout && rlen < work_len) {
            free(work); work = rout; work_len = rlen; flags |= CFDE_COMP_RLE;
        } else { if (rout) free(rout); }
    }

    if (do_lz77) {
        uint32_t llen = 0;
        uint8_t *lout = cfde_lz77_encode(work, work_len, &llen, lz_window);
        if (lout && llen < work_len) {
            free(work); work = lout; work_len = llen; flags |= CFDE_COMP_LZ77;
        } else { if (lout) free(lout); }
    }

    if (do_huffman) {
        uint32_t hlen = 0;
        uint8_t *hout = cfde_huffman_encode(work, work_len, &hlen);
        if (hout && hlen < work_len) {
            free(work); work = hout; work_len = hlen; flags |= CFDE_COMP_HUFFMAN;
        } else { if (hout) free(hout); }
    }

    if (work_len >= in_len) {
        free(work);
        work = (uint8_t*)malloc(in_len ? in_len : 1);
        if (!work) return NULL;
        if (in_len) memcpy(work, in, in_len);
        work_len = in_len; flags = 0;
    }

    (void)tmp2;
    *flags_out = flags;
    *out_len   = work_len;
    return work;
}

static uint8_t *cfde_decompress(const uint8_t *in, uint32_t in_len,
                                 uint8_t flags, uint32_t *out_len) {
    uint8_t *work; uint32_t work_len = in_len;

    if (flags & CFDE_COMP_HUFFMAN) {
        work = cfde_huffman_decode(in, in_len, &work_len);
        if (!work) { fprintf(stderr, "X Huffman decode failed\n"); return NULL; }
    } else {
        work = (uint8_t*)malloc(in_len ? in_len : 1);
        if (!work) return NULL;
        if (in_len) memcpy(work, in, in_len);
        work_len = in_len;
    }

    if (flags & CFDE_COMP_LZ77) {
        uint8_t *lz; uint32_t ll = 0;
        lz = cfde_lz77_decode(work, work_len, &ll);
        free(work);
        if (!lz) { fprintf(stderr, "X LZ77 decode failed\n"); return NULL; }
        work = lz; work_len = ll;
    }

    if (flags & CFDE_COMP_RLE) {
        uint8_t *rr; uint32_t rl = 0;
        rr = cfde_rle_decode(work, work_len, &rl);
        free(work);
        if (!rr) { fprintf(stderr, "X RLE decode failed\n"); return NULL; }
        work = rr; work_len = rl;
    }

    if (flags & CFDE_COMP_XOR)   cfde_xor_decode(work, work_len);
    if (flags & CFDE_COMP_DELTA) cfde_delta_decode(work, work_len);

    *out_len = work_len;
    return work;
}


static uint32_t row_bytes(uint32_t width, int bpp, int channels) {
    uint32_t bits = width * (uint32_t)bpp * (uint32_t)channels;
    return (bits + 7) / 8;
}

static void pack_row(const uint32_t *samples, uint32_t count, int bpp, uint8_t *buf) {
    uint32_t total_bits = count * (uint32_t)bpp;
    uint32_t nbytes = (total_bits + 7) / 8;
    uint32_t i;
    uint32_t bit_pos = 0;

    memset(buf, 0, nbytes);

    for (i = 0; i < count; i++) {
        uint32_t val = samples[i];
        int b;
        for (b = bpp - 1; b >= 0; b--) {
            uint32_t byte_idx = bit_pos / 8;
            int      bit_off  = 7 - (int)(bit_pos % 8);
            if ((val >> b) & 1)
                buf[byte_idx] |= (uint8_t)(1u << bit_off);
            bit_pos++;
        }
    }
}

static void unpack_row(const uint8_t *buf, uint32_t count, int bpp, uint32_t *samples) {
    uint32_t i;
    uint32_t bit_pos = 0;
    uint32_t mask = (bpp == 32) ? 0xFFFFFFFFUL : ((1UL << bpp) - 1);

    for (i = 0; i < count; i++) {
        uint32_t val = 0;
        int b;
        for (b = bpp - 1; b >= 0; b--) {
            uint32_t byte_idx = bit_pos / 8;
            int      bit_off  = 7 - (int)(bit_pos % 8);
            if ((buf[byte_idx] >> bit_off) & 1)
                val |= (1UL << b);
            bit_pos++;
        }
        samples[i] = val & mask;
    }
}

static void interlace_rows(uint8_t *data, uint32_t width, uint32_t height,
                           int bpp, int channels, uint8_t interlace) {
    uint32_t rb = row_bytes(width, bpp, channels);
    uint8_t *tmp;
    uint32_t y, src_y, row_idx;

    if (interlace <= 1 || height == 0 || rb == 0) return;

    tmp = (uint8_t*)malloc(height * rb);
    if (!tmp) return;
    memcpy(tmp, data, height * rb);

    row_idx = 0;
    for (y = 0; y < interlace; y++) {
        for (src_y = y; src_y < height; src_y += interlace) {
            if (row_idx < height) {
                memcpy(data + row_idx * rb, tmp + src_y * rb, rb);
                row_idx++;
            }
        }
    }
    free(tmp);
}

static void deinterlace_rows(uint8_t *data, uint32_t width, uint32_t height,
                             int bpp, int channels, uint8_t interlace) {
    uint32_t rb = row_bytes(width, bpp, channels);
    uint8_t *tmp;
    uint32_t y, src_y, row_idx;

    if (interlace <= 1 || height == 0 || rb == 0) return;

    tmp = (uint8_t*)malloc(height * rb);
    if (!tmp) return;
    memcpy(tmp, data, height * rb);

    row_idx = 0;
    for (y = 0; y < interlace; y++) {
        for (src_y = y; src_y < height; src_y += interlace) {
            if (row_idx < height) {
                memcpy(data + src_y * rb, tmp + row_idx * rb, rb);
                row_idx++;
            }
        }
    }
    free(tmp);
}

static int channels_for_type(int color_type) {
    switch (color_type) {
    case CT_GRAY:       return 1;
    case CT_RGB:        return 3;
    case CT_INDEXED:    return 1;
    case CT_GRAY_ALPHA: return 2;
    case CT_RGBA:       return 4;
    default:            return 1;
    }
}

typedef struct {
    uint32_t width;
    uint32_t height;
    uint8_t  color_type;
    uint8_t  bit_depth;
    uint8_t  compression;
    uint8_t  filter;
    uint8_t  interlace;
    uint32_t block_size;
    uint8_t *pixels;
    uint32_t pixels_len;
} CFDEImage;

static void cfde_image_free(CFDEImage *img) {
    if (img && img->pixels) { free(img->pixels); img->pixels = NULL; }
}

static int cfde_write(const char *path, const CFDEImage *img) {
    FILE *f;
    uint8_t header[CFDE_HEADER_SIZE];
    uint8_t *comp_data = NULL;
    uint32_t comp_len = 0;
    uint8_t comp_flags = 0;
    uint32_t block_size;
    uint32_t block_count;
    uint32_t crc;
    time_t ts;
    uint32_t ts_hi, ts_lo;
    uint32_t file_size_hi, file_size_lo;
    uint32_t bc_hi, bc_lo;
    uint32_t i;

    {
        uint8_t *src_pixels = img->pixels;
        uint32_t src_len = img->pixels_len;
        uint8_t *interlaced = NULL;

        if (img->interlace > 1) {
            int ch = channels_for_type((int)img->color_type);
            interlaced = (uint8_t*)malloc(src_len ? src_len : 1);
            if (!interlaced) return 0;
            memcpy(interlaced, src_pixels, src_len);
            interlace_rows(interlaced, img->width, img->height,
                          (int)img->bit_depth, ch, img->interlace);
            src_pixels = interlaced;
        }

        if (img->compression > 0) {
            comp_data = cfde_compress(src_pixels, src_len,
                                      &comp_len, (int)img->compression, &comp_flags);
        } else {
            comp_data = (uint8_t*)malloc(src_len ? src_len : 1);
            if (comp_data && src_len) memcpy(comp_data, src_pixels, src_len);
            comp_len = src_len;
            comp_flags = 0;
        }

        if (interlaced) free(interlaced);
        if (!comp_data) return 0;
    }

    block_size = img->block_size > 0 ? img->block_size : 65536;
    block_count = (comp_len + block_size - 1) / block_size;
    if (block_count == 0 && comp_len == 0) block_count = 0;

    {
        uint32_t total = (uint32_t)CFDE_HEADER_SIZE + 1 + comp_len;
        file_size_hi = 0;
        file_size_lo = total;
    }

    ts = time(NULL);
    ts_hi = 0;
    ts_lo = (uint32_t)(unsigned long)ts;

    bc_hi = 0; bc_lo = block_count;

    memset(header, 0, CFDE_HEADER_SIZE);
    memcpy(header, CFDE_MAGIC, 16);

    header[16] = (uint8_t)((img->width  >> 24) & 0xFF);
    header[17] = (uint8_t)((img->width  >> 16) & 0xFF);
    header[18] = (uint8_t)((img->width  >>  8) & 0xFF);
    header[19] = (uint8_t)( img->width         & 0xFF);
    header[20] = (uint8_t)((img->height >> 24) & 0xFF);
    header[21] = (uint8_t)((img->height >> 16) & 0xFF);
    header[22] = (uint8_t)((img->height >>  8) & 0xFF);
    header[23] = (uint8_t)( img->height        & 0xFF);

    header[24] = img->color_type;
    header[25] = img->bit_depth;
    header[26] = img->compression;
    header[27] = img->filter;
    header[28] = img->interlace;

    header[29] = 0; header[30] = 0; header[31] = 0; header[32] = 0;

    header[33] = (uint8_t)((block_size >> 24) & 0xFF);
    header[34] = (uint8_t)((block_size >> 16) & 0xFF);
    header[35] = (uint8_t)((block_size >>  8) & 0xFF);
    header[36] = (uint8_t)( block_size        & 0xFF);

    header[37] = (uint8_t)((bc_hi >> 24) & 0xFF);
    header[38] = (uint8_t)((bc_hi >> 16) & 0xFF);
    header[39] = (uint8_t)((bc_hi >>  8) & 0xFF);
    header[40] = (uint8_t)( bc_hi        & 0xFF);
    header[41] = (uint8_t)((bc_lo >> 24) & 0xFF);
    header[42] = (uint8_t)((bc_lo >> 16) & 0xFF);
    header[43] = (uint8_t)((bc_lo >>  8) & 0xFF);
    header[44] = (uint8_t)( bc_lo        & 0xFF);

    header[45] = (uint8_t)((file_size_hi >> 24) & 0xFF);
    header[46] = (uint8_t)((file_size_hi >> 16) & 0xFF);
    header[47] = (uint8_t)((file_size_hi >>  8) & 0xFF);
    header[48] = (uint8_t)( file_size_hi        & 0xFF);
    header[49] = (uint8_t)((file_size_lo >> 24) & 0xFF);
    header[50] = (uint8_t)((file_size_lo >> 16) & 0xFF);
    header[51] = (uint8_t)((file_size_lo >>  8) & 0xFF);
    header[52] = (uint8_t)( file_size_lo        & 0xFF);

    header[53] = (uint8_t)((ts_hi >> 24) & 0xFF);
    header[54] = (uint8_t)((ts_hi >> 16) & 0xFF);
    header[55] = (uint8_t)((ts_hi >>  8) & 0xFF);
    header[56] = (uint8_t)( ts_hi        & 0xFF);
    header[57] = (uint8_t)((ts_lo >> 24) & 0xFF);
    header[58] = (uint8_t)((ts_lo >> 16) & 0xFF);
    header[59] = (uint8_t)((ts_lo >>  8) & 0xFF);
    header[60] = (uint8_t)( ts_lo        & 0xFF);

    header[61] = 0x0D;
    header[62] = 0x0A;
    header[63] = 0x00;

    crc = crc32_update(0, header, 29);
    header[29] = (uint8_t)((crc >> 24) & 0xFF);
    header[30] = (uint8_t)((crc >> 16) & 0xFF);
    header[31] = (uint8_t)((crc >>  8) & 0xFF);
    header[32] = (uint8_t)( crc        & 0xFF);

    f = fopen(path, "wb");
    if (!f) { free(comp_data); fprintf(stderr, "X Cannot open: %s\n", path); return 0; }

    fwrite(header, 1, CFDE_HEADER_SIZE, f);
    fputc((int)comp_flags, f);

    for (i = 0; i < comp_len; ) {
        uint32_t chunk = comp_len - i;
        if (chunk > block_size) chunk = block_size;
        fwrite(comp_data + i, 1, chunk, f);
        i += chunk;
    }

    fclose(f);
    free(comp_data);

    printf("! Written: %s (%lux%lu, %dbpp, ct=%d, comp=%d, interlace=%d)\n",
           path, (unsigned long)img->width, (unsigned long)img->height,
           (int)img->bit_depth, (int)img->color_type, (int)img->compression,
           (int)img->interlace);
    return 1;
}

static int cfde_read(const char *path, CFDEImage *img) {
    FILE *f;
    uint8_t header[CFDE_HEADER_SIZE];
    uint8_t comp_flags;
    uint32_t crc_stored, crc_calc;
    long data_start;
    long data_len;
    uint8_t *raw_data, *decomp;
    uint32_t decomp_len = 0;

    f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "X Cannot open: %s\n", path); return 0; }

    if (fread(header, 1, CFDE_HEADER_SIZE, f) != CFDE_HEADER_SIZE) {
        fprintf(stderr, "X Short header\n"); fclose(f); return 0;
    }
    if (memcmp(header, CFDE_MAGIC, 16) != 0) {
        fprintf(stderr, "X Invalid CFDE magic\n"); fclose(f); return 0;
    }

    crc_stored = read_u32be(header + 29);
    crc_calc   = crc32_update(0, header, 29);
    if (crc_stored != crc_calc) {
        fprintf(stderr, "! Warning: CRC mismatch (stored=%08lX calc=%08lX)\n",
                (unsigned long)crc_stored, (unsigned long)crc_calc);
    }

    img->width       = read_u32be(header + 16);
    img->height      = read_u32be(header + 20);
    img->color_type  = header[24];
    img->bit_depth   = header[25];
    img->compression = header[26];
    img->filter      = header[27];
    img->interlace   = header[28];
    img->block_size  = read_u32be(header + 33);

    { int fb = fgetc(f); comp_flags = (fb == EOF) ? 0 : (uint8_t)fb; }

    data_start = ftell(f);
    fseek(f, 0, SEEK_END);
    data_len = ftell(f) - data_start;
    fseek(f, data_start, SEEK_SET);

    raw_data = (uint8_t*)malloc((size_t)(data_len > 0 ? data_len : 1));
    if (!raw_data) { fclose(f); return 0; }
    if (data_len > 0 && fread(raw_data, 1, (size_t)data_len, f) != (size_t)data_len) {
        fprintf(stderr, "X Read error\n"); free(raw_data); fclose(f); return 0;
    }
    fclose(f);

    if (comp_flags) {
        decomp = cfde_decompress(raw_data, (uint32_t)data_len, comp_flags, &decomp_len);
        free(raw_data);
        if (!decomp) return 0;
        img->pixels     = decomp;
        img->pixels_len = decomp_len;
    } else {
        img->pixels     = raw_data;
        img->pixels_len = (uint32_t)data_len;
    }

    if (img->interlace > 1) {
        int ch = channels_for_type((int)img->color_type);
        deinterlace_rows(img->pixels, img->width, img->height,
                        (int)img->bit_depth, ch, img->interlace);
    }

    return 1;
}


static uint8_t *cfde_to_rgba8(const CFDEImage *img) {
    int channels = channels_for_type((int)img->color_type);
    uint32_t rb = row_bytes(img->width, (int)img->bit_depth, channels);
    uint32_t max_val;
    uint32_t y, x, c;
    uint8_t *out;
    uint32_t *samples;

    if (img->bit_depth == 0) return NULL;
    max_val = (img->bit_depth == 32) ? 0xFFFFFFFFUL : ((1UL << img->bit_depth) - 1);

    out = (uint8_t*)malloc(img->width * img->height * (uint32_t)channels);
    if (!out) return NULL;

    samples = (uint32_t*)malloc(img->width * (uint32_t)channels * sizeof(uint32_t));
    if (!samples) { free(out); return NULL; }

    for (y = 0; y < img->height; y++) {
        const uint8_t *row_src = img->pixels + y * rb;
        uint8_t *row_dst = out + y * img->width * (uint32_t)channels;

        unpack_row(row_src, img->width * (uint32_t)channels,
                   (int)img->bit_depth, samples);

        for (x = 0; x < img->width; x++) {
            for (c = 0; c < (uint32_t)channels; c++) {
                uint32_t v = samples[x * (uint32_t)channels + c];
                uint32_t v8 = (max_val > 0) ? (uint32_t)((double)v / (double)max_val * 255.0 + 0.5) : 0;
                if (v8 > 255) v8 = 255;
                row_dst[x * (uint32_t)channels + c] = (uint8_t)v8;
            }
        }
    }
    free(samples);
    return out;
}

static uint8_t *rgba8_to_cfde_pixels(const uint8_t *rgba8, uint32_t width, uint32_t height,
                                      int channels, int bpp, uint32_t *out_len) {
    uint32_t rb = row_bytes(width, bpp, channels);
    uint32_t total = rb * height;
    uint8_t *out;
    uint32_t max_val;
    uint32_t *samples;
    uint32_t y, x, c;

    out = (uint8_t*)malloc(total ? total : 1);
    if (!out) return NULL;
    memset(out, 0, total);

    max_val = (bpp == 32) ? 0xFFFFFFFFUL : ((1UL << bpp) - 1);

    samples = (uint32_t*)malloc(width * (uint32_t)channels * sizeof(uint32_t));
    if (!samples) { free(out); return NULL; }

    for (y = 0; y < height; y++) {
        const uint8_t *row_src = rgba8 + y * width * (uint32_t)channels;
        uint8_t *row_dst = out + y * rb;

        for (x = 0; x < width; x++) {
            for (c = 0; c < (uint32_t)channels; c++) {
                uint8_t v8 = row_src[x * (uint32_t)channels + c];
                uint32_t v = (uint32_t)((double)v8 / 255.0 * (double)max_val + 0.5);
                if (v > max_val) v = max_val;
                samples[x * (uint32_t)channels + c] = v;
            }
        }
        pack_row(samples, width * (uint32_t)channels, bpp, row_dst);
    }

    free(samples);
    *out_len = total;
    return out;
}

static int read_ppm_skip_ws_comment(FILE *f) {
    int c;
    do {
        c = fgetc(f);
        if (c == '#') { while ((c = fgetc(f)) != '\n' && c != EOF); }
    } while (c == ' ' || c == '\t' || c == '\r' || c == '\n');
    return c;
}

static unsigned long read_ppm_uint(FILE *f, int first) {
    unsigned long v = (unsigned long)(first - '0');
    int c;
    while ((c = fgetc(f)) >= '0' && c <= '9') v = v * 10 + (unsigned long)(c - '0');
    return v;
}

static uint8_t *load_pnm(const char *path, uint32_t *w, uint32_t *h, int *channels) {
    FILE *f; int c;
    unsigned long W, H, maxv;
    uint8_t *data;
    size_t total;

    f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "X Cannot open PNM: %s\n", path); return NULL; }

    c = fgetc(f);
    if (c != 'P') { fclose(f); fprintf(stderr, "X Not a PNM file\n"); return NULL; }
    c = fgetc(f);
    if (c == '6') *channels = 3;
    else if (c == '5') *channels = 1;
    else { fclose(f); fprintf(stderr, "X Only P5/P6 PNM supported\n"); return NULL; }

    c = read_ppm_skip_ws_comment(f);
    W = read_ppm_uint(f, c);
    c = read_ppm_skip_ws_comment(f);
    H = read_ppm_uint(f, c);
    c = read_ppm_skip_ws_comment(f);
    maxv = read_ppm_uint(f, c);

    if (maxv != 255) {
        fprintf(stderr, "X Only 8-bit PNM supported (maxval=%lu)\n", maxv);
        fclose(f); return NULL;
    }

    total = (size_t)(W * H * (unsigned long)(*channels));
    data = (uint8_t*)malloc(total);
    if (!data) { fclose(f); return NULL; }
    if (fread(data, 1, total, f) != total) {
        fprintf(stderr, "X PNM read error\n"); free(data); fclose(f); return NULL;
    }
    fclose(f);
    *w = (uint32_t)W; *h = (uint32_t)H;
    return data;
}

static int save_pnm(const char *path, const uint8_t *data,
                    uint32_t w, uint32_t h, int channels) {
    FILE *f = fopen(path, "wb");
    if (!f) { fprintf(stderr, "X Cannot open output: %s\n", path); return 0; }
    fprintf(f, "P%d\n%lu %lu\n255\n", (channels == 3) ? 6 : 5,
            (unsigned long)w, (unsigned long)h);
    fwrite(data, 1, (size_t)(w * h * (uint32_t)channels), f);
    fclose(f);
    return 1;
}

static int cmd_from_raw(const char *inpath, const char *outpath,
                        uint32_t width, uint32_t height,
                        int color_type, int bpp, int compression, int interlace) {
    FILE *f;
    long file_size;
    uint8_t *raw;
    CFDEImage img;
    uint32_t expected;
    int channels = channels_for_type(color_type);

    f = fopen(inpath, "rb");
    if (!f) { fprintf(stderr, "X Cannot open raw: %s\n", inpath); return 0; }
    fseek(f, 0, SEEK_END); file_size = ftell(f); fseek(f, 0, SEEK_SET);
    raw = (uint8_t*)malloc((size_t)file_size);
    if (!raw) { fclose(f); return 0; }
    if (fread(raw, 1, (size_t)file_size, f) != (size_t)file_size) {
        fprintf(stderr, "X Read error\n");
        free(raw);
        fclose(f);
        return 0;
    }
    fclose(f);

    expected = row_bytes(width, bpp, channels) * height;

    memset(&img, 0, sizeof(img));
    img.width       = width;
    img.height      = height;
    img.color_type  = (uint8_t)color_type;
    img.bit_depth   = (uint8_t)bpp;
    img.compression = (uint8_t)compression;
    img.filter      = 0;
    img.interlace   = (uint8_t)interlace;
    img.block_size  = 65536;

    if ((uint32_t)file_size >= expected) {
        img.pixels     = raw;
        img.pixels_len = expected;
    } else {
        uint8_t *padded = (uint8_t*)calloc(expected, 1);
        if (!padded) { free(raw); return 0; }
        memcpy(padded, raw, (size_t)file_size);
        free(raw);
        img.pixels     = padded;
        img.pixels_len = expected;
    }

    printf("> Importing raw: %s (%lux%lu, ct=%d, %dbpp, interlace=%d)\n",
           inpath, (unsigned long)width, (unsigned long)height, color_type, bpp, interlace);
    { int r = cfde_write(outpath, &img); free(img.pixels); return r; }
}

static int cmd_pnm_to_cfde(const char *inpath, const char *outpath,
                            int bpp, int compression, int interlace) {
    uint32_t w = 0, h = 0;
    int channels = 0;
    uint8_t *rgba8;
    CFDEImage img;
    uint32_t plen;
    int color_type;
    int r;

    rgba8 = load_pnm(inpath, &w, &h, &channels);
    if (!rgba8) return 0;

    color_type = (channels == 3) ? CT_RGB : CT_GRAY;

    img.pixels = rgba8_to_cfde_pixels(rgba8, w, h, channels, bpp, &plen);
    free(rgba8);
    if (!img.pixels) return 0;

    img.width       = w;
    img.height      = h;
    img.color_type  = (uint8_t)color_type;
    img.bit_depth   = (uint8_t)bpp;
    img.compression = (uint8_t)compression;
    img.filter      = 0;
    img.interlace   = (uint8_t)interlace;
    img.block_size  = 65536;
    img.pixels_len  = plen;

    printf("> Converting PNM: %s -> %s (%lux%lu, ct=%d, %dbpp, comp=%d, interlace=%d)\n",
           inpath, outpath, (unsigned long)w, (unsigned long)h,
           color_type, bpp, compression, interlace);
    r = cfde_write(outpath, &img);
    free(img.pixels);
    return r;
}

static int cmd_cfde_to_pnm(const char *inpath, const char *outpath) {
    CFDEImage img;
    uint8_t *rgba8;
    int channels;
    int r;

    memset(&img, 0, sizeof(img));
    if (!cfde_read(inpath, &img)) return 0;

    channels = channels_for_type((int)img.color_type);
    rgba8 = cfde_to_rgba8(&img);
    cfde_image_free(&img);
    if (!rgba8) return 0;

    printf("> Exporting: %s -> %s (%lux%lu, ct=%d, %dbpp, interlace=%d)\n",
           inpath, outpath,
           (unsigned long)img.width, (unsigned long)img.height,
           (int)img.color_type, (int)img.bit_depth, (int)img.interlace);

    r = save_pnm(outpath, rgba8, img.width, img.height, channels);
    free(rgba8);
    return r;
}

static int cmd_create(const char *outpath,
                      uint32_t width, uint32_t height,
                      int color_type, int bpp, int compression, int interlace,
                      const char *type_str,
                      uint8_t r8, uint8_t g8, uint8_t b8) {
    int channels = channels_for_type(color_type);
    uint32_t plen;
    uint8_t *rgba8;
    uint8_t *cfde_pix;
    CFDEImage img;
    uint32_t x, y;
    int res;
    uint32_t max8 = 255;

    rgba8 = (uint8_t*)malloc(width * height * (uint32_t)channels);
    if (!rgba8) return 0;

    if (strcmp(type_str, "gradient") == 0) {
        for (y = 0; y < height; y++) {
            for (x = 0; x < width; x++) {
                uint8_t *px = rgba8 + (y * width + x) * (uint32_t)channels;
                double fx = (width  > 1) ? (double)x / (double)(width  - 1) : 0.0;
                double fy = (height > 1) ? (double)y / (double)(height - 1) : 0.0;
                switch (color_type) {
                case CT_GRAY:
                    px[0] = (uint8_t)(fx * 255.0 + 0.5);
                    break;
                case CT_GRAY_ALPHA:
                    px[0] = (uint8_t)(fx * 255.0 + 0.5);
                    px[1] = (uint8_t)(fy * 255.0 + 0.5);
                    break;
                case CT_RGB:
                    px[0] = (uint8_t)(fx * 255.0 + 0.5);
                    px[1] = (uint8_t)(fy * 255.0 + 0.5);
                    px[2] = (uint8_t)((1.0 - fx) * 255.0 + 0.5);
                    break;
                case CT_RGBA:
                    px[0] = (uint8_t)(fx * 255.0 + 0.5);
                    px[1] = (uint8_t)(fy * 255.0 + 0.5);
                    px[2] = (uint8_t)((1.0 - fx) * 255.0 + 0.5);
                    px[3] = (uint8_t)((0.5 + 0.5 * (fx + fy) / 2.0) * 255.0 + 0.5);
                    break;
                default:
                    px[0] = (uint8_t)(fx * 255.0 + 0.5);
                    break;
                }
            }
        }
    } else {
        for (y = 0; y < height; y++) {
            for (x = 0; x < width; x++) {
                uint8_t *px = rgba8 + (y * width + x) * (uint32_t)channels;
                switch (color_type) {
                case CT_GRAY:        px[0] = r8;                           break;
                case CT_GRAY_ALPHA:  px[0] = r8; px[1] = max8;            break;
                case CT_RGB:         px[0] = r8; px[1] = g8; px[2] = b8;  break;
                case CT_RGBA:        px[0] = r8; px[1] = g8; px[2] = b8; px[3] = max8; break;
                default:             px[0] = r8;                           break;
                }
            }
        }
    }

    cfde_pix = rgba8_to_cfde_pixels(rgba8, width, height, channels, bpp, &plen);
    free(rgba8);
    if (!cfde_pix) return 0;

    img.width       = width;
    img.height      = height;
    img.color_type  = (uint8_t)color_type;
    img.bit_depth   = (uint8_t)bpp;
    img.compression = (uint8_t)compression;
    img.filter      = 0;
    img.interlace   = (uint8_t)interlace;
    img.block_size  = 65536;
    img.pixels      = cfde_pix;
    img.pixels_len  = plen;

    printf("> Creating: %s (%lux%lu, %s, ct=%d, %dbpp, comp=%d, interlace=%d)\n",
           outpath, (unsigned long)width, (unsigned long)height,
           type_str, color_type, bpp, compression, interlace);
    res = cfde_write(outpath, &img);
    free(cfde_pix);
    return res;
}

static int rgb_to_ansi2_bg(uint8_t r, uint8_t g, uint8_t b) {
    int gray = (int)((double)r*0.299 + (double)g*0.587 + (double)b*0.114 + 0.5);
    return (gray > 127) ? 47 : 40;
}

static int rgb_to_ansi8_bg(uint8_t r, uint8_t g, uint8_t b) {
    static const uint8_t pal[8][3] = {
        {0,0,0}, {170,0,0}, {0,170,0}, {170,170,0},
        {0,0,170}, {170,0,170}, {0,170,170}, {170,170,170}
    };
    int i, best = 0;
    uint32_t best_dist = 0xFFFFFFFFUL;
    uint32_t dist;
    int dr, dg, db;
    for (i = 0; i < 8; i++) {
        dr = r - pal[i][0]; dg = g - pal[i][1]; db = b - pal[i][2];
        dist = (uint32_t)(dr*dr + dg*dg + db*db);
        if (dist < best_dist) { best_dist = dist; best = i; }
    }
    return 40 + best;
}

static int rgb_to_ansi16_bg(uint8_t r, uint8_t g, uint8_t b) {
    static const uint8_t pal[16][3] = {
        {0,0,0}, {128,0,0}, {0,128,0}, {128,128,0},
        {0,0,128}, {128,0,128}, {0,128,128}, {192,192,192},
        {128,128,128}, {255,0,0}, {0,255,0}, {255,255,0},
        {0,0,255}, {255,0,255}, {0,255,255}, {255,255,255}
    };
    int best = 0;
    uint32_t best_dist = (uint32_t)-1;
    int i;
    for (i = 0; i < 16; i++) {
        int dr = r - pal[i][0];
        int dg = g - pal[i][1];
        int db = b - pal[i][2];
        uint32_t dist = (uint32_t)(dr*dr + dg*dg + db*db);
        if (dist < best_dist) { best_dist = dist; best = i; }
    }
    return (best < 8) ? (40 + best) : (100 + (best - 8));
}

static const uint8_t ANSI256_PAL[256][3] = {
    {0,0,0},{128,0,0},{0,128,0},{128,128,0},{0,0,128},{128,0,128},{0,128,128},{192,192,192},
    {128,128,128},{255,0,0},{0,255,0},{255,255,0},{0,0,255},{255,0,255},{0,255,255},{255,255,255},
    {0,0,0},{0,0,95},{0,0,135},{0,0,175},{0,0,215},{0,0,255},{0,95,0},{0,95,95},{0,95,135},
    {0,95,175},{0,95,215},{0,95,255},{0,135,0},{0,135,95},{0,135,135},{0,135,175},{0,135,215},
    {0,135,255},{0,175,0},{0,175,95},{0,175,135},{0,175,175},{0,175,215},{0,175,255},{0,215,0},
    {0,215,95},{0,215,135},{0,215,175},{0,215,215},{0,215,255},{0,255,0},{0,255,95},{0,255,135},
    {0,255,175},{0,255,215},{0,255,255},{95,0,0},{95,0,95},{95,0,135},{95,0,175},{95,0,215},
    {95,0,255},{95,95,0},{95,95,95},{95,95,135},{95,95,175},{95,95,215},{95,95,255},{95,135,0},
    {95,135,95},{95,135,135},{95,135,175},{95,135,215},{95,135,255},{95,175,0},{95,175,95},
    {95,175,135},{95,175,175},{95,175,215},{95,175,255},{95,215,0},{95,215,95},{95,215,135},
    {95,215,175},{95,215,215},{95,215,255},{95,255,0},{95,255,95},{95,255,135},{95,255,175},
    {95,255,215},{95,255,255},{135,0,0},{135,0,95},{135,0,135},{135,0,175},{135,0,215},{135,0,255},
    {135,95,0},{135,95,95},{135,95,135},{135,95,175},{135,95,215},{135,95,255},{135,135,0},
    {135,135,95},{135,135,135},{135,135,175},{135,135,215},{135,135,255},{135,175,0},{135,175,95},
    {135,175,135},{135,175,175},{135,175,215},{135,175,255},{135,215,0},{135,215,95},{135,215,135},
    {135,215,175},{135,215,215},{135,215,255},{135,255,0},{135,255,95},{135,255,135},{135,255,175},
    {135,255,215},{135,255,255},{175,0,0},{175,0,95},{175,0,135},{175,0,175},{175,0,215},{175,0,255},
    {175,95,0},{175,95,95},{175,95,135},{175,95,175},{175,95,215},{175,95,255},{175,135,0},
    {175,135,95},{175,135,135},{175,135,175},{175,135,215},{175,135,255},{175,175,0},{175,175,95},
    {175,175,135},{175,175,175},{175,175,215},{175,175,255},{175,215,0},{175,215,95},{175,215,135},
    {175,215,175},{175,215,215},{175,215,255},{175,255,0},{175,255,95},{175,255,135},{175,255,175},
    {175,255,215},{175,255,255},{215,0,0},{215,0,95},{215,0,135},{215,0,175},{215,0,215},{215,0,255},
    {215,95,0},{215,95,95},{215,95,135},{215,95,175},{215,95,215},{215,95,255},{215,135,0},
    {215,135,95},{215,135,135},{215,135,175},{215,135,215},{215,135,255},{215,175,0},{215,175,95},
    {215,175,135},{215,175,175},{215,175,215},{215,175,255},{215,215,0},{215,215,95},{215,215,135},
    {215,215,175},{215,215,215},{215,215,255},{215,255,0},{215,255,95},{215,255,135},{215,255,175},
    {215,255,215},{215,255,255},{255,0,0},{255,0,95},{255,0,135},{255,0,175},{255,0,215},{255,0,255},
    {255,95,0},{255,95,95},{255,95,135},{255,95,175},{255,95,215},{255,95,255},{255,135,0},
    {255,135,95},{255,135,135},{255,135,175},{255,135,215},{255,135,255},{255,175,0},{255,175,95},
    {255,175,135},{255,175,175},{255,175,215},{255,175,255},{255,215,0},{255,215,95},{255,215,135},
    {255,215,175},{255,215,215},{255,215,255},{255,255,0},{255,255,95},{255,255,135},{255,255,175},
    {255,255,215},{255,255,255},{8,8,8},{18,18,18},{28,28,28},{38,38,38},{48,48,48},{58,58,58},
    {68,68,68},{78,78,78},{88,88,88},{98,98,98},{108,108,108},{118,118,118},{128,128,128},{138,138,138},
    {148,148,148},{158,158,158},{168,168,168},{178,178,178},{188,188,188},{198,198,198},{208,208,208},
    {218,218,218},{228,228,228},{238,238,238}
};

static int rgb_to_ansi256_bg(uint8_t r, uint8_t g, uint8_t b) {
    int i, best = 0;
    uint32_t best_dist = 0xFFFFFFFFUL;
    uint32_t dist;
    int dr, dg, db;
    for (i = 0; i < 256; i++) {
        dr = r - ANSI256_PAL[i][0];
        dg = g - ANSI256_PAL[i][1];
        db = b - ANSI256_PAL[i][2];
        dist = (uint32_t)(dr*dr + dg*dg + db*db);
        if (dist < best_dist) { best_dist = dist; best = i; }
        if (dist == 0) return i;
    }
    return best;
}

static char *render_pixel(char *p, int buf_mode,
                           uint8_t r, uint8_t g, uint8_t b,
                           int mode, int double_wide,
                           const char *RAMP, int ramp_len) {
    int gray, ci;
    if (buf_mode) {
        if (mode == VIEW_MODE_FULL) {
            p += sprintf(p, "\033[48;2;%d;%d;%dm%s\033[0m",
                         (int)r, (int)g, (int)b, double_wide ? "  " : " ");
        } else if (mode == VIEW_MODE_ANSI2) {
            p += sprintf(p, "\033[%dm%s\033[0m",
                         rgb_to_ansi2_bg(r, g, b), double_wide ? "  " : " ");
        } else if (mode == VIEW_MODE_ANSI8) {
            p += sprintf(p, "\033[%dm%s\033[0m",
                         rgb_to_ansi8_bg(r, g, b), double_wide ? "  " : " ");
        } else if (mode == VIEW_MODE_ANSI16) {
            p += sprintf(p, "\033[%dm%s\033[0m",
                         rgb_to_ansi16_bg(r, g, b), double_wide ? "  " : " ");
        } else if (mode == VIEW_MODE_ANSI256) {
            p += sprintf(p, "\033[48;5;%dm%s\033[0m",
                         rgb_to_ansi256_bg(r, g, b), double_wide ? "  " : " ");
        } else {
            gray = (int)((double)r*0.299 + (double)g*0.587 + (double)b*0.114 + 0.5);
            ci   = (int)((double)gray / 255.0 * (double)(ramp_len - 1) + 0.5);
            if (ci < 0) ci = 0;
            if (ci >= ramp_len) ci = ramp_len - 1;
            *p++ = RAMP[ci];
            if (double_wide) *p++ = RAMP[ci];
        }
    } else {
        if (mode == VIEW_MODE_FULL) {
            printf("\033[48;2;%d;%d;%dm%s\033[0m",
                   (int)r, (int)g, (int)b, double_wide ? "  " : " ");
        } else if (mode == VIEW_MODE_ANSI2) {
            printf("\033[%dm%s\033[0m",
                   rgb_to_ansi2_bg(r, g, b), double_wide ? "  " : " ");
        } else if (mode == VIEW_MODE_ANSI8) {
            printf("\033[%dm%s\033[0m",
                   rgb_to_ansi8_bg(r, g, b), double_wide ? "  " : " ");
        } else if (mode == VIEW_MODE_ANSI16) {
            printf("\033[%dm%s\033[0m",
                   rgb_to_ansi16_bg(r, g, b), double_wide ? "  " : " ");
        } else if (mode == VIEW_MODE_ANSI256) {
            printf("\033[48;5;%dm%s\033[0m",
                   rgb_to_ansi256_bg(r, g, b), double_wide ? "  " : " ");
        } else {
            gray = (int)((double)r*0.299 + (double)g*0.587 + (double)b*0.114 + 0.5);
            ci   = (int)((double)gray / 255.0 * (double)(ramp_len - 1) + 0.5);
            if (ci < 0) ci = 0;
            if (ci >= ramp_len) ci = ramp_len - 1;
            putchar(RAMP[ci]);
            if (double_wide) putchar(RAMP[ci]);
        }
    }
    return p;
}

static int cmd_view(const char *path, int mode, int layout, int flush_mode,
                    int show_interlaced) {
    CFDEImage img;
    uint8_t *rgba8;
    int channels;
    uint32_t x, y;
    uint32_t disp_w, disp_h;
    uint32_t src_x, src_y;
    const uint8_t *px;
    uint8_t r, g, b;
    int gray, ci;
    static const char *RAMP = " .:-=+*#%@";
    int ramp_len = 10;
    char *buf = NULL;
    char *p;
    size_t buf_size;
    uint32_t cols, rows;

    memset(&img, 0, sizeof(img));
    if (!cfde_read(path, &img)) return 0;

    channels = channels_for_type((int)img.color_type);
    rgba8 = cfde_to_rgba8(&img);
    if (!rgba8) { cfde_image_free(&img); return 0; }

    printf("\033[2J\033[H");
    printf("=== CFDE Viewer ===\n");
    printf("File       : %s\n", path);
    printf("Dimensions : %lux%lu\n", (unsigned long)img.width, (unsigned long)img.height);
    printf("Interlace  : %d\n", (int)img.interlace);
    printf("Render Mode: %s\n",
           mode == VIEW_MODE_FULL    ? "24-bit True Color" :
           mode == VIEW_MODE_ANSI2   ? "2-color ANSI (B/W)" :
           mode == VIEW_MODE_ANSI8   ? "8-color ANSI" :
           mode == VIEW_MODE_ANSI16  ? "16-color ANSI" :
           mode == VIEW_MODE_ANSI256 ? "256-color ANSI" : "Grayscale ASCII Art");
    printf("View Mode  : %s\n", layout == LAYOUT_HALF_VERT      ? "Half-Vert (1:1 pixels, half rows)" :
                                  layout == LAYOUT_FULL_STRETCHED  ? "Full (stretched, all pixels)" :
                                  layout == LAYOUT_ASPECT          ? "Aspect (2:1 width per pixel)" : "Miniature");
    printf("Flush Mode : %s\n\n",
           flush_mode == FLUSH_BUFFER ? "Full Buffer" :
           flush_mode == FLUSH_PIXEL  ? "Pixel-by-Pixel" : "Line-by-Line");

    if (show_interlaced && img.interlace > 1) {
        uint8_t stride = img.interlace;
        uint8_t pass;
        uint32_t ix;
        int double_wide = (layout == LAYOUT_ASPECT);
        int cells_per_pixel = double_wide ? 2 : 1;
        size_t row_buf_bytes = (size_t)img.width * (size_t)cells_per_pixel * 48 + 8;
        char *lbuf = (char *)malloc(row_buf_bytes);
        char *lp;                                    

        if (!lbuf) { free(rgba8); cfde_image_free(&img); return 0; }

        printf("\033[2J");
        fflush(stdout);

        for (pass = 0; pass < stride; pass++) {
            uint32_t pass_rows = 0;
            uint32_t tmp_r;
            for (tmp_r = (uint32_t)pass; tmp_r < img.height; tmp_r += (uint32_t)stride)
                pass_rows++;

            printf("\033[H");
            printf("--- Interlace pass %d/%d  (%lu row(s) this pass) ---\n",
                   (int)pass + 1, (int)stride, (unsigned long)pass_rows);
            fflush(stdout);

            for (y = pass; y < img.height; y += stride) {
                int skip_half = (layout == LAYOUT_HALF_VERT && (y % 2) != 0);
                if (skip_half) continue;

                lp = lbuf;                      
                for (ix = 0; ix < img.width; ix++) {
                    px = rgba8 + (y * img.width + ix) * (uint32_t)channels;
                    switch (channels) {
                    case 1: r = g = b = px[0]; break;
                    case 2: r = g = b = px[0]; break;
                    case 3: r = px[0]; g = px[1]; b = px[2]; break;
                    case 4: r = px[0]; g = px[1]; b = px[2]; break;
                    default: r = g = b = px[0]; break;
                    }
                    lp = render_pixel(lp, 1, r, g, b, mode, double_wide, RAMP, ramp_len);
                }
                lp += sprintf(lp, "\033[0m\n");

                printf("\033[%d;1H", 2 + (int)y);
                fwrite(lbuf, 1, (size_t)(lp - lbuf), stdout);
            }
            fflush(stdout);
        }

        printf("\033[0m\n\033[999;1H");
        fflush(stdout);
        free(lbuf);
        free(rgba8);
        cfde_image_free(&img);
        return 1;
    }

    if (layout == LAYOUT_HALF_VERT) {
        cols = img.width;
        rows = (img.height + 1) / 2;
        buf_size = (size_t)rows * ((size_t)cols * 30 + 8) + 1024;

        if (flush_mode == FLUSH_BUFFER) {
            buf = (char *)malloc(buf_size);
            if (!buf) { free(rgba8); cfde_image_free(&img); return 0; }
            p = buf;
            for (y = 0; y < img.height; y += 2) {
                src_y = y;
                for (x = 0; x < cols; x++) {
                    px = rgba8 + (src_y * img.width + x) * (uint32_t)channels;
                    switch (channels) {
                    case 1: r = g = b = px[0]; break;
                    case 2: r = g = b = px[0]; break;
                    case 3: r = px[0]; g = px[1]; b = px[2]; break;
                    case 4: r = px[0]; g = px[1]; b = px[2]; break;
                    default: r = g = b = px[0]; break;
                    }
                    if (mode == VIEW_MODE_FULL) p += sprintf(p, "\033[48;2;%d;%d;%dm \033[0m", (int)r, (int)g, (int)b);
                    else if (mode == VIEW_MODE_ANSI2) p += sprintf(p, "\033[%dm  \033[0m", rgb_to_ansi2_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI8) p += sprintf(p, "\033[%dm  \033[0m", rgb_to_ansi8_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI16) p += sprintf(p, "\033[%dm  \033[0m", rgb_to_ansi16_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI256) p += sprintf(p, "\033[48;5;%dm \033[0m", rgb_to_ansi256_bg(r, g, b));
                    else {
                        gray = (int)((double)r*0.299 + (double)g*0.587 + (double)b*0.114 + 0.5);
                        ci = (int)((double)gray / 255.0 * (double)(ramp_len - 1) + 0.5);
                        if (ci < 0) ci = 0;
                        if (ci >= ramp_len) ci = ramp_len - 1;
                        *p++ = RAMP[ci];
                    }
                }
                *p++ = '\n';
            }
            fwrite(buf, 1, (size_t)(p - buf), stdout);
            free(buf);
        } else if (flush_mode == FLUSH_LINE) {
            buf_size = (size_t)cols * 30 + 32;
            buf = (char *)malloc(buf_size);
            if (!buf) { free(rgba8); cfde_image_free(&img); return 0; }
            for (y = 0; y < img.height; y += 2) {
                src_y = y; p = buf;
                for (x = 0; x < cols; x++) {
                    px = rgba8 + (src_y * img.width + x) * (uint32_t)channels;
                    switch (channels) {
                    case 1: r = g = b = px[0]; break;
                    case 2: r = g = b = px[0]; break;
                    case 3: r = px[0]; g = px[1]; b = px[2]; break;
                    case 4: r = px[0]; g = px[1]; b = px[2]; break;
                    default: r = g = b = px[0]; break;
                    }
                    if (mode == VIEW_MODE_FULL) p += sprintf(p, "\033[48;2;%d;%d;%dm \033[0m", (int)r, (int)g, (int)b);
                    else if (mode == VIEW_MODE_ANSI2) p += sprintf(p, "\033[%dm  \033[0m", rgb_to_ansi2_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI8) p += sprintf(p, "\033[%dm  \033[0m", rgb_to_ansi8_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI16) p += sprintf(p, "\033[%dm  \033[0m", rgb_to_ansi16_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI256) p += sprintf(p, "\033[48;5;%dm \033[0m", rgb_to_ansi256_bg(r, g, b));
                    else {
                        gray = (int)((double)r*0.299 + (double)g*0.587 + (double)b*0.114 + 0.5);
                        ci = (int)((double)gray / 255.0 * (double)(ramp_len - 1) + 0.5);
                        if (ci < 0) ci = 0;
                        if (ci >= ramp_len) ci = ramp_len - 1;
                        *p++ = RAMP[ci];
                    }
                }
                *p++ = '\n';
                fwrite(buf, 1, (size_t)(p - buf), stdout);
            }
            free(buf);
        } else {
            for (y = 0; y < img.height; y += 2) {
                src_y = y;
                for (x = 0; x < cols; x++) {
                    px = rgba8 + (src_y * img.width + x) * (uint32_t)channels;
                    switch (channels) {
                    case 1: r = g = b = px[0]; break;
                    case 2: r = g = b = px[0]; break;
                    case 3: r = px[0]; g = px[1]; b = px[2]; break;
                    case 4: r = px[0]; g = px[1]; b = px[2]; break;
                    default: r = g = b = px[0]; break;
                    }
                    if (mode == VIEW_MODE_FULL) printf("\033[48;2;%d;%d;%dm \033[0m", (int)r, (int)g, (int)b);
                    else if (mode == VIEW_MODE_ANSI2) printf("\033[%dm  \033[0m", rgb_to_ansi2_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI8) printf("\033[%dm  \033[0m", rgb_to_ansi8_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI16) printf("\033[%dm  \033[0m", rgb_to_ansi16_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI256) printf("\033[48;5;%dm \033[0m", rgb_to_ansi256_bg(r, g, b));
                    else {
                        gray = (int)((double)r*0.299 + (double)g*0.587 + (double)b*0.114 + 0.5);
                        ci = (int)((double)gray / 255.0 * (double)(ramp_len - 1) + 0.5);
                        if (ci < 0) ci = 0;
                        if (ci >= ramp_len) ci = ramp_len - 1;
                        putchar(RAMP[ci]);
                    }
                }
                putchar('\n');
            }
        }
        printf("\033[0m\n\033[999;1H");
        fflush(stdout);
        free(rgba8);
        cfde_image_free(&img);
        return 1;
    }

    if (layout == LAYOUT_FULL_STRETCHED) {
        cols = img.width;
        buf_size = (size_t)img.height * ((size_t)cols * 30 + 8) + 1024;

        if (flush_mode == FLUSH_BUFFER) {
            buf = (char *)malloc(buf_size);
            if (!buf) { free(rgba8); cfde_image_free(&img); return 0; }
            p = buf;
            for (y = 0; y < img.height; y++) {
                for (x = 0; x < cols; x++) {
                    px = rgba8 + (y * img.width + x) * (uint32_t)channels;
                    switch (channels) {
                    case 1: r = g = b = px[0]; break;
                    case 2: r = g = b = px[0]; break;
                    case 3: r = px[0]; g = px[1]; b = px[2]; break;
                    case 4: r = px[0]; g = px[1]; b = px[2]; break;
                    default: r = g = b = px[0]; break;
                    }
                    if (mode == VIEW_MODE_FULL) p += sprintf(p, "\033[48;2;%d;%d;%dm \033[0m", (int)r, (int)g, (int)b);
                    else if (mode == VIEW_MODE_ANSI2) p += sprintf(p, "\033[%dm \033[0m", rgb_to_ansi2_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI8) p += sprintf(p, "\033[%dm \033[0m", rgb_to_ansi8_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI16) p += sprintf(p, "\033[%dm \033[0m", rgb_to_ansi16_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI256) p += sprintf(p, "\033[48;5;%dm \033[0m", rgb_to_ansi256_bg(r, g, b));
                    else {
                        gray = (int)((double)r*0.299 + (double)g*0.587 + (double)b*0.114 + 0.5);
                        ci = (int)((double)gray / 255.0 * (double)(ramp_len - 1) + 0.5);
                        if (ci < 0) ci = 0;
                        if (ci >= ramp_len) ci = ramp_len - 1;
                        *p++ = RAMP[ci];
                    }
                }
                *p++ = '\n';
            }
            fwrite(buf, 1, (size_t)(p - buf), stdout);
            free(buf);
        } else if (flush_mode == FLUSH_LINE) {
            buf_size = (size_t)cols * 30 + 32;
            buf = (char *)malloc(buf_size);
            if (!buf) { free(rgba8); cfde_image_free(&img); return 0; }
            for (y = 0; y < img.height; y++) {
                p = buf;
                for (x = 0; x < cols; x++) {
                    px = rgba8 + (y * img.width + x) * (uint32_t)channels;
                    switch (channels) {
                    case 1: r = g = b = px[0]; break;
                    case 2: r = g = b = px[0]; break;
                    case 3: r = px[0]; g = px[1]; b = px[2]; break;
                    case 4: r = px[0]; g = px[1]; b = px[2]; break;
                    default: r = g = b = px[0]; break;
                    }
                    if (mode == VIEW_MODE_FULL) p += sprintf(p, "\033[48;2;%d;%d;%dm \033[0m", (int)r, (int)g, (int)b);
                    else if (mode == VIEW_MODE_ANSI2) p += sprintf(p, "\033[%dm \033[0m", rgb_to_ansi2_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI8) p += sprintf(p, "\033[%dm \033[0m", rgb_to_ansi8_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI16) p += sprintf(p, "\033[%dm \033[0m", rgb_to_ansi16_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI256) p += sprintf(p, "\033[48;5;%dm \033[0m", rgb_to_ansi256_bg(r, g, b));
                    else {
                        gray = (int)((double)r*0.299 + (double)g*0.587 + (double)b*0.114 + 0.5);
                        ci = (int)((double)gray / 255.0 * (double)(ramp_len - 1) + 0.5);
                        if (ci < 0) ci = 0;
                        if (ci >= ramp_len) ci = ramp_len - 1;
                        *p++ = RAMP[ci];
                    }
                }
                *p++ = '\n';
                fwrite(buf, 1, (size_t)(p - buf), stdout);
            }
            free(buf);
        } else {
            for (y = 0; y < img.height; y++) {
                for (x = 0; x < cols; x++) {
                    px = rgba8 + (y * img.width + x) * (uint32_t)channels;
                    switch (channels) {
                    case 1: r = g = b = px[0]; break;
                    case 2: r = g = b = px[0]; break;
                    case 3: r = px[0]; g = px[1]; b = px[2]; break;
                    case 4: r = px[0]; g = px[1]; b = px[2]; break;
                    default: r = g = b = px[0]; break;
                    }
                    if (mode == VIEW_MODE_FULL) printf("\033[48;2;%d;%d;%dm \033[0m", (int)r, (int)g, (int)b);
                    else if (mode == VIEW_MODE_ANSI2) printf("\033[%dm \033[0m", rgb_to_ansi2_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI8) printf("\033[%dm \033[0m", rgb_to_ansi8_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI16) printf("\033[%dm \033[0m", rgb_to_ansi16_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI256) printf("\033[48;5;%dm \033[0m", rgb_to_ansi256_bg(r, g, b));
                    else {
                        gray = (int)((double)r*0.299 + (double)g*0.587 + (double)b*0.114 + 0.5);
                        ci = (int)((double)gray / 255.0 * (double)(ramp_len - 1) + 0.5);
                        if (ci < 0) ci = 0;
                        if (ci >= ramp_len) ci = ramp_len - 1;
                        putchar(RAMP[ci]);
                    }
                }
                putchar('\n');
            }
        }
        printf("\033[0m\n\033[999;1H");
        fflush(stdout);
        free(rgba8);
        cfde_image_free(&img);
        return 1;
    }

    if (layout == LAYOUT_ASPECT) {
        cols = img.width;
        buf_size = (size_t)img.height * ((size_t)cols * 60 + 8) + 1024;

        if (flush_mode == FLUSH_BUFFER) {
            buf = (char *)malloc(buf_size);
            if (!buf) { free(rgba8); cfde_image_free(&img); return 0; }
            p = buf;
            for (y = 0; y < img.height; y++) {
                for (x = 0; x < cols; x++) {
                    px = rgba8 + (y * img.width + x) * (uint32_t)channels;
                    switch (channels) {
                    case 1: r = g = b = px[0]; break;
                    case 2: r = g = b = px[0]; break;
                    case 3: r = px[0]; g = px[1]; b = px[2]; break;
                    case 4: r = px[0]; g = px[1]; b = px[2]; break;
                    default: r = g = b = px[0]; break;
                    }
                    if (mode == VIEW_MODE_FULL) p += sprintf(p, "\033[48;2;%d;%d;%dm  \033[0m", (int)r, (int)g, (int)b);
                    else if (mode == VIEW_MODE_ANSI2) p += sprintf(p, "\033[%dm  \033[0m", rgb_to_ansi2_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI8) p += sprintf(p, "\033[%dm  \033[0m", rgb_to_ansi8_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI16) p += sprintf(p, "\033[%dm  \033[0m", rgb_to_ansi16_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI256) p += sprintf(p, "\033[48;5;%dm  \033[0m", rgb_to_ansi256_bg(r, g, b));
                    else {
                        gray = (int)((double)r*0.299 + (double)g*0.587 + (double)b*0.114 + 0.5);
                        ci = (int)((double)gray / 255.0 * (double)(ramp_len - 1) + 0.5);
                        if (ci < 0) ci = 0;
                        if (ci >= ramp_len) ci = ramp_len - 1;
                        *p++ = RAMP[ci]; *p++ = RAMP[ci];
                    }
                }
                *p++ = '\n';
            }
            fwrite(buf, 1, (size_t)(p - buf), stdout);
            free(buf);
        } else if (flush_mode == FLUSH_LINE) {
            buf_size = (size_t)cols * 60 + 32;
            buf = (char *)malloc(buf_size);
            if (!buf) { free(rgba8); cfde_image_free(&img); return 0; }
            for (y = 0; y < img.height; y++) {
                p = buf;
                for (x = 0; x < cols; x++) {
                    px = rgba8 + (y * img.width + x) * (uint32_t)channels;
                    switch (channels) {
                    case 1: r = g = b = px[0]; break;
                    case 2: r = g = b = px[0]; break;
                    case 3: r = px[0]; g = px[1]; b = px[2]; break;
                    case 4: r = px[0]; g = px[1]; b = px[2]; break;
                    default: r = g = b = px[0]; break;
                    }
                    if (mode == VIEW_MODE_FULL) p += sprintf(p, "\033[48;2;%d;%d;%dm  \033[0m", (int)r, (int)g, (int)b);
                    else if (mode == VIEW_MODE_ANSI2) p += sprintf(p, "\033[%dm  \033[0m", rgb_to_ansi2_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI8) p += sprintf(p, "\033[%dm  \033[0m", rgb_to_ansi8_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI16) p += sprintf(p, "\033[%dm  \033[0m", rgb_to_ansi16_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI256) p += sprintf(p, "\033[48;5;%dm  \033[0m", rgb_to_ansi256_bg(r, g, b));
                    else {
                        gray = (int)((double)r*0.299 + (double)g*0.587 + (double)b*0.114 + 0.5);
                        ci = (int)((double)gray / 255.0 * (double)(ramp_len - 1) + 0.5);
                        if (ci < 0) ci = 0;
                        if (ci >= ramp_len) ci = ramp_len - 1;
                        *p++ = RAMP[ci]; *p++ = RAMP[ci];
                    }
                }
                *p++ = '\n';
                fwrite(buf, 1, (size_t)(p - buf), stdout);
            }
            free(buf);
        } else {
            for (y = 0; y < img.height; y++) {
                for (x = 0; x < cols; x++) {
                    px = rgba8 + (y * img.width + x) * (uint32_t)channels;
                    switch (channels) {
                    case 1: r = g = b = px[0]; break;
                    case 2: r = g = b = px[0]; break;
                    case 3: r = px[0]; g = px[1]; b = px[2]; break;
                    case 4: r = px[0]; g = px[1]; b = px[2]; break;
                    default: r = g = b = px[0]; break;
                    }
                    if (mode == VIEW_MODE_FULL) printf("\033[48;2;%d;%d;%dm  \033[0m", (int)r, (int)g, (int)b);
                    else if (mode == VIEW_MODE_ANSI2) printf("\033[%dm  \033[0m", rgb_to_ansi2_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI8) printf("\033[%dm  \033[0m", rgb_to_ansi8_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI16) printf("\033[%dm  \033[0m", rgb_to_ansi16_bg(r, g, b));
                    else if (mode == VIEW_MODE_ANSI256) printf("\033[48;5;%dm  \033[0m", rgb_to_ansi256_bg(r, g, b));
                    else {
                        gray = (int)((double)r*0.299 + (double)g*0.587 + (double)b*0.114 + 0.5);
                        ci = (int)((double)gray / 255.0 * (double)(ramp_len - 1) + 0.5);
                        if (ci < 0) ci = 0;
                        if (ci >= ramp_len) ci = ramp_len - 1;
                        putchar(RAMP[ci]); putchar(RAMP[ci]);
                    }
                }
                putchar('\n');
            }
        }
        printf("\033[0m\n\033[999;1H");
        fflush(stdout);
        free(rgba8);
        cfde_image_free(&img);
        return 1;
    }

    disp_w = img.width  > 80 ? 80 : img.width;
    disp_h = (uint32_t)((double)disp_w * (double)img.height / (double)img.width * 0.5 + 0.5);
    if (disp_h < 1) disp_h = 1;
    if (disp_h > 40) disp_h = 40;

    for (y = 0; y < disp_h; y++) {
        src_y = (uint32_t)((double)y / (double)disp_h * (double)img.height);
        if (src_y >= img.height) src_y = img.height - 1;

        for (x = 0; x < disp_w; x++) {
            src_x = (uint32_t)((double)x / (double)disp_w * (double)img.width);
            if (src_x >= img.width) src_x = img.width - 1;
            px = rgba8 + (src_y * img.width + src_x) * (uint32_t)channels;

            switch (channels) {
            case 1: r = g = b = px[0]; break;
            case 2: r = g = b = px[0]; break;
            case 3: r = px[0]; g = px[1]; b = px[2]; break;
            case 4: r = px[0]; g = px[1]; b = px[2]; break;
            default: r = g = b = px[0]; break;
            }

            if (mode == VIEW_MODE_FULL) printf("\033[48;2;%d;%d;%dm ", (int)r, (int)g, (int)b);
            else if (mode == VIEW_MODE_ANSI2) printf("\033[%dm  ", rgb_to_ansi2_bg(r, g, b));
            else if (mode == VIEW_MODE_ANSI8) printf("\033[%dm  ", rgb_to_ansi8_bg(r, g, b));
            else if (mode == VIEW_MODE_ANSI16) printf("\033[%dm  ", rgb_to_ansi16_bg(r, g, b));
            else if (mode == VIEW_MODE_ANSI256) printf("\033[48;5;%dm ", rgb_to_ansi256_bg(r, g, b));
            else {
                gray = (int)((double)r*0.299 + (double)g*0.587 + (double)b*0.114 + 0.5);
                ci = (int)((double)gray / 255.0 * (double)(ramp_len - 1) + 0.5);
                if (ci < 0) ci = 0;
                if (ci >= ramp_len) ci = ramp_len - 1;
                putchar(RAMP[ci]);
            }
        }
        printf("\033[0m\n");
    }

    printf("\033[0m\n\033[999;1H");
    fflush(stdout);
    free(rgba8);
    cfde_image_free(&img);
    return 1;
}

static int cmd_info(const char *path) {
    CFDEImage img;
    memset(&img, 0, sizeof(img));
    if (!cfde_read(path, &img)) return 0;
    printf("=== CFDE Info ===\n");
    printf("File       : %s\n", path);
    printf("Width      : %lu\n", (unsigned long)img.width);
    printf("Height     : %lu\n", (unsigned long)img.height);
    printf("Color Type : %d\n", (int)img.color_type);
    printf("Bit Depth  : %d\n", (int)img.bit_depth);
    printf("Compression: %d\n", (int)img.compression);
    printf("Filter     : %d\n", (int)img.filter);
    printf("Interlace  : %d\n", (int)img.interlace);
    printf("Block Size : %lu\n", (unsigned long)img.block_size);
    printf("Pixel Bytes: %lu\n", (unsigned long)img.pixels_len);
    cfde_image_free(&img);
    return 1;
}

static void print_usage(const char *prog) {
    fprintf(stderr, "CFDE Image Format Tool\n\n");
    fprintf(stderr, "Usage:\n");
    fprintf(stderr, "  %s convert <input.pnm> <output.cfde> [--bpp=N] [--comp=N] [--interlace=N]\n", prog);
    fprintf(stderr, "      Convert PPM/PGM to CFDE\n\n");
    fprintf(stderr, "  %s decode <input.cfde> <output.pnm>\n", prog);
    fprintf(stderr, "      Decode CFDE to PPM/PGM\n\n");
    fprintf(stderr, "  %s raw <input.bin> <output.cfde> --width=N --height=N\n", prog);
    fprintf(stderr, "          [--color-type=N] [--bpp=N] [--comp=N] [--interlace=N]\n");
    fprintf(stderr, "      Import raw binary as CFDE (--color-type: 0=gray,2=RGB,4=gray+alpha,6=RGBA)\n\n");
    fprintf(stderr, "  %s create <output.cfde> --width=N --height=N\n", prog);
    fprintf(stderr, "          [--type=gradient|single-colour] [--rgb=RRGGBB]\n");
    fprintf(stderr, "          [--color-type=N] [--bpp=N] [--comp=N] [--interlace=N]\n");
    fprintf(stderr, "      Create a synthetic CFDE image\n\n");
    fprintf(stderr, "  %s view <input.cfde>\n", prog);
    fprintf(stderr, "      View CFDE image in terminal (requires one rendering flag)\n");
    fprintf(stderr, "      Render Flags: --full-color  (24-bit ANSI true color)\n");
    fprintf(stderr, "                    --ansi-2      (2-color monochrome B/W)\n");
    fprintf(stderr, "                    --ansi-8      (8-color standard palette)\n");
    fprintf(stderr, "                    --ansi-16     (16-color VGA palette)\n");
    fprintf(stderr, "                    --ansi-256    (256-color xterm palette)\n");
    fprintf(stderr, "                    --grayscale   (Classic ASCII art ramp)\n");
    fprintf(stderr, "      View Flags:   --half-view   (1:1 pixels, skip every other row for terminal aspect, halves the vertical quality)\n");
    fprintf(stderr, "                    --raw-view    (1 char per pixel, all rows, stretched due to terminal limits)\n");
    fprintf(stderr, "                    --full-view   (2 chars wide per pixel, all rows, correct aspect)\n");
    fprintf(stderr, "                    --interlaced-view  (show each interlace pass in sequence; requires interlace>1)\n");
    fprintf(stderr, "      Flush Flags:  --buffer      (Full RAM buffer, single fwrite)\n");
    fprintf(stderr, "                    --flush-line  (Line buffer, fwrite per row)\n");
    fprintf(stderr, "                    --flush       (printf per pixel)\n\n");
    fprintf(stderr, "  %s info <input.cfde>\n", prog);
    fprintf(stderr, "      Print CFDE header info\n\n");
    fprintf(stderr, "Options:\n");
    fprintf(stderr, "  --bpp=N         Bits per sample (1,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32)\n");
    fprintf(stderr, "  --comp=N        Compression level (0=none..5=hyper)\n");
    fprintf(stderr, "  --interlace=N   Interlace stride (0=none, 1-255=rearrange rows by stride N)\n");
    fprintf(stderr, "  --color-type=N  0=Gray, 2=RGB, 3=Indexed, 4=Gray+Alpha, 6=RGBA\n");
    fprintf(stderr, "  --width=N       Image width\n");
    fprintf(stderr, "  --height=N      Image height\n");
    fprintf(stderr, "  --rgb=RRGGBB    Hex color for single-colour create\n");
    fprintf(stderr, "  --type=T        gradient or single-colour\n");
}

static int find_arg_val(int argc, char **argv, const char *prefix, const char **val) {
    size_t plen = strlen(prefix);
    int i;
    for (i = 0; i < argc; i++) {
        if (strncmp(argv[i], prefix, plen) == 0) {
            if (val) *val = argv[i] + plen;
            return i;
        }
    }
    return -1;
}

static int valid_bpp(int bpp) {
    int valid[] = {1,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32};
    int i;
    for (i = 0; i < 17; i++) if (valid[i] == bpp) return 1;
    return 0;
}

int main(int argc, char **argv) {
    const char *cmd, *v;
    int bpp = 8, comp = 0, color_type = CT_RGB, interlace = 0;
    uint32_t width = 0, height = 0;

    if (argc < 2) { print_usage(argv[0]); return 1; }
    cmd = argv[1];

    if (find_arg_val(argc - 2, argv + 2, "--bpp=",        &v) >= 0) bpp        = atoi(v);
    if (find_arg_val(argc - 2, argv + 2, "--comp=",       &v) >= 0) comp       = atoi(v);
    if (find_arg_val(argc - 2, argv + 2, "--color-type=", &v) >= 0) color_type = atoi(v);
    if (find_arg_val(argc - 2, argv + 2, "--width=",      &v) >= 0) width      = (uint32_t)atol(v);
    if (find_arg_val(argc - 2, argv + 2, "--height=",     &v) >= 0) height     = (uint32_t)atol(v);
    if (find_arg_val(argc - 2, argv + 2, "--interlace=",  &v) >= 0) interlace  = atoi(v);

    if (!valid_bpp(bpp)) { fprintf(stderr, "X Invalid --bpp=%d\n", bpp); return 1; }
    if (comp < 0) comp = 0;
    if (comp > 5) comp = 5;
    if (interlace < 0) interlace = 0;
    if (interlace > 255) interlace = 255;

    if (strcmp(cmd, "convert") == 0) {
        if (argc < 4) { print_usage(argv[0]); return 1; }
        return cmd_pnm_to_cfde(argv[2], argv[3], bpp, comp, interlace) ? 0 : 1;
    }

    if (strcmp(cmd, "decode") == 0) {
        if (argc < 4) { print_usage(argv[0]); return 1; }
        return cmd_cfde_to_pnm(argv[2], argv[3]) ? 0 : 1;
    }

    if (strcmp(cmd, "raw") == 0) {
        if (argc < 4) { print_usage(argv[0]); return 1; }
        if (width == 0 || height == 0) {
            fprintf(stderr, "X --width and --height required for raw import\n");
            return 1;
        }
        return cmd_from_raw(argv[2], argv[3], width, height,
                            color_type, bpp, comp, interlace) ? 0 : 1;
    }

    if (strcmp(cmd, "create") == 0) {
        const char *type_str = "gradient";
        uint8_t r8 = 128, g8 = 128, b8 = 128;

        if (argc < 3) { print_usage(argv[0]); return 1; }
        if (width == 0)  width  = 256;
        if (height == 0) height = 256;
        if (find_arg_val(argc - 2, argv + 2, "--type=", &v) >= 0) type_str = v;
        if (find_arg_val(argc - 2, argv + 2, "--rgb=",  &v) >= 0) {
            unsigned long rgb = strtoul(v, NULL, 16);
            r8 = (uint8_t)((rgb >> 16) & 0xFF);
            g8 = (uint8_t)((rgb >>  8) & 0xFF);
            b8 = (uint8_t)( rgb        & 0xFF);
        }
        if (color_type == CT_GRAY || color_type == CT_GRAY_ALPHA) {
            r8 = (uint8_t)((double)r8*0.299 + (double)g8*0.587 + (double)b8*0.114 + 0.5);
        }

        return cmd_create(argv[2], width, height, color_type, bpp, comp, interlace,
                          type_str, r8, g8, b8) ? 0 : 1;
    }

    if (strcmp(cmd, "view") == 0) {
        int view_mode = VIEW_MODE_NONE;
        int flush_mode = FLUSH_NONE;
        int layout = LAYOUT_NONE;
        int show_interlaced = 0;

        if (argc < 3) { print_usage(argv[0]); return 1; }

        if (find_arg_val(argc - 2, argv + 2, "--full-color",      NULL) >= 0) view_mode = VIEW_MODE_FULL;
        if (find_arg_val(argc - 2, argv + 2, "--ansi-2",          NULL) >= 0) view_mode = VIEW_MODE_ANSI2;
        if (find_arg_val(argc - 2, argv + 2, "--ansi-8",          NULL) >= 0) view_mode = VIEW_MODE_ANSI8;
        if (find_arg_val(argc - 2, argv + 2, "--ansi-16",         NULL) >= 0) view_mode = VIEW_MODE_ANSI16;
        if (find_arg_val(argc - 2, argv + 2, "--ansi-256",        NULL) >= 0) view_mode = VIEW_MODE_ANSI256;
        if (find_arg_val(argc - 2, argv + 2, "--grayscale",       NULL) >= 0) view_mode = VIEW_MODE_GRAY;
        if (find_arg_val(argc - 2, argv + 2, "--half-view",       NULL) >= 0) layout = LAYOUT_HALF_VERT;
        if (find_arg_val(argc - 2, argv + 2, "--raw-view",        NULL) >= 0) layout = LAYOUT_FULL_STRETCHED;
        if (find_arg_val(argc - 2, argv + 2, "--full-view",       NULL) >= 0) layout = LAYOUT_ASPECT;
        if (find_arg_val(argc - 2, argv + 2, "--interlaced-view", NULL) >= 0) show_interlaced = 1;

        if (find_arg_val(argc - 2, argv + 2, "--buffer",          NULL) >= 0) flush_mode = FLUSH_BUFFER;
        if (find_arg_val(argc - 2, argv + 2, "--flush",           NULL) >= 0) flush_mode = FLUSH_PIXEL;
        if (find_arg_val(argc - 2, argv + 2, "--flush-line",      NULL) >= 0) flush_mode = FLUSH_LINE;

        if (flush_mode == FLUSH_NONE) flush_mode = FLUSH_LINE;

        if (view_mode == VIEW_MODE_NONE) {
            fprintf(stderr, "X Error: view command requires at least one rendering flag:\n");
            fprintf(stderr, "  --full-color, --ansi-2, --ansi-8, --ansi-16, --ansi-256, or --grayscale\n");
            return 1;
        }

        return cmd_view(argv[2], view_mode, layout, flush_mode, show_interlaced) ? 0 : 1;
    }

    if (strcmp(cmd, "info") == 0) {
        if (argc < 3) { print_usage(argv[0]); return 1; }
        return cmd_info(argv[2]) ? 0 : 1;
    }

    fprintf(stderr, "X Unknown command: %s\n", cmd);
    print_usage(argv[0]);
    return 1;
}
