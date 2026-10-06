/*
 * Spacebox remote GPU
 *
 * Splits the virgl virtio-gpu device between two QEMU processes:
 *
 *   source  runs the guest. It renders nothing. Every control-queue command is
 *           forwarded over a stream socket, together with the guest memory a
 *           transfer command refers to. Responses come back asynchronously.
 *   sink    has no guest. It feeds the forwarded commands to the ordinary virgl
 *           code, so they are rendered by the local GPU and shown by the local
 *           display backend. Input events of the local window go back to the
 *           source.
 *
 * SPACEBOX_GPU_REMOTE_SOURCE=<unix socket>  connect to a sink (retries)
 * SPACEBOX_GPU_REMOTE_SINK=<unix socket>    listen for one source
 * SPACEBOX_GPU_REMOTE_STATS=1               print traffic counters every 5 s
 * SPACEBOX_GPU_REMOTE_FENCE_WINDOW=<n>      source: answer up to n fenced commands
 *                                           before the sink has finished them
 *                                           (default 8, 0 = wait for the sink).
 *                                           Without it every frame of the guest
 *                                           waits one network round trip.
 * SPACEBOX_GPU_REMOTE_KEEP_TIMER_QUERY=1    sink: leave GPU timer queries advertised.
 *                                           By default they are hidden from the
 *                                           guest: a compositor that times every
 *                                           frame with one (Mutter does) would
 *                                           block for a network round trip per
 *                                           frame.
 * SPACEBOX_GPU_REMOTE_TX_LIMIT_MB=<n>       source: stop taking guest commands while
 *                                           more than n MiB wait for the network
 *                                           (default 8). Bounds the display lag
 *                                           that queued uploads would add.
 * SPACEBOX_GPU_REMOTE_ZSTD=<level>          compress the connection (default 1,
 *                                           0 = off; used when both ends offer
 *                                           it). See sp_z_filter().
 *
 * Both ends are assumed little-endian. One source per sink; a lost connection
 * resets the sink and is not recovered on the source (prototype).
 */

#include "qemu/osdep.h"
#include "qemu/iov.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "qemu/thread.h"
#include "qemu/timer.h"
#include "qemu/sockets.h"
#include "hw/virtio/virtio.h"
#include "hw/virtio/virtio-gpu.h"
#include "ui/console.h"
#include "ui/input.h"
#include "audio/audio.h"
#include "virtio-gpu-remote-formats.h"

#include <virglrenderer.h>

#include "hw/display/spacebox-zstd.h"
#ifdef SPACEBOX_ZSTD
#include <zstd.h>
#endif

#include <sys/socket.h>
#include <sys/un.h>

#define SP_MAGIC 0x32475053u /* "SPG2" */
#define SP_RESP_CAP_MAX (4u << 20)

enum {
    SP_CTRL = 1,    /* a: request bytes, b: transfer data. arg0 response capacity,
                       arg1/arg2 byte range of the resource backing */
    SP_CURSOR = 2,  /* a: struct virtio_gpu_update_cursor */
    SP_RESET = 3,
    SP_MEMWRITE = 4, /* either direction. a: bytes for the backing of resource arg0
                        at offset arg1. source->sink arg2 = 1: watch the first 16
                        bytes (query result) and report when the renderer writes them */
    SP_EXPECT = 5,   /* a: SpExpect[], ranges the next SP_CTRL makes the renderer
                        write; the sink returns them as SP_MEMWRITE */
    SP_RESP = 10,   /* a: response bytes, b: read-back data for arg1 offset */
    SP_INPUT = 11,  /* a: SpInput */
    SP_DISPLAY = 12, /* arg0: virtio-gpu event bits */
    SP_HELLO = 20,   /* first message on every connection. arg0 own session,
                        arg1 the peer session it was paired with (0 = none),
                        arg2 highest id received from that peer */
    SP_ACK = 21,     /* nothing but the ack field */
    SP_AUDIO = 14,   /* source -> sink. a: signed 16 bit little endian samples,
                        arg0 rate, arg1 channels */
    SP_INFO = 13     /* sink's answers to the guest's queries, sent ahead so the
                        source can answer them itself. arg0 = 1: capset arg1,
                        arg2 = max version << 32 | size, a = data. arg0 = 2: a =
                        display info response. arg0 = 3: a = EDID response. */
};

typedef struct SpHdr {
    uint32_t magic;
    uint32_t type;
    uint64_t id;    /* per direction, 1.. for reliable messages; 0 = not resent */
    uint64_t ack;   /* highest id received from the peer */
    uint64_t seq;
    uint32_t a_len;
    uint32_t b_len;
    uint64_t arg0;
    uint64_t arg1;
    uint64_t arg2;
    int64_t t_us;   /* sender's clock when the message was queued */
    uint64_t flags; /* SP_F_* */
} QEMU_PACKED SpHdr;

#define SP_F_VIDEO 1    /* source: the guest is decoding video */
#define SP_F_ZSTD 2     /* in SP_HELLO: this end can compress the connection */

/*
 * A compressed connection (both ends set SP_F_ZSTD in their HELLO): after the
 * HELLO every message travels as SpZHdr, SpZFilter if flagged, and clen bytes
 * of zstd data. Each direction is one zstd stream for the life of the
 * connection, flushed after every message.
 */
#define SP_Z_FILTERED 0x80000000u /* in clen: an SpZFilter follows the header */
#define SP_Z_MAX (1u << 30)

typedef struct SpZHdr {
    uint32_t clen;  /* compressed bytes that follow */
    uint32_t rlen;  /* size of the message they expand to */
} QEMU_PACKED SpZHdr;

typedef struct SpZFilter {
    uint32_t off;    /* the message from here to its end went through sp_z_filter() */
    uint32_t bpp;
    uint32_t stride;
} QEMU_PACKED SpZFilter;

typedef struct SpInput {
    uint32_t kind; /* 0 sync, 1 key, 2 button, 3 rel, 4 abs */
    uint32_t a;
    int32_t b;
} QEMU_PACKED SpInput;

typedef struct SpExpect {
    uint32_t res;
    uint32_t pad;
    uint64_t off;
    uint64_t len;
} QEMU_PACKED SpExpect;

typedef struct SpWatch {
    uint8_t last[16];
} SpWatch;

typedef struct SpBuf {
    size_t len;
    uint64_t id;
    int64_t t_us;
    bool writing;
    uint32_t f_off, f_bpp, f_stride; /* picture data in it, see sp_z_filter() */
    uint8_t data[];
} SpBuf;

/* source: layout of the picture data in the message about to be queued */
static struct {
    uint32_t bpp;   /* bytes per pixel, 0 = no picture data */
    uint32_t stride;
    bool in_b;      /* it is the b part of the message (else the a part) */
} sp_tx_hint;

/* statistics: time from queueing a message to having written it to the socket
 * (sp_txd), and how much later than the fastest one a message arrived (sp_rxd) */
static struct SpDelayStat {
    unsigned n, over12, over25, over50;
    int64_t max;
} sp_txd, sp_rxd;
static int64_t sp_rx_min = INT64_MAX, sp_rx_min_prev = INT64_MAX, sp_rx_min_t0;

static void sp_rx_delay(int64_t t_us)
{
    int64_t now = g_get_monotonic_time();
    int64_t d = now - t_us, base;

    if (!t_us) {
        return;
    }
    if (now - sp_rx_min_t0 > 10 * G_USEC_PER_SEC) {
        sp_rx_min_prev = sp_rx_min;
        sp_rx_min = INT64_MAX;
        sp_rx_min_t0 = now;
    }
    sp_rx_min = MIN(sp_rx_min, d);
    base = MIN(sp_rx_min, sp_rx_min_prev);
    d -= base;
    sp_rxd.n++;
    sp_rxd.over12 += d > 12000;
    sp_rxd.over25 += d > 25000;
    sp_rxd.over50 += d > 50000;
    sp_rxd.max = MAX(sp_rxd.max, d);
}

static int64_t sp_video_until;  /* source: video was decoded until about now */
static int64_t sp_aud_delay_us; /* sink: how much later sound is played, 0 = silent */

/*
 * Sink: when to show the frame of the screen update being handled (this
 * machine's clock, 0 = at once). Read by the presenter (ui/spacebox-presenter.h).
 *
 * Messages cross the network with a delay that varies (a lost packet holds up
 * everything behind it), so frames the guest produced evenly arrive in bunches.
 * Each message carries the source's clock. The smallest "arrival - source
 * clock" seen recently is the delay of an undisturbed message; a frame is
 * shown at "source clock + that + allowance", which restores the guest's
 * spacing for every frame that is late by less than the allowance. The
 * allowance follows how late recent screen updates were (95th percentile,
 * capped) and costs that much latency, so by default it is used only while
 * the guest decodes video. SPACEBOX_PRESENT_JITTER=off|video|always,
 * SPACEBOX_PRESENT_JITTER_MAX_MS (default 50).
 *
 * The guest's own spacing is not even either: a 60 Hz video on its 120 Hz
 * desktop comes out as updates 1, 2 or 3 refreshes apart. While pacing, the
 * presenter also does not show a frame sooner after the previous one than the
 * usual spacing of recent updates (their median, in whole refreshes), which
 * turns "1 then 3" into "2 then 2".
 */
int64_t spacebox_frame_due_us;
int64_t spacebox_frame_gap_us;  /* usual spacing of recent updates, 0 = not a steady stream */
/*
 * When the user last pressed a key, clicked or scrolled (this machine's clock;
 * set here and by ui/spacebox-scroll.m). Pacing holds every frame back, the
 * answer to the user's input included. For SPACEBOX_PRESENT_INPUT_MS after
 * such input (default 1500, 0 = never) frames are shown as they come instead:
 * the screen answers at once; a playing video is less even for that time and
 * runs ahead of its sound.
 */
int64_t spacebox_last_input_us;

static struct {
    int mode;           /* 0 off, 1 video, 2 always; -1 not read yet */
    int64_t max_us;
    int64_t late[128];  /* lateness of recent screen updates */
    unsigned n, pos;
    int64_t allowance;
    int64_t input_us;
    uint64_t paced, late_frames, frames, unpaced_for_input;
    int64_t usual_gap;
    int64_t gap[32];    /* source-clock spacing of recent screen updates */
    unsigned gap_n, gap_pos;
    int64_t last_t_us;
} sp_pace = { .mode = -1, .input_us = -1 };

static int sp_cmp_i64(const void *a, const void *b)
{
    int64_t x = *(const int64_t *)a, y = *(const int64_t *)b;

    return x < y ? -1 : x > y;
}

static void sp_sink_pace_frame(const SpHdr *h)
{
    int64_t now = g_get_monotonic_time();
    int64_t base = MIN(sp_rx_min, sp_rx_min_prev);
    int64_t late, target, sorted[G_N_ELEMENTS(sp_pace.late)];

    if (sp_pace.mode < 0) {
        const char *m = getenv("SPACEBOX_PRESENT_JITTER");
        const char *x = getenv("SPACEBOX_PRESENT_JITTER_MAX_MS");

        sp_pace.mode = !m || !strcmp(m, "video") ? 1 : !strcmp(m, "always") ? 2 : 0;
        sp_pace.max_us = (x ? MAX(atoi(x), 0) : 50) * 1000;
    }
    spacebox_frame_due_us = 0;
    spacebox_frame_gap_us = 0;
    if (!h->t_us || base == INT64_MAX) {
        return;
    }
    if (sp_pace.last_t_us && h->t_us > sp_pace.last_t_us &&
        h->t_us - sp_pace.last_t_us < 200000) {
        sp_pace.gap[sp_pace.gap_pos] = h->t_us - sp_pace.last_t_us;
        sp_pace.gap_pos = (sp_pace.gap_pos + 1) % G_N_ELEMENTS(sp_pace.gap);
        sp_pace.gap_n = MIN(sp_pace.gap_n + 1, G_N_ELEMENTS(sp_pace.gap));
    } else {
        sp_pace.gap_n = sp_pace.gap_pos = 0; /* a pause: not a steady stream */
    }
    sp_pace.last_t_us = h->t_us;
    late = MAX(now - h->t_us - base, 0);
    sp_pace.late[sp_pace.pos] = late;
    sp_pace.pos = (sp_pace.pos + 1) % G_N_ELEMENTS(sp_pace.late);
    sp_pace.n = MIN(sp_pace.n + 1, G_N_ELEMENTS(sp_pace.late));
    sp_pace.frames++;

    if (sp_pace.mode == 0 || (sp_pace.mode == 1 && !(h->flags & SP_F_VIDEO))) {
        sp_pace.allowance = 0;
        return;
    }
    if (sp_pace.input_us < 0) {
        const char *i = getenv("SPACEBOX_PRESENT_INPUT_MS");

        sp_pace.input_us = (i ? MAX(atoi(i), 0) : 1500) * 1000;
    }
    if (sp_pace.input_us && spacebox_last_input_us &&
        now - spacebox_last_input_us < sp_pace.input_us) {
        sp_pace.unpaced_for_input++;
        return; /* the allowance is kept, so pacing picks up where it was */
    }
    memcpy(sorted, sp_pace.late, sp_pace.n * sizeof(sorted[0]));
    qsort(sorted, sp_pace.n, sizeof(sorted[0]), sp_cmp_i64);
    target = MIN(sorted[sp_pace.n * 95 / 100], sp_pace.max_us);
    if (sp_aud_delay_us) {
        /*
         * Sound is played later by its buffer; keep the picture with it. On top
         * of that the sound reaches the source later than the picture that
         * belongs to it (the guest's sound stack and QEMU's mixer hold it for
         * a while after the guest counts it as played) and the Mac's audio
         * output adds its own delay. Measured with a flash-and-beep clip the
         * sound was 41 ms behind before the output delay; SPACEBOX_AV_OFFSET_MS
         * (default 50) holds the picture back by that much more.
         */
        static int64_t av_offset = -1;

        if (av_offset < 0) {
            const char *o = getenv("SPACEBOX_AV_OFFSET_MS");

            av_offset = (o ? MAX(atoi(o), 0) : 50) * 1000;
        }
        target = MAX(target, sp_aud_delay_us + av_offset);
    }
    /* The presenter keeps the waiting frames as textures (SP_SLOTS in
     * ui/spacebox-presenter.h is 16); do not ask it to keep more than 12, nor
     * to wait longer than a quarter of a second. */
    target = MIN(target, MIN(MAX(12 * sp_pace.usual_gap, 40000), 250000));
    if (target > sp_pace.allowance) {
        sp_pace.allowance = target;
    } else {
        sp_pace.allowance -= (sp_pace.allowance - target) / 64;
    }
    spacebox_frame_gap_us = 0;
    if (sp_pace.gap_n >= 16) {
        memcpy(sorted, sp_pace.gap, sp_pace.gap_n * sizeof(sorted[0]));
        qsort(sorted, sp_pace.gap_n, sizeof(sorted[0]), sp_cmp_i64);
        spacebox_frame_gap_us = sp_pace.usual_gap = sorted[sp_pace.gap_n / 2];
    }
    if (late >= sp_pace.allowance) {
        sp_pace.late_frames++;
        /* later than the allowance: as soon as the spacing allows */
        spacebox_frame_due_us = now;
        return;
    }
    sp_pace.paced++;
    spacebox_frame_due_us = now - late + sp_pace.allowance;
}

typedef struct SpPending {
    struct virtio_gpu_ctrl_command *cmd;
    uint32_t type;
    uint32_t res;
    uint64_t off;
    uint64_t len;
    uint64_t seq;
    int64_t t0;
    bool early;     /* the guest already has its answer; cmd is NULL */
    bool fence_wait; /* fenced and waiting for the sink; counted in wait_out */
    bool hard;       /* ... and the sink's answer brings data; counted in hard_out */
} SpPending;

/* source: a guest read of a query result buffer whose result is not in yet */
typedef struct SpParked {
    struct virtio_gpu_ctrl_command *cmd;
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t res;
    int64_t t0;
} SpParked;

typedef struct SpShadow {
    uint8_t *ptr;
    size_t size;
} SpShadow;

static struct {
    int mode;
    bool mode_known;
    VirtIOGPU *g;
    char *path;
    int listen_fd;
    int fd;
    bool rx_busy;

    /*
     * Reliable messages stay in tx_queue until the peer acknowledges them, so
     * that a new connection can continue where the old one stopped. tx_next is
     * the first one not yet written to the current connection.
     */
    QemuMutex tx_lock;
    QemuCond tx_cond;
    GQueue tx_queue;
    GList *tx_next;
    bool tx_enabled;
    size_t tx_backlog;      /* bytes not yet written */
    size_t tx_unacked;      /* bytes written or not, not yet acknowledged */
    QemuThread tx_thread;
    uint64_t tx_id;
    uint64_t rx_id;
    uint64_t rx_id_acked;   /* last value of rx_id the peer was told */
    uint64_t session;       /* own */
    uint64_t peer_session;  /* the peer this state belongs to, 0 = none */
    bool hello_done;
    bool broken;            /* source: the sink lost the state; restart needed */
    QEMUTimer *ack_timer;
    uint64_t n_reconnect, n_resent;

    GByteArray *rx;

    /* compression of the connection */
    int z_level;            /* 0 = not offered */
    bool z_on;              /* both ends offered it (set with the peer's HELLO) */
    uint64_t conn_gen;      /* counts connections: each starts new zstd streams */
    GByteArray *rxw;        /* bytes as received, before sp_rx_unwrap() */
    bool rx_hello_in;       /* the peer's HELLO has been taken out of rxw */
    uint64_t wire_tx, wire_rx; /* bytes written to / read from the socket */
#ifdef SPACEBOX_ZSTD
    ZSTD_DCtx *z_d;
#endif

    uint64_t seq;
    GHashTable *pending;
    QEMUTimer *retry;

    GHashTable *queries; /* source: (ctx << 32 | query handle) -> resource id */
    GHashTable *query_bufs; /* source: resource ids used as query result buffers */
    GHashTable *custom_bufs; /* source: buffers that live only in renderer memory */
    uint64_t n_defer, defer_bytes;
    GQueue parked;          /* source: SpParked */
    char ctx_name[64][24];  /* source, diagnostics: guest context names */
    uint32_t ctx_qpoll[64]; /* source, diagnostics: query polls answered here */
    struct {
        uint32_t id, ver, size;
        uint8_t *data;
    } caps[4];           /* source: capsets as reported by the sink */
    GByteArray *dinfo;   /* source: display info response of the sink */
    GByteArray *edid;    /* source: EDID response of the sink */

    GHashTable *shadow;
    GHashTable *watch;   /* sink: resource id -> SpWatch */
    GByteArray *next_expect;
    QemuInputHandlerState *input;

    size_t tx_limit;
    unsigned fence_window;  /* source */
    unsigned fence_out;     /* source: answered early, not yet finished by the sink */
    unsigned wait_out;      /* source: fenced commands waiting for the sink */
    unsigned hard_out;      /* source: ... of which the answer brings data */
    uint64_t n_early, n_local, n_waited, n_ahead;
    /* source: send times of fenced commands the sink has not answered yet, oldest first */
    int64_t fq[512];
    unsigned fq_head, fq_len;
    int64_t fence_age_us;   /* source: see sp_source_may_answer_early() */
    int64_t rtt_min_cur, rtt_min_prev, rtt_min_t0;
    int64_t rtt_us;         /* source: last measured command round trip */
    bool stats;
    QEMUTimer *stats_timer;
    uint64_t n_ctrl, n_submit, submit_bytes, n_xfer, xfer_bytes, tx_bytes, rx_bytes;
    uint64_t n_mem, mem_bytes, n_back, back_bytes;
} sp = { .fd = -1, .listen_fd = -1 };

int sp_remote_mode(void)
{
    if (!sp.mode_known) {
        const char *p;

        sp.mode_known = true;
        if ((p = getenv("SPACEBOX_GPU_REMOTE_SOURCE")) && *p) {
            sp.mode = SP_REMOTE_SOURCE;
            sp.path = g_strdup(p);
        } else if ((p = getenv("SPACEBOX_GPU_REMOTE_SINK")) && *p) {
            sp.mode = SP_REMOTE_SINK;
            sp.path = g_strdup(p);
        }
        sp.stats = getenv("SPACEBOX_GPU_REMOTE_STATS") != NULL;
        p = getenv("SPACEBOX_GPU_REMOTE_FENCE_WINDOW");
        sp.fence_window = p ? MAX(atoi(p), 0) : 8;
        p = getenv("SPACEBOX_GPU_REMOTE_FENCE_AGE_MS");
        sp.fence_age_us = (p ? MAX(atoi(p), 0) : 20) * 1000;
        sp.rtt_min_cur = sp.rtt_min_prev = INT64_MAX;
        p = getenv("SPACEBOX_GPU_REMOTE_TX_LIMIT_MB");
        sp.tx_limit = (size_t)MAX(p ? atoi(p) : 8, 1) << 20;
#ifdef SPACEBOX_ZSTD
        p = getenv("SPACEBOX_GPU_REMOTE_ZSTD");
        sp.z_level = p ? CLAMP(atoi(p), 0, 19) : 1;
#endif
    }
    return sp.mode;
}

/* ---- transport ---------------------------------------------------------- */

#ifdef SPACEBOX_ZSTD
/*
 * Picture data (texture uploads) is most of what a page full of images sends,
 * and a general compressor does little with it. Each byte is replaced by its
 * difference from a prediction made of the pixel to the left, the pixel above
 * and the pixel above left (left + above - above left, per byte, wrapping).
 * What remains is mostly small numbers, which zstd then packs well; measured
 * on the uploads of a page of photos, 8.2 times smaller against 5.3 times for
 * the zlib compression of SSH, at seven times its speed.
 *
 * The same thing in two steps, which is how it is undone: e[i] = s[i] -
 * s[i - stride], then d[i] = e[i] - e[i - bpp]; a term whose index is negative
 * is 0. stride 0 leaves out the first step. A wrong bpp or stride only makes
 * the result compress less; the bytes always come back exactly.
 */
static void sp_z_filter(const uint8_t *s, uint8_t *d, size_t n, size_t bpp,
                        size_t stride)
{
    size_t i;

    for (i = 0; i < n && i < stride + bpp; i++) {
        uint8_t e = s[i] - (stride && i >= stride ? s[i - stride] : 0);
        uint8_t el = i < bpp ? 0 :
            s[i - bpp] - (stride && i >= stride + bpp ? s[i - bpp - stride] : 0);

        d[i] = e - el;
    }
    if (stride) {
        for (; i < n; i++) {
            d[i] = s[i] - s[i - stride] - s[i - bpp] + s[i - bpp - stride];
        }
    } else {
        for (; i < n; i++) {
            d[i] = s[i] - s[i - bpp];
        }
    }
}

static void sp_z_unfilter(uint8_t *d, size_t n, size_t bpp, size_t stride)
{
    size_t i;

    for (i = bpp; i < n; i++) {
        d[i] += d[i - bpp];
    }
    if (stride) {
        for (i = stride; i < n; i++) {
            d[i] += d[i - stride];
        }
    }
}

/*
 * The writer's side of a compressed connection: one message into *out as
 * SpZHdr [SpZFilter] data. Returns the number of bytes, 0 on failure.
 */
static size_t sp_z_pack(ZSTD_CCtx *c, const SpBuf *b, uint8_t **out,
                        size_t *out_cap, uint8_t **tmp, size_t *tmp_cap)
{
    bool filt = b->f_bpp && b->f_off >= sizeof(SpHdr) && b->f_off < b->len;
    size_t hl = sizeof(SpZHdr) + (filt ? sizeof(SpZFilter) : 0);
    size_t need = hl + b->len + (b->len >> 7) + 1024;
    SpZHdr zh;
    ZSTD_inBuffer in;
    ZSTD_outBuffer o;
    size_t r;

    if (b->len >= SP_Z_MAX) {
        return 0;
    }
    if (*out_cap < need) {
        *out_cap = need;
        *out = g_realloc(*out, need);
    }
    o = (ZSTD_outBuffer){ *out + hl, *out_cap - hl, 0 };
    if (filt) {
        size_t n = b->len - b->f_off;
        SpZFilter zf = { .off = b->f_off, .bpp = b->f_bpp, .stride = b->f_stride };

        if (*tmp_cap < n) {
            *tmp_cap = n;
            *tmp = g_realloc(*tmp, n);
        }
        sp_z_filter(b->data + b->f_off, *tmp, n, b->f_bpp, b->f_stride);
        memcpy(*out + sizeof(zh), &zf, sizeof(zf));
        in = (ZSTD_inBuffer){ b->data, b->f_off, 0 };
        while (in.pos < in.size) {
            r = ZSTD_compressStream2(c, &o, &in, ZSTD_e_continue);
            if (ZSTD_isError(r) || o.pos == o.size) {
                return 0;
            }
        }
        in = (ZSTD_inBuffer){ *tmp, n, 0 };
    } else {
        in = (ZSTD_inBuffer){ b->data, b->len, 0 };
    }
    do {
        r = ZSTD_compressStream2(c, &o, &in, ZSTD_e_flush);
        if (ZSTD_isError(r) || (r && o.pos == o.size)) {
            return 0;
        }
    } while (r);
    zh.clen = o.pos | (filt ? SP_Z_FILTERED : 0);
    zh.rlen = b->len;
    memcpy(*out, &zh, sizeof(zh));
    return hl + o.pos;
}

/*
 * The reader's side: move what has arrived (sp.rxw) into sp.rx as plain
 * messages. 1 = moved something, 0 = nothing complete yet, -1 = bad data.
 */
static int sp_z_unpack(void)
{
    size_t off = 0;

    while (sp.rxw->len - off >= sizeof(SpZHdr)) {
        SpZHdr zh;
        SpZFilter zf = { 0 };
        size_t hl = sizeof(zh), clen, at;
        ZSTD_inBuffer in;
        ZSTD_outBuffer o;

        memcpy(&zh, sp.rxw->data + off, sizeof(zh));
        clen = zh.clen & ~SP_Z_FILTERED;
        if (zh.clen & SP_Z_FILTERED) {
            hl += sizeof(zf);
        }
        if (zh.rlen < sizeof(SpHdr) || zh.rlen >= SP_Z_MAX ||
            clen > (size_t)zh.rlen + (zh.rlen >> 7) + 1024) {
            return -1;
        }
        if (sp.rxw->len - off < hl + clen) {
            break;
        }
        if (zh.clen & SP_Z_FILTERED) {
            memcpy(&zf, sp.rxw->data + off + sizeof(zh), sizeof(zf));
            if (zf.off < sizeof(SpHdr) || zf.off >= zh.rlen || !zf.bpp ||
                zf.bpp > 64) {
                return -1;
            }
        }
        at = sp.rx->len;
        g_byte_array_set_size(sp.rx, at + zh.rlen);
        in = (ZSTD_inBuffer){ sp.rxw->data + off + hl, clen, 0 };
        o = (ZSTD_outBuffer){ sp.rx->data + at, zh.rlen, 0 };
        while (in.pos < in.size || o.pos < o.size) {
            size_t before = in.pos + o.pos;
            size_t r = ZSTD_decompressStream(sp.z_d, &o, &in);

            if (ZSTD_isError(r) || in.pos + o.pos == before) {
                g_byte_array_set_size(sp.rx, at);
                return -1;
            }
        }
        if (zf.bpp) {
            sp_z_unfilter(sp.rx->data + at + zf.off, zh.rlen - zf.off, zf.bpp,
                          zf.stride);
        }
        off += hl + clen;
    }
    if (off) {
        g_byte_array_remove_range(sp.rxw, 0, off);
    }
    return off ? 1 : 0;
}
#endif

/*
 * Move received bytes from sp.rxw to sp.rx. The first message of a connection
 * is the peer's HELLO, never compressed; it is handed over alone, because it
 * decides how the rest is to be read.
 */
static int sp_rx_unwrap(void)
{
    if (!sp.rx_hello_in) {
        if (sp.rxw->len < sizeof(SpHdr)) {
            return 0;
        }
        g_byte_array_append(sp.rx, sp.rxw->data, sizeof(SpHdr));
        g_byte_array_remove_range(sp.rxw, 0, sizeof(SpHdr));
        sp.rx_hello_in = true;
        return 1;
    }
#ifdef SPACEBOX_ZSTD
    if (sp.z_on) {
        return sp_z_unpack();
    }
#endif
    if (!sp.rxw->len) {
        return 0;
    }
    g_byte_array_append(sp.rx, sp.rxw->data, sp.rxw->len);
    g_byte_array_set_size(sp.rxw, 0);
    return 1;
}

static void *sp_tx_thread(void *opaque)
{
#ifdef SPACEBOX_ZSTD
    ZSTD_CCtx *zc = ZSTD_createCCtx();
    uint64_t zgen = 0;
    uint8_t *zout = NULL, *ztmp = NULL;
    size_t zout_cap = 0, ztmp_cap = 0;
#endif

    for (;;) {
        SpBuf *b;
        int fd;
        size_t done = 0;
        const uint8_t *wire;
        size_t wire_len;
        bool z;
        uint64_t gen;

        qemu_mutex_lock(&sp.tx_lock);
        while (!sp.tx_enabled || sp.fd < 0 || !sp.tx_next) {
            qemu_cond_wait(&sp.tx_cond, &sp.tx_lock);
        }
        b = sp.tx_next->data;
        sp.tx_next = sp.tx_next->next;
        b->writing = true;
        fd = sp.fd;
        z = sp.z_on;
        gen = sp.conn_gen;
        qemu_mutex_unlock(&sp.tx_lock);

        {
            /* SPACEBOX_GPU_REMOTE_DUMP=<file>: copy of everything sent, to study
             * how well the stream compresses */
            static FILE *dump;
            static bool tried;

            if (!tried) {
                const char *path = getenv("SPACEBOX_GPU_REMOTE_DUMP");

                tried = true;
                dump = path ? fopen(path, "wb") : NULL;
            }
            if (dump) {
                fwrite(b->data, 1, b->len, dump);
            }
        }
        wire = b->data;
        wire_len = b->len;
        (void)gen;
#ifdef SPACEBOX_ZSTD
        if (z) {
            if (gen != zgen) {
                zgen = gen;
                ZSTD_CCtx_reset(zc, ZSTD_reset_session_only);
                ZSTD_CCtx_setParameter(zc, ZSTD_c_compressionLevel, sp.z_level);
            }
            wire_len = sp_z_pack(zc, b, &zout, &zout_cap, &ztmp, &ztmp_cap);
            wire = zout;
            if (!wire_len) {
                /* cannot happen with a sane library; drop the connection so
                 * that both streams start again */
                error_report("spacebox remote gpu: compression failed");
                shutdown(fd, SHUT_RDWR);
                zgen = 0;
            }
        }
#else
        (void)z;
#endif
        while (done < wire_len) {
            ssize_t n = send(fd, wire + done, wire_len - done, 0);

            if (n > 0) {
                done += n;
            } else if (n < 0 && (errno == EINTR || errno == EAGAIN)) {
                continue;
            } else {
                break; /* the reader notices the closed connection */
            }
        }

        qemu_mutex_lock(&sp.tx_lock);
        b->writing = false;
        sp.wire_tx += done;
        {
            int64_t d = g_get_monotonic_time() - b->t_us;

            sp_txd.n++;
            sp_txd.over12 += d > 12000;
            sp_txd.over25 += d > 25000;
            sp_txd.over50 += d > 50000;
            sp_txd.max = MAX(sp_txd.max, d);
        }
        if (sp.tx_backlog >= b->len) {
            sp.tx_backlog -= b->len;
        } else {
            sp.tx_backlog = 0;
        }
        if (!b->id) {
            /* not resent: forget it once written (or once the write failed) */
            GList *l = g_queue_find(&sp.tx_queue, b);
            if (l) {
                if (sp.tx_next == l) {
                    sp.tx_next = l->next;
                }
                g_queue_delete_link(&sp.tx_queue, l);
                sp.tx_unacked -= MIN(sp.tx_unacked, b->len);
                g_free(b);
            }
        }
        qemu_mutex_unlock(&sp.tx_lock);
    }
    return NULL;
}

/* reliable = false: sent at most once, on the current connection only. */
static void sp_send_full(bool reliable, uint32_t type, uint64_t seq,
                         const void *a, size_t a_len,
                         const void *b, size_t b_len,
                         uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    SpHdr h = {
        .magic = SP_MAGIC, .type = type, .seq = seq,
        .a_len = a_len, .b_len = b_len,
        .arg0 = arg0, .arg1 = arg1, .arg2 = arg2,
        .t_us = g_get_monotonic_time(),
        .flags = sp_video_until > g_get_monotonic_time() ? SP_F_VIDEO : 0,
    };
    SpBuf *buf;
    GList *link;

    if (!reliable && sp.fd < 0) {
        return;
    }
    buf = g_malloc0(sizeof(*buf) + sizeof(h) + a_len + b_len);
    buf->t_us = h.t_us;
    buf->len = sizeof(h) + a_len + b_len;
    if (sp_tx_hint.bpp && (sp_tx_hint.in_b ? b_len : a_len) >= 4096) {
        buf->f_off = sizeof(h) + (sp_tx_hint.in_b ? a_len : 0);
        buf->f_bpp = sp_tx_hint.bpp;
        buf->f_stride = sp_tx_hint.stride;
    }
    sp_tx_hint.bpp = 0;
    if (a_len) {
        memcpy(buf->data + sizeof(h), a, a_len);
    }
    if (b_len) {
        memcpy(buf->data + sizeof(h) + a_len, b, b_len);
    }
    sp.tx_bytes += buf->len;

    qemu_mutex_lock(&sp.tx_lock);
    buf->id = h.id = reliable ? ++sp.tx_id : 0;
    h.ack = sp.rx_id;
    sp.rx_id_acked = sp.rx_id;
    memcpy(buf->data, &h, sizeof(h));
    g_queue_push_tail(&sp.tx_queue, buf);
    link = g_queue_peek_tail_link(&sp.tx_queue);
    if (!sp.tx_next) {
        sp.tx_next = link;
    }
    sp.tx_backlog += buf->len;
    sp.tx_unacked += buf->len;
    qemu_cond_signal(&sp.tx_cond);
    qemu_mutex_unlock(&sp.tx_lock);
}

static void sp_send(uint32_t type, uint64_t seq,
                    const void *a, size_t a_len, const void *b, size_t b_len,
                    uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
    sp_send_full(true, type, seq, a, a_len, b, b_len, arg0, arg1, arg2);
}

static size_t sp_tx_backlog(void)
{
    size_t n;

    qemu_mutex_lock(&sp.tx_lock);
    n = sp.tx_backlog;
    qemu_mutex_unlock(&sp.tx_lock);
    return n;
}

/* The peer has everything up to id: drop it from the resend queue. */
static void sp_acked(uint64_t id)
{
    GList *l;

    qemu_mutex_lock(&sp.tx_lock);
    for (l = sp.tx_queue.head; l && l != sp.tx_next; ) {
        SpBuf *b = l->data;
        GList *next = l->next;

        if (b->id && b->id <= id && !b->writing) {
            sp.tx_unacked -= MIN(sp.tx_unacked, b->len);
            g_queue_delete_link(&sp.tx_queue, l);
            g_free(b);
        } else if (b->id > id) {
            break;
        }
        l = next;
    }
    qemu_mutex_unlock(&sp.tx_lock);
}

static void sp_source_unpark(VirtIOGPU *g, uint32_t res, int64_t max_age_us);

static void sp_ack_timer(void *opaque)
{
    if (sp.mode == SP_REMOTE_SOURCE && sp.g && !g_queue_is_empty(&sp.parked)) {
        sp_source_unpark(sp.g, 0, 200000);
    }
    if (sp.fd >= 0 && sp.hello_done && sp.rx_id != sp.rx_id_acked) {
        sp_send_full(false, SP_ACK, 0, NULL, 0, NULL, 0, 0, 0, 0);
    }
    timer_mod(sp.ack_timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 100);
}

static void sp_dispatch(SpHdr *h, uint8_t *a, uint8_t *b);
static void sp_hello(SpHdr *h);
static void sp_disconnected(void);

static void sp_rx(void *opaque)
{
    static uint8_t tmp[1 << 18];
    size_t off = 0, got = 0;
    bool closed = false;
    int unwrapped = 0;

    if (sp.fd < 0 || sp.rx_busy) {
        return;
    }
    while (got < (8u << 20)) {
        ssize_t n = recv(sp.fd, tmp, sizeof(tmp), MSG_DONTWAIT);

        if (n > 0) {
            g_byte_array_append(sp.rxw, tmp, n);
            got += n;
        } else if (n == 0) {
            closed = true;
            break;
        } else if (errno == EINTR) {
            continue;
        } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
            break;
        } else {
            closed = true;
            break;
        }
    }
    sp.wire_rx += got;

    sp.rx_busy = true;
  more:
    if (!closed) {
        size_t had = sp.rx->len;

        unwrapped = sp_rx_unwrap();
        if (unwrapped < 0) {
            error_report("spacebox remote gpu: bad compressed data, closing");
            closed = true;
        } else {
            sp.rx_bytes += sp.rx->len - had;
        }
    }
    while (!closed && sp.rx->len - off >= sizeof(SpHdr)) {
        SpHdr h;
        size_t total;

        memcpy(&h, sp.rx->data + off, sizeof(h));
        if (h.magic != SP_MAGIC) {
            error_report("spacebox remote gpu: bad frame, closing");
            closed = true;
            break;
        }
        total = sizeof(h) + (size_t)h.a_len + h.b_len;
        if (sp.rx->len - off < total) {
            break;
        }
        off += total;
        if (h.type == SP_HELLO) {
            sp_hello(&h);
            continue;
        }
        if (!sp.hello_done) {
            continue;
        }
        sp_acked(h.ack);
        if (h.id) {
            if (h.id <= sp.rx_id) {
                continue; /* resent, already handled */
            }
            if (h.id != sp.rx_id + 1) {
                error_report("spacebox remote gpu: message %" PRIu64
                             " after %" PRIu64 ", closing", h.id, sp.rx_id);
                closed = true;
                break;
            }
            sp.rx_id = h.id;
        }
        if (h.type != SP_ACK) {
            sp_rx_delay(h.t_us);
            sp_dispatch(&h, sp.rx->data + off - total + sizeof(h),
                        sp.rx->data + off - total + sizeof(h) + h.a_len);
        }
        if (sp.rx_id - sp.rx_id_acked >= 256) {
            sp_send_full(false, SP_ACK, 0, NULL, 0, NULL, 0, 0, 0, 0);
        }
    }
    if (off) {
        g_byte_array_remove_range(sp.rx, 0, off);
        off = 0;
    }
    if (!closed && unwrapped > 0 && sp.fd >= 0) {
        goto more;
    }
    sp.rx_busy = false;
    if (closed) {
        sp_disconnected();
    }
}

static void sp_set_connected(int fd)
{
    int yes = 1;

#ifdef SO_NOSIGPIPE
    setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &yes, sizeof(yes));
#endif
    (void)yes;
    g_byte_array_set_size(sp.rx, 0);
    g_byte_array_set_size(sp.rxw, 0);
    sp.rx_hello_in = false;
#ifdef SPACEBOX_ZSTD
    ZSTD_DCtx_reset(sp.z_d, ZSTD_reset_session_only);
#endif
    qemu_mutex_lock(&sp.tx_lock);
    sp.fd = fd;
    sp.hello_done = false;
    sp.tx_enabled = false;
    sp.z_on = false;
    sp.conn_gen++;
    qemu_mutex_unlock(&sp.tx_lock);
    qemu_set_fd_handler(fd, sp_rx, NULL, NULL);

    /* HELLO goes out directly: the queue stays closed until the peer answers. */
    {
        SpHdr h = {
            .magic = SP_MAGIC, .type = SP_HELLO,
            .arg0 = sp.session, .arg1 = sp.peer_session, .arg2 = sp.rx_id,
            .flags = sp.z_level ? SP_F_ZSTD : 0,
        };
        size_t done = 0;

        while (done < sizeof(h)) {
            ssize_t n = send(fd, (uint8_t *)&h + done, sizeof(h) - done, 0);
            if (n > 0) {
                done += n;
            } else if (n < 0 && errno == EINTR) {
                continue;
            } else {
                break;
            }
        }
    }
}

/* Open the queue: everything the peer does not have yet goes out (again). */
static void sp_tx_start(uint64_t peer_has)
{
    GList *l;
    size_t unsent = 0;

    sp_acked(peer_has);
    qemu_mutex_lock(&sp.tx_lock);
    for (l = sp.tx_queue.head; l; ) {
        SpBuf *b = l->data;
        GList *next = l->next;

        if (!b->id && !b->writing) {
            /* one-shot messages of the old connection are stale */
            sp.tx_unacked -= MIN(sp.tx_unacked, b->len);
            g_queue_delete_link(&sp.tx_queue, l);
            g_free(b);
        } else {
            unsent += b->len;
        }
        l = next;
    }
    sp.tx_next = sp.tx_queue.head;
    sp.tx_backlog = unsent;
    sp.tx_enabled = true;
    qemu_cond_signal(&sp.tx_cond);
    qemu_mutex_unlock(&sp.tx_lock);
}

static void sp_tx_forget(void)
{
    GList *l;

    qemu_mutex_lock(&sp.tx_lock);
    for (l = sp.tx_queue.head; l; ) {
        SpBuf *b = l->data;
        GList *next = l->next;

        if (!b->writing) {
            g_queue_delete_link(&sp.tx_queue, l);
            g_free(b);
        } else {
            b->id = 0; /* the writer frees it */
        }
        l = next;
    }
    sp.tx_next = NULL;
    sp.tx_backlog = sp.tx_unacked = 0;
    sp.tx_id = 0;
    qemu_mutex_unlock(&sp.tx_lock);
    sp.rx_id = sp.rx_id_acked = 0;
}

/* ---- byte range of a transfer (same arithmetic as vrend_transfer_size) --- */

static void sp_range(uint32_t format, uint32_t width0, uint32_t height0,
                     uint32_t level, uint32_t stride, uint32_t layer_stride,
                     const struct virtio_gpu_box *box, uint64_t offset,
                     size_t backing, uint64_t *off, uint64_t *len)
{
    uint64_t size;

    *off = offset;
    *len = 0;
    if (offset >= backing) {
        return;
    }
    if (format < SP_FORMAT_COUNT && sp_format_blocks[format].bits >= 8) {
        uint64_t bw = sp_format_blocks[format].bw;
        uint64_t bh = sp_format_blocks[format].bh;
        uint64_t bs = sp_format_blocks[format].bits / 8;
        uint64_t w = box->w ? box->w : 1;
        uint64_t h = box->h ? box->h : 1;
        uint64_t d = box->d ? box->d : 1;
        uint64_t lw = MAX(level < 32 ? width0 >> level : 0, 1);
        uint64_t lh = MAX(level < 32 ? height0 >> level : 0, 1);
        uint64_t vstride = stride ? stride : ((lw + bw - 1) / bw) * bs;
        uint64_t vlayer = layer_stride ? layer_stride
                                       : ((lh + bh - 1) / bh) * vstride;

        size = (d - 1) * vlayer + ((h + bh - 1) / bh - 1) * vstride +
               ((w + bw - 1) / bw) * bs;
    } else {
        size = backing - offset; /* unknown layout: everything from offset */
    }
    *len = MIN(size, backing - offset);
}

/* ---- source ------------------------------------------------------------- */

static void sp_source_res_create(VirtIOGPU *g, uint32_t id, uint32_t format,
                                 uint32_t width, uint32_t height)
{
    struct virtio_gpu_simple_resource *res;

    if (!id || virtio_gpu_find_resource(g, id)) {
        return;
    }
    res = g_new0(struct virtio_gpu_simple_resource, 1);
    res->resource_id = id;
    res->format = format;
    res->width = width;
    res->height = height;
    res->dmabuf_fd = -1;
    QTAILQ_INSERT_HEAD(&g->reslist, res, next);
}

static void sp_source_res_unmap(VirtIOGPU *g,
                                struct virtio_gpu_simple_resource *res)
{
    if (res->iov) {
        virtio_gpu_cleanup_mapping_iov(g, res->iov, res->iov_cnt);
        res->iov = NULL;
        res->iov_cnt = 0;
    }
}

void sp_source_resource_destroy(VirtIOGPU *g,
                                struct virtio_gpu_simple_resource *res,
                                Error **errp)
{
    sp_source_res_unmap(g, res);
    QTAILQ_REMOVE(&g->reslist, res, next);
    g_free(res);
}

/*
 * The data about to be sent is pixels of a texture: note their layout for
 * sp_z_filter(). stride 0 = the rows are as wide as the texture's level.
 * Buffers (one row) and block-compressed formats are left alone.
 */
static void sp_source_hint(const struct virtio_gpu_simple_resource *tex,
                           uint32_t level, uint32_t stride, bool in_b)
{
    sp_tx_hint.bpp = 0;
    if (tex && tex->height > 1 && tex->format < SP_FORMAT_COUNT &&
        sp_format_blocks[tex->format].bits >= 8 &&
        sp_format_blocks[tex->format].bw == 1 &&
        sp_format_blocks[tex->format].bh == 1) {
        uint32_t bs = sp_format_blocks[tex->format].bits / 8;
        uint32_t lw = MAX(level < 32 ? tex->width >> level : 0, 1);

        sp_tx_hint.bpp = bs;
        sp_tx_hint.stride = stride ? stride : lw * bs;
        sp_tx_hint.in_b = in_b;
    }
}

/* Send a piece of a resource's guest memory to the sink's copy of it. */
static void sp_source_push_mem(VirtIOGPU *g, uint32_t res_id, uint64_t off,
                               uint64_t len, uint64_t watch)
{
    struct virtio_gpu_simple_resource *res = virtio_gpu_find_resource(g, res_id);
    g_autofree uint8_t *data = NULL;

    if (!res || !res->iov || !len) {
        return;
    }
    data = g_malloc(len);
    len = iov_to_buf(res->iov, res->iov_cnt, off, data, len);
    sp.n_mem++;
    sp.mem_bytes += len;
    sp_send(SP_MEMWRITE, 0, data, len, NULL, 0, res_id, off, watch);
}

/*
 * A buffer created with VIRGL_BIND_CUSTOM exists only in the renderer's memory.
 * The renderer uses its contents in two cases: query results (16 bytes) and
 * video decode, where it reads the buffer from guest memory again when the
 * decode command arrives. The guest's video driver uploads the whole bitstream
 * buffer for every picture (twice the picture's pixel count in bytes; 4 MB at
 * 1080p) although the compressed picture is a few kB. Such an upload carries
 * nothing the renderer will use, so it is not sent; the decode command sends
 * the bytes that are needed.
 */
static bool sp_source_defer(uint32_t res_id, uint64_t len)
{
    if (len < 4096 ||
        !g_hash_table_contains(sp.custom_bufs, GUINT_TO_POINTER(res_id))) {
        return false;
    }
    sp.n_defer++;
    sp.defer_bytes += len;
    return true;
}

/*
 * The virgl command stream can make the renderer read or write a resource's
 * guest memory without any virtio-gpu transfer command: in-stream transfers,
 * copy transfers through a staging buffer, and query results. Find them, send
 * what the renderer will read, and list what it will write.
 */
static bool sp_source_scan_submit(VirtIOGPU *g, uint32_t ctx_id,
                                  const uint8_t *bytes, size_t size,
                                  GArray *expect)
{
    size_t ndw = size / 4, i = 0;
    bool wants_result = false;

    while (i < ndw) {
        uint32_t c[16] = { 0 };
        uint32_t hdr, len, cmd, obj;
        struct virtio_gpu_simple_resource *a, *b;
        struct virtio_gpu_box box;
        uint64_t off = 0, n = 0;

        memcpy(&hdr, bytes + i * 4, 4);
        len = hdr >> 16;
        cmd = hdr & 0xff;
        obj = (hdr >> 8) & 0xff;
        if (i + 1 + len > ndw) {
            break;
        }
        memcpy(c, bytes + i * 4, MIN((size_t)len + 1, G_N_ELEMENTS(c)) * 4);
        box = (struct virtio_gpu_box){ c[6], c[7], c[8], c[9], c[10], c[11] };

        switch (cmd) {
        case 43: /* VIRGL_CCMD_TRANSFER3D: res, level, usage, stride, layer_stride,
                    box, data offset, direction */
            a = len >= 13 ? virtio_gpu_find_resource(g, c[1]) : NULL;
            if (a && a->iov) {
                sp_range(a->format, a->width, a->height, c[2], c[4], c[5], &box,
                         c[12], iov_size(a->iov, a->iov_cnt), &off, &n);
                if (c[13] == 1) {
                    if (sp_source_defer(c[1], n)) {
                        break;
                    }
                    sp_source_hint(a, c[2], c[4], false);
                    sp_source_push_mem(g, c[1], off, n, 0);
                } else if (n && !g_hash_table_contains(sp.query_bufs,
                                                       GUINT_TO_POINTER(c[1]))) {
                    SpExpect e = { .res = c[1], .off = off, .len = n };
                    g_array_append_val(expect, e);
                }
            }
            break;
        case 45: /* VIRGL_CCMD_COPY_TRANSFER3D: texture, level, usage, stride,
                    layer_stride, box, staging resource, staging offset, flags */
            a = len >= 14 ? virtio_gpu_find_resource(g, c[1]) : NULL;
            b = len >= 14 ? virtio_gpu_find_resource(g, c[12]) : NULL;
            if (a && b && b->iov) {
                sp_range(a->format, a->width, a->height, c[2], c[4], c[5], &box,
                         c[13], iov_size(b->iov, b->iov_cnt), &off, &n);
                if (!(c[14] & 2)) {
                    sp_source_hint(a, c[2], c[4], false);
                    sp_source_push_mem(g, c[12], off, n, 0);
                } else if (n) {
                    SpExpect e = { .res = c[12], .off = off, .len = n };
                    g_array_append_val(expect, e);
                }
            }
            break;
        case 1: /* VIRGL_CCMD_CREATE_OBJECT, VIRGL_OBJECT_QUERY: handle, type,
                   offset, result resource */
            if (obj == 9 && len >= 4) {
                uint64_t key = ((uint64_t)ctx_id << 32) | c[1];
                g_hash_table_insert(sp.queries, g_memdup2(&key, sizeof(key)),
                                    GUINT_TO_POINTER(c[4]));
                g_hash_table_add(sp.query_bufs, GUINT_TO_POINTER(c[4]));
                if (getenv("SPACEBOX_GPU_REMOTE_DEBUG_WAIT")) {
                    fprintf(stderr, "[SPACEBOX-QUERY] create ctx=%u (%s) type=%u res=%u\n",
                            ctx_id, ctx_id < 64 ? sp.ctx_name[ctx_id] : "?",
                            c[2] & 0xffff, c[4]);
                }
            }
            break;
        case 3: /* VIRGL_CCMD_DESTROY_OBJECT */
            if (obj == 9 && len >= 1) {
                uint64_t key = ((uint64_t)ctx_id << 32) | c[1];
                g_hash_table_remove(sp.queries, &key);
            }
            break;
        case 21: /* VIRGL_CCMD_GET_QUERY_RESULT: the guest has just marked its
                    result buffer "waiting"; the renderer writes the result there. */
            if (len >= 1) {
                uint64_t key = ((uint64_t)ctx_id << 32) | c[1];
                uint32_t res_id =
                    GPOINTER_TO_UINT(g_hash_table_lookup(sp.queries, &key));
                /*
                 * The result comes back on its own when the renderer writes it
                 * (the sink watches the buffer). The guest polls for it; that
                 * poll must not cost a network round trip per attempt.
                 */
                if (res_id) {
                    sp_source_push_mem(g, res_id, 0, 16, 1);
                }
            }
            break;
        case 59: /* VIRGL_CCMD_DECODE_BITSTREAM: decoder, target, description
                    buffer, bitstream buffer, bitstream size. The renderer reads
                    both buffers from guest memory when it gets this command. */
            sp_video_until = g_get_monotonic_time() + G_USEC_PER_SEC;
            if (len >= 5) {
                a = virtio_gpu_find_resource(g, c[3]);
                if (a && a->iov) {
                    sp_source_push_mem(g, c[3], 0,
                                       MIN(iov_size(a->iov, a->iov_cnt),
                                           (size_t)a->width), 0);
                }
                b = virtio_gpu_find_resource(g, c[4]);
                if (b && b->iov) {
                    sp_source_push_mem(g, c[4], 0,
                                       MIN(iov_size(b->iov, b->iov_cnt),
                                           (size_t)c[5]), 0);
                }
            }
            break;
        case 50: /* VIRGL_CCMD_GET_MEMORY_INFO: resource */
            a = len >= 1 ? virtio_gpu_find_resource(g, c[1]) : NULL;
            if (a && a->iov) {
                SpExpect e = { .res = c[1], .off = 0,
                               .len = MIN(iov_size(a->iov, a->iov_cnt), 64) };
                g_array_append_val(expect, e);
            }
            break;
        default:
            break;
        }
        i += 1 + (size_t)len;
    }
    return wants_result || expect->len;
}

/* Spacing of the guest's screen updates (RESOURCE_FLUSH), as seen on this side. */
static struct {
    int64_t last;
    unsigned n, over12, over25, over50;
    int64_t max;
} sp_flush;

/* source: time from sending a command to the sink's answer.
 * sink: how late a 2 ms timer fires, i.e. how long the main loop was held up. */
static struct {
    unsigned n, over12, over25, over50;
    int64_t max;
} sp_delay;
static QEMUTimer *sp_late_timer;
static int64_t sp_late_due;

static void sp_late_tick(void *opaque)
{
    int64_t now = g_get_monotonic_time();
    int64_t late = now - sp_late_due;

    sp_delay.n++;
    sp_delay.over12 += late > 12000;
    sp_delay.over25 += late > 25000;
    sp_delay.over50 += late > 50000;
    sp_delay.max = MAX(sp_delay.max, late);
    sp_late_due = now + 2000;
    timer_mod(sp_late_timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 2);
}

static void sp_flush_seen(void)
{
    int64_t now = g_get_monotonic_time();

    if (sp_flush.last) {
        int64_t d = now - sp_flush.last;

        sp_flush.n++;
        sp_flush.over12 += d > 12000;
        sp_flush.over25 += d > 25000;
        sp_flush.over50 += d > 50000;
        sp_flush.max = MAX(sp_flush.max, d);
    }
    sp_flush.last = now;
}

static void sp_flush_report(const char *who)
{
    fprintf(stderr, "[SPACEBOX-REMOTE] %s screen updates=%u gaps over 12 ms=%u over 25 ms=%u"
            " over 50 ms=%u longest_ms=%.1f\n", who, sp_flush.n, sp_flush.over12,
            sp_flush.over25, sp_flush.over50, sp_flush.max / 1000.0);
    sp_flush.n = sp_flush.over12 = sp_flush.over25 = sp_flush.over50 = 0;
    sp_flush.max = 0;
}

static void sp_fq_push(int64_t t0)
{
    if (sp.fq_len < G_N_ELEMENTS(sp.fq)) {
        sp.fq[(sp.fq_head + sp.fq_len++) % G_N_ELEMENTS(sp.fq)] = t0;
    }
}

static void sp_fq_pop(void)
{
    if (sp.fq_len) {
        sp.fq_head = (sp.fq_head + 1) % G_N_ELEMENTS(sp.fq);
        sp.fq_len--;
    }
}

/*
 * May a fenced command be answered here, before the sink has run it?
 *
 * Not while an older fenced command waits for data from the sink (see below).
 * Otherwise yes for the first fence_window commands the sink has not answered.
 * Beyond that only while the sink is keeping up: the oldest unanswered command
 * was sent no longer ago than two shortest round trips plus fence_age. A count
 * alone stops a guest that issues many small fenced commands per frame (video
 * decode does) although the sink is only one round trip behind; an age bounds
 * what matters, how far the guest runs ahead of the display.
 */
static bool sp_source_may_answer_early(void)
{
    unsigned out = sp.fence_out + sp.wait_out;
    int64_t rtt_min = MIN(sp.rtt_min_cur, sp.rtt_min_prev);

    if (sp.hard_out) {
        return false;
    }
    if (out < sp.fence_window) {
        return true;
    }
    if (!sp.fence_age_us || !sp.fq_len || sp.fq_len >= G_N_ELEMENTS(sp.fq) / 2 ||
        rtt_min == INT64_MAX) {
        return false;
    }
    if (g_get_monotonic_time() - sp.fq[sp.fq_head] < 2 * rtt_min + sp.fence_age_us) {
        sp.n_ahead++;
        return true;
    }
    return false;
}

static void sp_source_answer(VirtIOGPU *g, struct virtio_gpu_ctrl_command *cmd,
                             const void *resp, size_t len)
{
    size_t s = iov_from_buf(cmd->elem.in_sg, cmd->elem.in_num, 0, resp, len);

    virtqueue_push(cmd->vq, &cmd->elem, s);
    virtio_notify(VIRTIO_DEVICE(g), cmd->vq);
    g_free(cmd);
}

/*
 * Capability and display queries have a 5 second limit in the guest kernel. On
 * a busy link the answer can arrive later than that, and then the guest's
 * desktop silently falls back to software rendering. Answer them here from
 * what the sink sent ahead. Returns false if this query must be forwarded.
 */
static bool sp_source_query(VirtIOGPU *g, struct virtio_gpu_ctrl_command *cmd,
                            const uint8_t *req, size_t req_len, uint32_t type)
{
    unsigned i;

    switch (type) {
    case VIRTIO_GPU_CMD_GET_CAPSET_INFO: {
        struct virtio_gpu_get_capset_info q;
        struct virtio_gpu_resp_capset_info r = { 0 };
        uint32_t id;

        if (req_len < sizeof(q)) {
            return false;
        }
        memcpy(&q, req, sizeof(q));
        if (q.capset_index >= g->capset_ids->len) {
            return false;
        }
        id = g_array_index(g->capset_ids, uint32_t, q.capset_index);
        for (i = 0; i < G_N_ELEMENTS(sp.caps); i++) {
            if (sp.caps[i].data && sp.caps[i].id == id) {
                r.hdr.type = VIRTIO_GPU_RESP_OK_CAPSET_INFO;
                r.capset_id = id;
                r.capset_max_version = sp.caps[i].ver;
                r.capset_max_size = sp.caps[i].size;
                sp_source_answer(g, cmd, &r, sizeof(r));
                return true;
            }
        }
        return false;
    }
    case VIRTIO_GPU_CMD_GET_CAPSET: {
        struct virtio_gpu_get_capset q;

        if (req_len < sizeof(q)) {
            return false;
        }
        memcpy(&q, req, sizeof(q));
        for (i = 0; i < G_N_ELEMENTS(sp.caps); i++) {
            /* The renderer fills a virgl capset the same way for every version
             * (Mesa asks for version 0), so the id alone selects the data. */
            if (sp.caps[i].data && sp.caps[i].id == q.capset_id &&
                q.capset_version <= sp.caps[i].ver) {
                size_t len = sizeof(struct virtio_gpu_resp_capset) + sp.caps[i].size;
                g_autofree struct virtio_gpu_resp_capset *r = g_malloc0(len);

                r->hdr.type = VIRTIO_GPU_RESP_OK_CAPSET;
                memcpy(r->capset_data, sp.caps[i].data, sp.caps[i].size);
                sp_source_answer(g, cmd, r, len);
                return true;
            }
        }
        return false;
    }
    case VIRTIO_GPU_CMD_GET_DISPLAY_INFO:
        if (sp.dinfo) {
            sp_source_answer(g, cmd, sp.dinfo->data, sp.dinfo->len);
            return true;
        }
        return false;
    case VIRTIO_GPU_CMD_GET_EDID: {
        struct virtio_gpu_cmd_get_edid q;

        if (req_len < sizeof(q) || !sp.edid) {
            return false;
        }
        memcpy(&q, req, sizeof(q));
        if (q.scanout != 0) {
            return false;
        }
        sp_source_answer(g, cmd, sp.edid->data, sp.edid->len);
        return true;
    }
    default:
        return false;
    }
}

static void sp_source_info(SpHdr *h, uint8_t *a)
{
    unsigned i, slot = G_N_ELEMENTS(sp.caps);

    switch (h->arg0) {
    case 1:
        for (i = 0; i < G_N_ELEMENTS(sp.caps); i++) {
            if (sp.caps[i].data && sp.caps[i].id == h->arg1) {
                slot = i;
                break;
            }
            if (!sp.caps[i].data && slot == G_N_ELEMENTS(sp.caps)) {
                slot = i;
            }
        }
        if (slot < G_N_ELEMENTS(sp.caps) && h->a_len == (uint32_t)h->arg2) {
            g_free(sp.caps[slot].data);
            sp.caps[slot].id = h->arg1;
            sp.caps[slot].ver = h->arg2 >> 32;
            sp.caps[slot].size = h->a_len;
            sp.caps[slot].data = g_memdup2(a, h->a_len);
        }
        break;
    case 2:
    case 3: {
        GByteArray **dst = h->arg0 == 2 ? &sp.dinfo : &sp.edid;

        if (*dst) {
            g_byte_array_unref(*dst);
        }
        *dst = g_byte_array_new();
        g_byte_array_append(*dst, a, h->a_len);
        break;
    }
    default:
        break;
    }
}

static void sp_source_one(VirtIOGPU *g, struct virtio_gpu_ctrl_command *cmd)
{
    size_t req_len = iov_size(cmd->elem.out_sg, cmd->elem.out_num);
    size_t resp_cap = iov_size(cmd->elem.in_sg, cmd->elem.in_num);
    g_autofree uint8_t *req = g_malloc(MAX(req_len, 1));
    g_autofree uint8_t *extra = NULL;
    struct virtio_gpu_simple_resource *res;
    struct virtio_gpu_ctrl_hdr hdr = { 0 };
    SpPending *p;
    uint64_t off = 0, len = 0;
    uint64_t seq;
    bool needs_sink = false; /* the guest needs data or timing from the sink */
    uint32_t t3d_res;

    iov_to_buf(cmd->elem.out_sg, cmd->elem.out_num, 0, req, req_len);
    memcpy(&hdr, req, MIN(req_len, sizeof(hdr)));
    cmd->cmd_hdr = hdr;
    if (hdr.type == VIRTIO_GPU_CMD_RESOURCE_FLUSH) {
        sp_flush_seen();
    }
    if (sp_source_query(g, cmd, req, req_len, hdr.type)) {
        return;
    }
    p = g_new0(SpPending, 1);
    seq = ++sp.seq;
    p->cmd = cmd;
    p->type = hdr.type;
    p->seq = seq;
    p->t0 = g_get_monotonic_time();

#define SP_REQ(var) (req_len >= sizeof(var) ? (memcpy(&(var), req, sizeof(var)), true) : false)
    switch (hdr.type) {
    case VIRTIO_GPU_CMD_CTX_CREATE: {
        struct virtio_gpu_ctx_create c;
        if (SP_REQ(c) && hdr.ctx_id < 64) {
            g_strlcpy(sp.ctx_name[hdr.ctx_id], c.debug_name,
                      sizeof(sp.ctx_name[0]));
        }
        break;
    }
    case VIRTIO_GPU_CMD_RESOURCE_CREATE_2D: {
        struct virtio_gpu_resource_create_2d c;
        if (SP_REQ(c)) {
            sp_source_res_create(g, c.resource_id, c.format, c.width, c.height);
        }
        break;
    }
    case VIRTIO_GPU_CMD_RESOURCE_CREATE_3D: {
        struct virtio_gpu_resource_create_3d c;
        if (SP_REQ(c)) {
            sp_source_res_create(g, c.resource_id, c.format, c.width, c.height);
            if (c.target == 0 /* PIPE_BUFFER */ &&
                (c.bind & (1 << 17)) /* VIRGL_BIND_CUSTOM */) {
                g_hash_table_add(sp.custom_bufs, GUINT_TO_POINTER(c.resource_id));
            } else {
                g_hash_table_remove(sp.custom_bufs, GUINT_TO_POINTER(c.resource_id));
            }
        }
        break;
    }
    case VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING: {
        struct virtio_gpu_resource_attach_backing a;
        if (SP_REQ(a) && (res = virtio_gpu_find_resource(g, a.resource_id)) &&
            !res->iov) {
            if (virtio_gpu_create_mapping_iov(g, a.nr_entries, sizeof(a), cmd,
                                              NULL, &res->iov, &res->iov_cnt)) {
                res->iov = NULL;
                res->iov_cnt = 0;
            }
        }
        break;
    }
    case VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING: {
        struct virtio_gpu_resource_detach_backing d;
        if (SP_REQ(d) && (res = virtio_gpu_find_resource(g, d.resource_id))) {
            sp_source_res_unmap(g, res);
        }
        break;
    }
    case VIRTIO_GPU_CMD_RESOURCE_UNREF: {
        struct virtio_gpu_resource_unref u;
        if (SP_REQ(u) && (res = virtio_gpu_find_resource(g, u.resource_id))) {
            g_hash_table_remove(sp.query_bufs, GUINT_TO_POINTER(u.resource_id));
            g_hash_table_remove(sp.custom_bufs, GUINT_TO_POINTER(u.resource_id));
            sp_source_resource_destroy(g, res, NULL);
        }
        break;
    }
    case VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D: {
        struct virtio_gpu_transfer_to_host_2d t;
        if (SP_REQ(t) && (res = virtio_gpu_find_resource(g, t.resource_id)) &&
            res->iov) {
            struct virtio_gpu_box box = {
                .x = t.r.x, .y = t.r.y, .w = t.r.width, .h = t.r.height, .d = 1,
            };
            sp_range(res->format, res->width, res->height, 0, 0, 0, &box,
                     t.offset, iov_size(res->iov, res->iov_cnt), &off, &len);
        }
        break;
    }
    case VIRTIO_GPU_CMD_TRANSFER_TO_HOST_3D:
    case VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D: {
        struct virtio_gpu_transfer_host_3d t;
        if (SP_REQ(t) && (res = virtio_gpu_find_resource(g, t.resource_id)) &&
            res->iov) {
            sp_range(res->format, res->width, res->height, t.level, t.stride,
                     t.layer_stride, &t.box, t.offset,
                     iov_size(res->iov, res->iov_cnt), &off, &len);
            p->res = t.resource_id;
            if (hdr.type == VIRTIO_GPU_CMD_TRANSFER_TO_HOST_3D &&
                sp_source_defer(t.resource_id, len)) {
                len = 0;
            }
        }
        break;
    }
    default:
        break;
    }
#undef SP_REQ

    if (len && hdr.type != VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D) {
        res = NULL;
        extra = g_malloc(len);
        /* res was validated in the switch above */
        if (hdr.type == VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D) {
            struct virtio_gpu_transfer_to_host_2d t;
            memcpy(&t, req, sizeof(t));
            res = virtio_gpu_find_resource(g, t.resource_id);
            sp_source_hint(res, 0, 0, true);
        } else {
            struct virtio_gpu_transfer_host_3d t;
            memcpy(&t, req, sizeof(t));
            res = virtio_gpu_find_resource(g, t.resource_id);
            sp_source_hint(res, t.level, t.stride, true);
        }
        iov_to_buf(res->iov, res->iov_cnt, off, extra, len);
        sp.n_xfer++;
        sp.xfer_bytes += len;
    }
    if (hdr.type == VIRTIO_GPU_CMD_SUBMIT_3D) {
        struct virtio_gpu_cmd_submit cs;

        sp.n_submit++;
        sp.submit_bytes += req_len;
        if (req_len >= sizeof(cs)) {
            g_autoptr(GArray) expect = g_array_new(false, false, sizeof(SpExpect));

            memcpy(&cs, req, sizeof(cs));
            needs_sink = sp_source_scan_submit(g, hdr.ctx_id, req + sizeof(cs),
                                  MIN((size_t)cs.size, req_len - sizeof(cs)),
                                  expect);
            if (expect->len) {
                sp_send(SP_EXPECT, 0, expect->data,
                        expect->len * sizeof(SpExpect), NULL, 0, 0, 0, 0);
            }
        }
    }
    sp.n_ctrl++;

    p->off = off;
    p->len = len;

    t3d_res = p->res;
    if (hdr.type == VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D && p->res && !sp.hard_out &&
        g_hash_table_contains(sp.query_bufs, GUINT_TO_POINTER(p->res))) {
        /* The guest's copy of a query result buffer is kept current by the
         * sink's watch, so there is nothing to fetch. */
        struct virtio_gpu_ctrl_hdr r = { .type = VIRTIO_GPU_RESP_OK_NODATA };
        uint32_t state = 0;

        sp.n_local++;
        sp.seq--;
        if (hdr.ctx_id < 64) {
            sp.ctx_qpoll[hdr.ctx_id]++;
        }
        g_free(p);
        res = virtio_gpu_find_resource(g, t3d_res);
        if (res && res->iov) {
            iov_to_buf(res->iov, res->iov_cnt, 0, &state, sizeof(state));
        }
        if (state == 2 /* VIRGL_QUERY_STATE_WAIT_HOST */) {
            /* Not in yet. Hold the read until the sink reports the result (or
             * 200 ms pass), so a guest that loops on it does not spin. */
            SpParked *k = g_new0(SpParked, 1);

            k->cmd = cmd;
            k->hdr = hdr;
            k->res = t3d_res;
            k->t0 = g_get_monotonic_time();
            g_queue_push_tail(&sp.parked, k);
            return;
        }
        if (hdr.flags & VIRTIO_GPU_FLAG_FENCE) {
            r.flags = VIRTIO_GPU_FLAG_FENCE;
            r.fence_id = hdr.fence_id;
            r.ctx_id = hdr.ctx_id;
        }
        sp_source_answer(g, cmd, &r, sizeof(r));
        return;
    }

    switch (hdr.type) {
    case VIRTIO_GPU_CMD_GET_CAPSET_INFO:
    case VIRTIO_GPU_CMD_GET_CAPSET:
    case VIRTIO_GPU_CMD_GET_DISPLAY_INFO:
    case VIRTIO_GPU_CMD_GET_EDID:
    case VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D:
    case VIRTIO_GPU_CMD_RESOURCE_CREATE_BLOB:
    case VIRTIO_GPU_CMD_RESOURCE_MAP_BLOB:
    case VIRTIO_GPU_CMD_RESOURCE_UNMAP_BLOB:
        needs_sink = true;
        break;
    default:
        break;
    }

    /*
     * Most commands return nothing but "done". Waiting for the sink to say so
     * costs a network round trip, and for fenced commands that round trip is
     * what paces the guest's frames. Answer here instead. Commands still reach
     * the sink in order, so rendering is unchanged; only the moment the guest
     * hears "done" moves. Fenced answers are limited to fence_window ahead of
     * the sink, so the guest cannot run arbitrarily far ahead of the display.
     */
    if (!needs_sink && sp.fence_window) {
        struct virtio_gpu_ctrl_hdr r = { .type = VIRTIO_GPU_RESP_OK_NODATA };
        bool fenced = hdr.flags & VIRTIO_GPU_FLAG_FENCE;

        if (!fenced) {
            sp.n_local++;
            sp_send(SP_CTRL, 0, req, req_len, extra, extra ? len : 0,
                    resp_cap, off, len);
            sp_source_answer(g, cmd, &r, sizeof(r));
            g_free(p);
            return;
        }
        /*
         * The guest kernel treats a signalled fence as signalling every older
         * fence too. While a fenced command is still waiting for data from
         * the sink (a read back), a later fence must not be answered first,
         * or the guest reads its memory before the data is there. Commands
         * that wait only because of the window do not hold later ones back.
         */
        if (sp_source_may_answer_early()) {
            sp_fq_push(p->t0);
            r.flags = VIRTIO_GPU_FLAG_FENCE;
            r.fence_id = hdr.fence_id;
            r.ctx_id = hdr.ctx_id;
            sp.n_early++;
            sp.fence_out++;
            p->early = true;
            p->cmd = NULL;
            g_hash_table_insert(sp.pending, g_memdup2(&seq, sizeof(seq)), p);
            sp_send(SP_CTRL, seq, req, req_len, extra, extra ? len : 0,
                    resp_cap, off, len);
            sp_source_answer(g, cmd, &r, sizeof(r));
            return;
        }
    }
    sp.n_waited++;
    if (getenv("SPACEBOX_GPU_REMOTE_DEBUG_WAIT")) {
        static unsigned shown;
        struct virtio_gpu_transfer_host_3d t = { 0 };

        if (req_len >= sizeof(t)) {
            memcpy(&t, req, sizeof(t));
        }
        if (shown++ % 200 < 30) {
            res = hdr.type == VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D ?
                  virtio_gpu_find_resource(g, t.resource_id) : NULL;
            fprintf(stderr, "[SPACEBOX-WAIT] type=0x%x fenced=%d ctx=%u len=%zu"
                    " from_host_res=%u fmt=%u %ux%u box=%u,%u+%ux%u bytes=%" PRIu64
                    " fence_out=%u wait_out=%u hard_out=%u needs_sink=%d\n", hdr.type,
                    !!(hdr.flags & VIRTIO_GPU_FLAG_FENCE), hdr.ctx_id, req_len,
                    res ? t.resource_id : 0, res ? res->format : 0,
                    res ? res->width : 0, res ? res->height : 0,
                    t.box.x, t.box.y, t.box.w, t.box.h, len, sp.fence_out,
                    sp.wait_out, sp.hard_out, needs_sink);
        }
    }
    if (hdr.flags & VIRTIO_GPU_FLAG_FENCE) {
        sp_fq_push(p->t0);
        p->fence_wait = true;
        sp.wait_out++;
        if (needs_sink) {
            p->hard = true;
            sp.hard_out++;
        }
    }
    g_hash_table_insert(sp.pending, g_memdup2(&seq, sizeof(seq)), p);
    sp_send(SP_CTRL, seq, req, req_len, extra, extra ? len : 0,
            resp_cap, off, len);
}

void sp_source_handle_ctrl(VirtIOGPU *g, VirtQueue *vq)
{
    struct virtio_gpu_ctrl_command *cmd;

    if (!virtio_queue_ready(vq)) {
        return;
    }
    if (sp.fd < 0 || !sp.hello_done || sp.broken) {
        return; /* resumed by sp_hello() */
    }
    for (;;) {
        if (sp_tx_backlog() > sp.tx_limit) {
            /* Let the network drain before taking more work from the guest. */
            timer_mod(sp.retry, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 1);
            return;
        }
        cmd = virtqueue_pop(vq, sizeof(struct virtio_gpu_ctrl_command));
        if (!cmd) {
            return;
        }
        cmd->vq = vq;
        cmd->error = 0;
        cmd->finished = false;
        cmd->suspended = false;
        sp_source_one(g, cmd);
    }
}

void sp_source_cursor(VirtIOGPU *g, struct virtio_gpu_update_cursor *c)
{
    sp_send(SP_CURSOR, 0, c, sizeof(*c), NULL, 0, 0, 0, 0);
}

static gboolean sp_pending_drop(gpointer key, gpointer value, gpointer opaque)
{
    SpPending *p = value;

    g_free(p->cmd);
    return TRUE;
}

void sp_source_reset(VirtIOGPU *g)
{
    if (sp.pending) {
        g_hash_table_foreach_remove(sp.pending, sp_pending_drop, NULL);
    }
    sp.fence_out = 0;
    sp.wait_out = 0;
    sp.hard_out = 0;
    sp.fq_head = sp.fq_len = 0;
    if (sp.custom_bufs) {
        g_hash_table_remove_all(sp.custom_bufs);
    }
    while (!g_queue_is_empty(&sp.parked)) {
        SpParked *k = g_queue_pop_head(&sp.parked);
        g_free(k->cmd);
        g_free(k);
    }
    sp_send(SP_RESET, 0, NULL, 0, NULL, 0, 0, 0, 0);
}

static void sp_source_resp(VirtIOGPU *g, SpHdr *h, uint8_t *a, uint8_t *b)
{
    uint64_t seq = h->seq;
    SpPending *p = g_hash_table_lookup(sp.pending, &seq);
    struct virtio_gpu_ctrl_command *cmd;
    size_t s;

    if (!p) {
        return; /* dropped by a reset */
    }
    sp.rtt_us = g_get_monotonic_time() - p->t0;
    if (p->t0 + sp.rtt_us - sp.rtt_min_t0 > 5 * G_USEC_PER_SEC) {
        sp.rtt_min_prev = sp.rtt_min_cur;
        sp.rtt_min_cur = INT64_MAX;
        sp.rtt_min_t0 = p->t0 + sp.rtt_us;
    }
    sp.rtt_min_cur = MIN(sp.rtt_min_cur, sp.rtt_us);
    sp_delay.n++;
    sp_delay.over12 += sp.rtt_us > 12000;
    sp_delay.over25 += sp.rtt_us > 25000;
    sp_delay.over50 += sp.rtt_us > 50000;
    sp_delay.max = MAX(sp_delay.max, sp.rtt_us);
    if (p->early || p->fence_wait) {
        sp_fq_pop();
    }
    if (p->early) {
        if (sp.fence_out) {
            sp.fence_out--;
        }
        g_hash_table_remove(sp.pending, &seq);
        return;
    }
    if (p->fence_wait && sp.wait_out) {
        sp.wait_out--;
    }
    if (p->hard && sp.hard_out) {
        sp.hard_out--;
    }
    cmd = p->cmd;
    if (p->type == VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D && h->b_len) {
        struct virtio_gpu_simple_resource *res =
            virtio_gpu_find_resource(g, p->res);

        if (res && res->iov) {
            iov_from_buf(res->iov, res->iov_cnt, p->off, b,
                         MIN((uint64_t)h->b_len, p->len));
        }
    }
    s = iov_from_buf(cmd->elem.in_sg, cmd->elem.in_num, 0, a, h->a_len);
    virtqueue_push(cmd->vq, &cmd->elem, s);
    virtio_notify(VIRTIO_DEVICE(g), cmd->vq);
    g_free(cmd);
    g_hash_table_remove(sp.pending, &seq);
}

/* Answer parked reads of resource res (0 = all older than max_age_us). */
static void sp_source_unpark(VirtIOGPU *g, uint32_t res, int64_t max_age_us)
{
    int64_t now = g_get_monotonic_time();
    GList *l = sp.parked.head;

    while (l) {
        SpParked *k = l->data;
        GList *next = l->next;

        if ((res && k->res == res) || (!res && now - k->t0 > max_age_us)) {
            struct virtio_gpu_ctrl_hdr r = { .type = VIRTIO_GPU_RESP_OK_NODATA };

            if (k->hdr.flags & VIRTIO_GPU_FLAG_FENCE) {
                r.flags = VIRTIO_GPU_FLAG_FENCE;
                r.fence_id = k->hdr.fence_id;
                r.ctx_id = k->hdr.ctx_id;
            }
            sp_source_answer(g, k->cmd, &r, sizeof(r));
            g_queue_delete_link(&sp.parked, l);
            g_free(k);
        }
        l = next;
    }
}

static void sp_source_memwrite(VirtIOGPU *g, SpHdr *h, uint8_t *a)
{
    struct virtio_gpu_simple_resource *res = virtio_gpu_find_resource(g, h->arg0);

    if (res && res->iov) {
        iov_from_buf(res->iov, res->iov_cnt, h->arg1, a, h->a_len);
        sp.n_back++;
        sp.back_bytes += h->a_len;
    }
    sp_source_unpark(g, h->arg0, 0);
}

static void sp_source_input(SpHdr *h, uint8_t *a)
{
    SpInput m;

    if (h->a_len < sizeof(m)) {
        return;
    }
    memcpy(&m, a, sizeof(m));
    switch (m.kind) {
    case 0:
        qemu_input_event_sync();
        break;
    case 1:
        qemu_input_event_send_key_qcode(NULL, m.a, m.b);
        break;
    case 2:
        qemu_input_queue_btn(NULL, m.a, m.b);
        break;
    case 3:
        qemu_input_queue_rel(NULL, m.a, m.b);
        break;
    case 4: {
        InputMoveEvent move = { .axis = m.a, .value = m.b };
        InputEvent evt = { .type = INPUT_EVENT_KIND_ABS, .u.abs.data = &move };

        qemu_input_event_send(NULL, &evt);
        break;
    }
    default:
        break;
    }
}

static void sp_source_connect(void *opaque)
{
    VirtIOGPU *g = sp.g;
    struct sockaddr_un addr = { .sun_family = AF_UNIX };
    int fd;

    if (sp.fd >= 0) {
        /* backlog retry */
        sp_source_handle_ctrl(g, g->ctrl_vq);
        return;
    }
    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    g_strlcpy(addr.sun_path, sp.path, sizeof(addr.sun_path));
    if (fd < 0 || connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        if (fd >= 0) {
            close(fd);
        }
        timer_mod(sp.retry, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 250);
        return;
    }
    info_report("spacebox remote gpu: source connected to %s", sp.path);
    sp_set_connected(fd);
}

/* ---- sink --------------------------------------------------------------- */

int sp_sink_create_mapping(VirtIOGPU *g, uint32_t nr_entries, uint32_t offset,
                           struct virtio_gpu_ctrl_command *cmd,
                           struct iovec **iov, uint32_t *niov)
{
    struct virtio_gpu_resource_attach_backing att;
    g_autofree struct virtio_gpu_mem_entry *ents = NULL;
    size_t esize = sizeof(*ents) * (size_t)nr_entries, total = 0;
    SpShadow *sh;
    uint8_t *ptr;
    uint32_t e;

    *iov = NULL;
    *niov = 0;
    if (!cmd->sp_remote || nr_entries > 16384 || offset != sizeof(att) ||
        iov_to_buf(cmd->elem.out_sg, cmd->elem.out_num, 0, &att,
                   sizeof(att)) != sizeof(att)) {
        return -1;
    }
    ents = g_malloc(MAX(esize, 1));
    if (iov_to_buf(cmd->elem.out_sg, cmd->elem.out_num, offset, ents,
                   esize) != esize) {
        return -1;
    }
    for (e = 0; e < nr_entries; e++) {
        total += le32_to_cpu(ents[e].length);
    }
    ptr = g_try_malloc0(MAX(total, 1));
    if (!ptr) {
        return -1;
    }
    *iov = g_new0(struct iovec, 1);
    (*iov)[0].iov_base = ptr;
    (*iov)[0].iov_len = total;
    *niov = 1;

    if (!g_hash_table_lookup(sp.shadow, GUINT_TO_POINTER(att.resource_id))) {
        sh = g_new0(SpShadow, 1);
        sh->ptr = ptr;
        sh->size = total;
        g_hash_table_insert(sp.shadow, GUINT_TO_POINTER(att.resource_id), sh);
    }
    return 0;
}

static gboolean sp_shadow_is(gpointer key, gpointer value, gpointer ptr)
{
    return ((SpShadow *)value)->ptr == ptr;
}

void sp_sink_cleanup_mapping(struct iovec *iov, uint32_t count)
{
    uint32_t i;

    for (i = 0; i < count; i++) {
        g_hash_table_foreach_remove(sp.shadow, sp_shadow_is, iov[i].iov_base);
        g_free(iov[i].iov_base);
    }
    g_free(iov);
}

static void sp_sink_memwrite(SpHdr *h, uint8_t *a)
{
    SpShadow *sh = g_hash_table_lookup(sp.shadow, GUINT_TO_POINTER(h->arg0));

    if (!sh || h->arg1 >= sh->size || h->a_len > sh->size - h->arg1) {
        return;
    }
    memcpy(sh->ptr + h->arg1, a, h->a_len);
    sp.n_mem++;
    sp.mem_bytes += h->a_len;
    if (h->arg2 == 1 && h->arg1 == 0 && sh->size >= 16) {
        SpWatch *w = g_hash_table_lookup(sp.watch, GUINT_TO_POINTER(h->arg0));

        if (!w) {
            w = g_new0(SpWatch, 1);
            g_hash_table_insert(sp.watch, GUINT_TO_POINTER(h->arg0), w);
        }
        memcpy(w->last, sh->ptr, 16);
    }
}

/* Report renderer writes to watched query-result buffers. */
void sp_sink_poll_watches(void)
{
    GHashTableIter it;
    gpointer key, value;

    if (!sp.watch || sp.fd < 0) {
        return;
    }
    g_hash_table_iter_init(&it, sp.watch);
    while (g_hash_table_iter_next(&it, &key, &value)) {
        SpShadow *sh = g_hash_table_lookup(sp.shadow, key);
        SpWatch *w = value;

        if (!sh || sh->size < 16) {
            g_hash_table_iter_remove(&it);
        } else if (memcmp(w->last, sh->ptr, 16)) {
            memcpy(w->last, sh->ptr, 16);
            sp.n_back++;
            sp.back_bytes += 16;
            sp_send(SP_MEMWRITE, 0, sh->ptr, 16, NULL, 0,
                    GPOINTER_TO_UINT(key), 0, 0);
        }
    }
}

/* After a command stream ran: return what it made the renderer write. */
void sp_sink_after_submit(struct virtio_gpu_ctrl_command *cmd)
{
    GByteArray *list = cmd->sp_expect;
    size_t i;

    cmd->sp_expect = NULL;
    for (i = 0; list && i + sizeof(SpExpect) <= list->len; i += sizeof(SpExpect)) {
        SpExpect e;
        SpShadow *sh;

        memcpy(&e, list->data + i, sizeof(e));
        sh = g_hash_table_lookup(sp.shadow, GUINT_TO_POINTER(e.res));
        if (sh && e.off < sh->size && e.len <= sh->size - e.off) {
            sp.n_back++;
            sp.back_bytes += e.len;
            sp_send(SP_MEMWRITE, 0, sh->ptr + e.off, e.len, NULL, 0,
                    e.res, e.off, 0);
        }
    }
    if (list) {
        g_byte_array_unref(list);
    }
    sp_sink_poll_watches();
}

void sp_sink_snapshot_readback(struct virtio_gpu_ctrl_command *cmd)
{
    SpShadow *sh = g_hash_table_lookup(sp.shadow, GUINT_TO_POINTER(cmd->sp_res));

    if (sh && cmd->sp_len && cmd->sp_off < sh->size &&
        cmd->sp_len <= sh->size - cmd->sp_off) {
        cmd->sp_snap = g_memdup2(sh->ptr + cmd->sp_off, cmd->sp_len);
    }
}

void sp_sink_response(VirtIOGPU *g, struct virtio_gpu_ctrl_command *cmd,
                      struct virtio_gpu_ctrl_hdr *resp, size_t resp_len)
{
    bool ok = resp->type == VIRTIO_GPU_RESP_OK_NODATA && cmd->sp_snap;

    sp_sink_poll_watches(); /* results first, then the completion */
    if (!cmd->sp_seq) {
        /* The source already answered the guest and wants nothing back. */
        if (resp->type >= VIRTIO_GPU_RESP_ERR_UNSPEC) {
            fprintf(stderr, "[SPACEBOX-REMOTE] sink: command 0x%x failed with 0x%x "
                    "after the guest was told it succeeded\n",
                    cmd->cmd_hdr.type, resp->type);
        }
        g_free(cmd->sp_snap);
        cmd->sp_snap = NULL;
        return;
    }
    sp_send(SP_RESP, cmd->sp_seq, resp, resp_len,
            ok ? cmd->sp_snap : NULL, ok ? cmd->sp_len : 0,
            0, cmd->sp_off, 0);
    g_free(cmd->sp_snap);
    cmd->sp_snap = NULL;
}

static bool sp_sink_renderer(VirtIOGPU *g);

/* Send the answers to the guest's capability and display queries ahead. */
static void sp_sink_send_info(VirtIOGPU *g, bool caps)
{
    struct virtio_gpu_resp_display_info di = { 0 };
    struct virtio_gpu_resp_edid ed = { 0 };
    unsigned i;

    if (sp.fd < 0) {
        return;
    }
    if (caps && sp_sink_renderer(g)) {
        for (i = 0; i < g->capset_ids->len; i++) {
            uint32_t id = g_array_index(g->capset_ids, uint32_t, i);
            uint32_t ver = 0, size = 0;
            g_autofree uint8_t *buf = NULL;

            virgl_renderer_get_cap_set(id, &ver, &size);
            if (!size) {
                continue;
            }
            buf = g_malloc0(size);
            virgl_renderer_fill_caps(id, ver, buf);
            /* struct virgl_caps_v1 .bset.timer_query: byte 261, bit 4 (checked
             * against virgl_hw.h of the pinned virglrenderer). */
            if (size > 261 && !getenv("SPACEBOX_GPU_REMOTE_KEEP_TIMER_QUERY")) {
                buf[261] &= ~0x10;
            }
            sp_send(SP_INFO, 0, buf, size, NULL, 0, 1, id,
                    ((uint64_t)ver << 32) | size);
        }
    }
    di.hdr.type = VIRTIO_GPU_RESP_OK_DISPLAY_INFO;
    virtio_gpu_base_fill_display_info(VIRTIO_GPU_BASE(g), &di);
    sp_send(SP_INFO, 0, &di, sizeof(di), NULL, 0, 2, 0, 0);
    ed.hdr.type = VIRTIO_GPU_RESP_OK_EDID;
    virtio_gpu_base_generate_edid(VIRTIO_GPU_BASE(g), 0, &ed);
    sp_send(SP_INFO, 0, &ed, sizeof(ed), NULL, 0, 3, 0, 0);
}

void sp_sink_notify_event(uint32_t event_type)
{
    if (sp.g) {
        sp_sink_send_info(sp.g, false);
    }
    sp_send(SP_DISPLAY, 0, NULL, 0, NULL, 0, event_type, 0, 0);
}

static bool sp_sink_renderer(VirtIOGPU *g)
{
    VirtIOGPUGL *gl = VIRTIO_GPU_GL(g);

    switch (gl->renderer_state) {
    case RS_RESET:
        virtio_gpu_virgl_reset(g);
        /* fallthrough */
    case RS_START:
        if (virtio_gpu_virgl_init(g)) {
            gl->renderer_state = RS_INIT_FAILED;
            return false;
        }
        gl->renderer_state = RS_INITED;
        return true;
    case RS_INIT_FAILED:
        return false;
    case RS_INITED:
        return true;
    }
    return false;
}

static void sp_sink_ctrl(VirtIOGPU *g, SpHdr *h, uint8_t *a, uint8_t *b)
{
    size_t resp_cap = MIN(h->arg0, SP_RESP_CAP_MAX);
    struct virtio_gpu_ctrl_command *cmd;
    struct virtio_gpu_ctrl_hdr hdr = { 0 };
    struct iovec *iov;
    uint8_t *req;
    uint32_t res_id = 0;

    if (!sp_sink_renderer(g)) {
        return;
    }
    cmd = g_malloc0(sizeof(*cmd) + 2 * sizeof(*iov) + h->a_len + resp_cap);
    iov = (struct iovec *)(cmd + 1);
    req = (uint8_t *)(iov + 2);
    memcpy(req, a, h->a_len);
    iov[0].iov_base = req;
    iov[0].iov_len = h->a_len;
    iov[1].iov_base = req + h->a_len;
    iov[1].iov_len = resp_cap;
    cmd->elem.out_sg = &iov[0];
    cmd->elem.out_num = 1;
    cmd->elem.in_sg = &iov[1];
    cmd->elem.in_num = 1;
    cmd->sp_remote = true;
    cmd->sp_seq = h->seq;
    cmd->sp_off = h->arg1;
    cmd->sp_len = h->arg2;
    cmd->sp_expect = sp.next_expect;
    sp.next_expect = NULL;

    memcpy(&hdr, req, MIN((size_t)h->a_len, sizeof(hdr)));
    if (hdr.type == VIRTIO_GPU_CMD_RESOURCE_FLUSH) {
        sp_flush_seen();
        sp_sink_pace_frame(h);
    }
    if (hdr.type == VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D &&
        h->a_len >= sizeof(struct virtio_gpu_transfer_to_host_2d)) {
        res_id = ((struct virtio_gpu_transfer_to_host_2d *)req)->resource_id;
    } else if ((hdr.type == VIRTIO_GPU_CMD_TRANSFER_TO_HOST_3D ||
                hdr.type == VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D) &&
               h->a_len >= sizeof(struct virtio_gpu_transfer_host_3d)) {
        res_id = ((struct virtio_gpu_transfer_host_3d *)req)->resource_id;
    }
    cmd->sp_res = res_id;
    if (res_id && h->b_len) {
        SpShadow *sh = g_hash_table_lookup(sp.shadow, GUINT_TO_POINTER(res_id));

        if (sh && h->arg1 < sh->size && h->b_len <= sh->size - h->arg1) {
            memcpy(sh->ptr + h->arg1, b, h->b_len);
        }
        sp.n_xfer++;
        sp.xfer_bytes += h->b_len;
    }
    if (hdr.type == VIRTIO_GPU_CMD_SUBMIT_3D) {
        sp.n_submit++;
        sp.submit_bytes += h->a_len;
    }
    sp.n_ctrl++;

    QTAILQ_INSERT_TAIL(&g->cmdq, cmd, next);
    virtio_gpu_process_cmdq(g);
    virtio_gpu_virgl_fence_poll(g);
    sp_sink_poll_watches();
}

static void sp_sink_reset(VirtIOGPU *g)
{
    VirtioDeviceClass *vdc = VIRTIO_DEVICE_GET_CLASS(g);

    vdc->reset(VIRTIO_DEVICE(g));
}

static void sp_input_event(DeviceState *dev, QemuConsole *src, InputEvent *evt)
{
    SpInput m = { 0 };

    switch (evt->type) {
    case INPUT_EVENT_KIND_KEY:
        m.kind = 1;
        m.a = qemu_input_key_value_to_qcode(evt->u.key.data->key);
        m.b = evt->u.key.data->down;
        spacebox_last_input_us = g_get_monotonic_time();
        break;
    case INPUT_EVENT_KIND_BTN:
        m.kind = 2;
        m.a = evt->u.btn.data->button;
        m.b = evt->u.btn.data->down;
        spacebox_last_input_us = g_get_monotonic_time();
        break;
    case INPUT_EVENT_KIND_REL:
        m.kind = 3;
        m.a = evt->u.rel.data->axis;
        m.b = evt->u.rel.data->value;
        break;
    case INPUT_EVENT_KIND_ABS:
        m.kind = 4;
        m.a = evt->u.abs.data->axis;
        m.b = evt->u.abs.data->value;
        break;
    default:
        return;
    }
    if (sp.fd < 0 || !sp.hello_done) {
        return; /* not kept for later: old input would only confuse */
    }
    sp_send(SP_INPUT, 0, &m, sizeof(m), NULL, 0, 0, 0, 0);
}

static void sp_input_sync(DeviceState *dev)
{
    SpInput m = { 0 };

    if (sp.fd < 0 || !sp.hello_done) {
        return;
    }
    sp_send(SP_INPUT, 0, &m, sizeof(m), NULL, 0, 0, 0, 0);
}

static const QemuInputHandler sp_input_handler = {
    .name = "spacebox-remote-input",
    .mask = INPUT_EVENT_MASK_KEY | INPUT_EVENT_MASK_BTN |
            INPUT_EVENT_MASK_REL | INPUT_EVENT_MASK_ABS,
    .event = sp_input_event,
    .sync = sp_input_sync,
};

static void sp_sink_accept(void *opaque)
{
    int fd = accept(sp.listen_fd, NULL, NULL);

    if (fd < 0) {
        return;
    }
    if (sp.fd >= 0) {
        /* The newer connection wins; the old one may be dead without notice. */
        sp_disconnected();
    }
    sp_set_connected(fd);
}

/* ---- common ------------------------------------------------------------- */

static void sp_disconnected(void)
{
    int fd = sp.fd;

    if (fd < 0) {
        return;
    }
    qemu_set_fd_handler(fd, NULL, NULL, NULL);
    qemu_mutex_lock(&sp.tx_lock);
    sp.fd = -1;
    sp.hello_done = false;
    sp.tx_enabled = false;
    qemu_mutex_unlock(&sp.tx_lock);
    close(fd);
    g_byte_array_set_size(sp.rx, 0);
    g_byte_array_set_size(sp.rxw, 0);

    /* Both ends keep their state. The source reconnects and the stream
     * continues from the last message the other end received. */
    if (sp.mode == SP_REMOTE_SINK) {
        info_report("spacebox remote gpu: connection lost, waiting for the source");
    } else {
        info_report("spacebox remote gpu: connection lost, reconnecting");
        timer_mod(sp.retry, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 250);
    }
}

static void sp_sink_reset(VirtIOGPU *g);
static void sp_sink_send_info(VirtIOGPU *g, bool caps);

static void sp_hello(SpHdr *h)
{
    bool resume = sp.peer_session && h->arg0 == sp.peer_session &&
                  h->arg1 == sp.session;

    if (sp.hello_done) {
        return;
    }
    qemu_mutex_lock(&sp.tx_lock);
    sp.z_on = sp.z_level && (h->flags & SP_F_ZSTD);
    qemu_mutex_unlock(&sp.tx_lock);
    if (resume) {
        sp.n_reconnect++;
        info_report("spacebox remote gpu: resumed, peer has %" PRIu64 " of %"
                    PRIu64 " messages", (uint64_t)h->arg2, sp.tx_id);
    } else if (sp.mode == SP_REMOTE_SINK) {
        /* A source this renderer state does not belong to: start empty. */
        info_report("spacebox remote gpu: source attached");
        sp_tx_forget();
        sp_sink_reset(sp.g);
        sp.peer_session = h->arg0;
    } else if (sp.peer_session && sp.seq) {
        error_report("spacebox remote gpu: the sink has lost this guest's "
                     "renderer state (it restarted); the guest display cannot "
                     "continue until the VM restarts");
        sp.broken = true;
        sp.hello_done = true;
        return;
    } else {
        sp_tx_forget();
        sp.peer_session = h->arg0;
    }
    sp.hello_done = true;
    if (!resume && sp.mode == SP_REMOTE_SINK) {
        /* Tell the new source what this renderer and display look like. */
        sp_sink_send_info(sp.g, true);
        sp_send(SP_DISPLAY, 0, NULL, 0, NULL, 0, VIRTIO_GPU_EVENT_DISPLAY, 0, 0);
    }
    sp_tx_start(resume ? h->arg2 : 0);
    if (sp.mode == SP_REMOTE_SOURCE) {
        sp_source_handle_ctrl(sp.g, sp.g->ctrl_vq);
    }
}

/*
 * Sound. The source runs the guest's sound card on QEMU's "none" backend,
 * which takes the samples at playback speed; they are sent as they are. The
 * sink plays them through its own audio backend (Core Audio on the Mac).
 *
 * The sink keeps a little sound in hand so that messages arriving unevenly do
 * not leave gaps: it starts playing when sp_aud.target is buffered and from
 * then on hands the samples to the audio backend at playback speed (the
 * backend asks for as much as it has room for, which would empty the buffer
 * at once), a fixed lead ahead of it. The lead is what waits in the backend:
 * Core Audio plays nothing at all for a request that finds less than one
 * request's worth waiting (512 frames, 11.6 ms), so the lead is at least two
 * requests plus 15 ms (by default the target less 15 ms), and it grows by 5 ms
 * whenever a request still found too little while sound was playing. (It used to be 20 ms: about one request in ten came
 * up empty in some sessions and none in others, depending on how the hand-overs
 * happened to fall between the requests. The user heard it as rapid stutter.)
 * If the buffer runs empty it waits for the target again. A surplus
 * (after a hold-up the delayed sound arrives at once; the two machines' clocks
 * drift) is worked off by playing 1% faster; only when three times the target
 * has piled up is the oldest part dropped.
 *
 * The target starts at SPACEBOX_AUDIO_BUFFER_MS (default 80). Each time the
 * buffer runs empty while sound is still arriving, the target goes up by 20 ms,
 * to at most 40 ms above the start. After 30 s without that it comes down by
 * 10 ms at a time. (A larger range was tried: on a link with half-second
 * outages it only added delay, up to 230 ms, without preventing the gaps.)
 */
static struct {
    QEMUSoundCard card;
    SWVoiceOut *voice;
    int freq, channels;
    uint8_t *ring;
    size_t cap, head, len;  /* bytes */
    size_t target;
    int target_ms, base_ms;
    int64_t last_underrun, last_lowered;
    bool playing, failed;
    bool fast;              /* playing 1% faster to work off a surplus */
    double frac;            /* position between two input frames while fast */
    uint64_t fast_ms;
    int64_t t_start;        /* when playing began */
    uint64_t fed;           /* bytes handed to the backend since then */
    int64_t last_rx;
    int64_t held_avg;       /* running average of the buffered time, us */
    int64_t lead_us;        /* how far ahead of playback the backend is supplied */
    int64_t lead_raised;    /* when it last grew */
    uint64_t dev_starved_seen, dev_starved; /* requests that found too little */
    FILE *dump;
    uint64_t rx_bytes, played, dropped, underruns;
} sp_aud;

void spacebox_remote_audio_out(int freq, int channels, const void *buf, size_t len);
#ifdef __APPLE__
/* audio/coreaudio.m */
void spacebox_coreaudio_stats(uint64_t *requests, uint64_t *starved,
                              uint32_t *frames, uint32_t *least_waiting);
uint64_t spacebox_coreaudio_starved_total(void);
uint32_t spacebox_coreaudio_request_us(void);
#endif

#define SP_AUD_LEAD_OLD_US 20000 /* the lead the picture/sound offset was tuned with */

/* How far ahead of playback speed the backend is kept supplied. */
static int64_t sp_aud_lead(int64_t now)
{
    if (!sp_aud.lead_us) {
        const char *ms = getenv("SPACEBOX_AUDIO_LEAD_MS");
        int64_t request_us = 11610;

#ifdef __APPLE__
        if (spacebox_coreaudio_request_us()) {
            request_us = spacebox_coreaudio_request_us();
        }
#endif
        /*
         * Two requests plus 15 ms at least; by default all of the target but
         * 15 ms. The hand-over runs in the main loop, which also executes the
         * guest's GPU commands: while that takes 50 ms nothing is handed over
         * and only what waits in the backend keeps the sound going. Sound
         * that waits in the backend instead of in the buffer here comes out
         * no later.
         */
        sp_aud.lead_us = ms ? MAX(atoi(ms), 5) * 1000 :
            MAX(2 * request_us + 15000, (int64_t)sp_aud.target_ms * 1000 - 15000);
#ifdef __APPLE__
        sp_aud.dev_starved_seen = spacebox_coreaudio_starved_total();
#endif
    }
#ifdef __APPLE__
    {
        /* requests keep coming while nothing plays; only those during steady
         * playing count */
        uint64_t st = spacebox_coreaudio_starved_total();

        if (st != sp_aud.dev_starved_seen) {
            if (sp_aud.playing && now - sp_aud.t_start > 500000 &&
                now - sp_aud.last_rx < 100000) {
                sp_aud.dev_starved += st - sp_aud.dev_starved_seen;
                if (sp_aud.lead_us < 100000 &&
                    now - sp_aud.lead_raised > G_USEC_PER_SEC) {
                    sp_aud.lead_us += 5000;
                    sp_aud.lead_raised = now;
                }
            }
            sp_aud.dev_starved_seen = st;
        }
    }
#endif
    return sp_aud.lead_us;
}

/*
 * Sound has a connection of its own (SPACEBOX_GPU_REMOTE_AUDIO=<unix socket>,
 * the sink listens, the source connects). On the shared connection every late
 * or lost packet of the picture's data also holds up the sound behind it, and
 * a gap in sound is heard at once; on its own TCP connection the sound only
 * waits for its own packets. If that connection is not there the sound goes
 * with the rest, as before.
 */
static struct {
    const char *path;
    int fd;             /* source: connected; sink: accepted */
    int listen_fd;      /* sink */
    int64_t last_try;
    GByteArray *rx;     /* sink */
    uint64_t sent, dropped;
} sp_ach = { .fd = -1, .listen_fd = -1 };

static void sp_ach_close(void)
{
    if (sp_ach.fd >= 0) {
        qemu_set_fd_handler(sp_ach.fd, NULL, NULL, NULL);
        close(sp_ach.fd);
        sp_ach.fd = -1;
    }
    if (sp_ach.rx) {
        g_byte_array_set_size(sp_ach.rx, 0);
    }
}

static bool sp_ach_source_ready(void)
{
    struct sockaddr_un addr = { .sun_family = AF_UNIX };
    int64_t now = g_get_monotonic_time();
    int fd;

    if (sp_ach.fd >= 0) {
        return true;
    }
    if (!sp_ach.path) {
        sp_ach.path = getenv("SPACEBOX_GPU_REMOTE_AUDIO");
        if (!sp_ach.path) {
            sp_ach.path = "";
        }
    }
    if (!*sp_ach.path || now - sp_ach.last_try < G_USEC_PER_SEC) {
        return false;
    }
    sp_ach.last_try = now;
    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        return false;
    }
    g_strlcpy(addr.sun_path, sp_ach.path, sizeof(addr.sun_path));
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd);
        return false;
    }
    qemu_socket_set_nonblock(fd);
#ifdef SO_NOSIGPIPE
    {
        int yes = 1;
        setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &yes, sizeof(yes));
    }
#endif
    sp_ach.fd = fd;
    info_report("spacebox remote gpu: sound has its own connection");
    return true;
}

void spacebox_remote_audio_out(int freq, int channels, const void *buf, size_t len)
{
    if (sp.mode != SP_REMOTE_SOURCE || sp.fd < 0 || !sp.hello_done) {
        return;
    }
    if (sp_ach_source_ready()) {
        SpHdr h = {
            .magic = SP_MAGIC, .type = SP_AUDIO, .a_len = len,
            .arg0 = freq, .arg1 = channels, .t_us = g_get_monotonic_time(),
        };
        struct iovec iov[2] = {
            { .iov_base = &h, .iov_len = sizeof(h) },
            { .iov_base = (void *)buf, .iov_len = len },
        };
        struct msghdr m = { .msg_iov = iov, .msg_iovlen = 2 };
        ssize_t n;
        int flags = 0;

#ifdef MSG_NOSIGNAL
        flags |= MSG_NOSIGNAL;
#endif
        n = sendmsg(sp_ach.fd, &m, flags);
        if (n == (ssize_t)(sizeof(h) + len)) {
            sp_ach.sent++;
            return;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            sp_ach.dropped++;   /* the connection is backed up: skip this bit */
            return;
        }
        /* broken, or a partial write that leaves the stream out of step */
        sp_ach_close();
    }
    sp_send_full(false, SP_AUDIO, 0, buf, len, NULL, 0, freq, channels, 0);
}

static void sp_sink_audio_cb(void *opaque, int avail)
{
    size_t frame = 2 * sp_aud.channels;
    uint64_t bps = (uint64_t)sp_aud.freq * frame;
    int64_t now = g_get_monotonic_time();
    int64_t allowed;

    if (!sp_aud.playing) {
        return;
    }
    /* what playback speed allows by now, plus the lead that waits in the
     * backend */
    allowed = (now - sp_aud.t_start + sp_aud_lead(now)) * bps / G_USEC_PER_SEC - sp_aud.fed;
    allowed -= allowed % (int64_t)frame;
    while (sp_aud.fast && avail >= (int)frame && allowed >= (int64_t)frame &&
           sp_aud.len > 2 * frame) {
        /* 100 output frames from 101 input frames, by linear interpolation */
        int16_t out[2 * 256];
        size_t want = MIN(MIN((size_t)avail, (size_t)allowed) / frame, 256), n = 0, w;

        while (n < want && sp_aud.len > 2 * frame) {
            for (int ch = 0; ch < sp_aud.channels; ch++) {
                int16_t x, y;

                memcpy(&x, sp_aud.ring + (sp_aud.head + 2 * ch) % sp_aud.cap, 2);
                memcpy(&y, sp_aud.ring + (sp_aud.head + frame + 2 * ch) % sp_aud.cap, 2);
                out[n * sp_aud.channels + ch] = x + (int16_t)((y - x) * sp_aud.frac);
            }
            n++;
            sp_aud.frac += 1.01;
            while (sp_aud.frac >= 1.0 && sp_aud.len > 2 * frame) {
                sp_aud.frac -= 1.0;
                sp_aud.head = (sp_aud.head + frame) % sp_aud.cap;
                sp_aud.len -= frame;
            }
        }
        if (!n) {
            break;
        }
        w = AUD_write(sp_aud.voice, out, n * frame);
        if (sp_aud.dump && w) {
            int64_t rec[2] = { now, (int64_t)w };

            fwrite(rec, sizeof(rec), 1, sp_aud.dump);
            fwrite(out, 1, w, sp_aud.dump);
        }
        sp_aud.played += w;
        sp_aud.fed += w;
        sp_aud.fast_ms += w * 1000 / bps;
        avail -= w;
        allowed -= w;
        if (w < n * frame) {
            break;
        }
    }
    while (!sp_aud.fast && avail > 0 && allowed > 0 && sp_aud.len) {
        size_t n = MIN(MIN((size_t)avail, sp_aud.len), sp_aud.cap - sp_aud.head);
        size_t w;

        n = MIN(n, (size_t)allowed);
        w = AUD_write(sp_aud.voice, sp_aud.ring + sp_aud.head, n);
        if (!w) {
            break;
        }
        if (sp_aud.dump) {
            /* a record per hand-over: time (us), byte count, then the samples */
            int64_t rec[2] = { now, (int64_t)w };

            fwrite(rec, sizeof(rec), 1, sp_aud.dump);
            fwrite(sp_aud.ring + sp_aud.head, 1, w, sp_aud.dump);
        }
        sp_aud.head = (sp_aud.head + w) % sp_aud.cap;
        sp_aud.len -= w;
        sp_aud.played += w;
        sp_aud.fed += w;
        avail -= w;
        allowed -= w;
    }
    /* nothing left here, and what waits in the backend is down to a request
     * and a half */
    if (!sp_aud.len &&
        allowed > (int64_t)(MAX(sp_aud.lead_us - 17000, 20000) * bps / G_USEC_PER_SEC)) {
        sp_aud.playing = false; /* ran dry: collect the target again */
        if (now - sp_aud.last_rx < 300000) {
            sp_aud.underruns++; /* not simply the end of the sound */
            sp_aud.target_ms = MIN(sp_aud.target_ms + 20, sp_aud.base_ms + 40);
            sp_aud.target = (size_t)sp_aud.freq * sp_aud.target_ms / 1000 * frame;
            sp_aud.last_underrun = now;
        } else {
            sp_aud_delay_us = 0;
            sp_aud.held_avg = 0;
        }
    }
}

static void sp_sink_audio(SpHdr *h, const uint8_t *a)
{
    int freq = h->arg0, channels = h->arg1;
    size_t frame = 2 * channels, len = h->a_len;

    if (sp_aud.failed || freq < 8000 || freq > 192000 || channels < 1 ||
        channels > 2 || !len) {
        return;
    }
    if (!sp_aud.voice || freq != sp_aud.freq || channels != sp_aud.channels) {
        struct audsettings as = {
            .freq = freq, .nchannels = channels, .fmt = AUDIO_FORMAT_S16,
            .endianness = 0,
        };
        const char *ms = getenv("SPACEBOX_AUDIO_BUFFER_MS");
        Error *err = NULL;

        if (!sp_aud.card.name) {
            /* the sink is started with -audiodev <backend>,id=spa */
            sp_aud.card.state = audio_state_by_name("spa", &err);
            if (!sp_aud.card.state ||
                !AUD_register_card("spacebox-remote", &sp_aud.card, &err)) {
                error_report_err(err);
                sp_aud.failed = true;
                return;
            }
        }
        sp_aud.voice = AUD_open_out(&sp_aud.card, sp_aud.voice, "spacebox-remote",
                                    NULL, sp_sink_audio_cb, &as);
        if (!sp_aud.voice) {
            error_report("spacebox remote gpu: cannot open a sound output");
            sp_aud.failed = true;
            return;
        }
        sp_aud.freq = freq;
        sp_aud.channels = channels;
        g_free(sp_aud.ring);
        sp_aud.cap = (size_t)freq * frame;      /* one second */
        sp_aud.ring = g_malloc(sp_aud.cap);
        sp_aud.head = sp_aud.len = 0;
        sp_aud.base_ms = sp_aud.target_ms = ms ? MAX(atoi(ms), 10) : 80;
        sp_aud.target = (size_t)freq * sp_aud.target_ms / 1000 * frame;
        sp_aud.playing = false;
        AUD_set_active_out(sp_aud.voice, 1);
        /* For tests while nobody should hear anything: SPACEBOX_AUDIO_SILENT=1
         * plays at volume 0, SPACEBOX_AUDIO_DUMP=<file> records what is
         * handed to the backend. */
        if (getenv("SPACEBOX_AUDIO_SILENT")) {
            AUD_set_volume_out(sp_aud.voice, 1, 0, 0);
        }
        if (getenv("SPACEBOX_AUDIO_DUMP") && !sp_aud.dump) {
            sp_aud.dump = fopen(getenv("SPACEBOX_AUDIO_DUMP"), "wb");
        }
    }

    len -= len % frame;
    sp_aud.rx_bytes += len;
    if (len > sp_aud.cap) {
        a += len - sp_aud.cap;
        len = sp_aud.cap;
    }
    if (sp_aud.len + len > sp_aud.cap) {
        size_t drop = sp_aud.len + len - sp_aud.cap;

        sp_aud.head = (sp_aud.head + drop) % sp_aud.cap;
        sp_aud.len -= drop;
        sp_aud.dropped += drop;
    }
    {
        size_t tail = (sp_aud.head + sp_aud.len) % sp_aud.cap;
        size_t first = MIN(len, sp_aud.cap - tail);

        memcpy(sp_aud.ring + tail, a, first);
        memcpy(sp_aud.ring, a + first, len - first);
        sp_aud.len += len;
    }
    if (sp_aud.len > sp_aud.target * 3) {
        size_t drop = sp_aud.len - sp_aud.target;

        drop -= drop % frame;
        sp_aud.head = (sp_aud.head + drop) % sp_aud.cap;
        sp_aud.len -= drop;
        sp_aud.dropped += drop;
    }
    sp_aud.last_rx = g_get_monotonic_time();
    if (sp_aud.target_ms > sp_aud.base_ms &&
        sp_aud.last_rx - sp_aud.last_underrun > 30 * G_USEC_PER_SEC &&
        sp_aud.last_rx - sp_aud.last_lowered > 30 * G_USEC_PER_SEC) {
        sp_aud.target_ms = MAX(sp_aud.target_ms - 10, sp_aud.base_ms);
        sp_aud.target = (size_t)freq * sp_aud.target_ms / 1000 * frame;
        sp_aud.last_lowered = sp_aud.last_rx;
    }
    if (!sp_aud.playing && sp_aud.len >= sp_aud.target) {
        sp_aud.playing = true;
        sp_aud.t_start = sp_aud.last_rx;
        sp_aud.fed = 0;
    }
    /* How much later the sound comes out: what is buffered on average here
     * and in the backend (the lead), less the 20 ms the picture/sound offset
     * was tuned with. A longer lead moves sound from this buffer into the
     * backend; it does not make the sound later. */
    {
        int64_t bps = (int64_t)freq * frame;
        int64_t held = (int64_t)sp_aud.len * G_USEC_PER_SEC / bps;
        int64_t target_us = (int64_t)sp_aud.target * G_USEC_PER_SEC / bps;
        int64_t more_lead = sp_aud_lead(sp_aud.last_rx) - SP_AUD_LEAD_OLD_US;

        held += more_lead;
        sp_aud.held_avg = sp_aud.held_avg ? sp_aud.held_avg + (held - sp_aud.held_avg) / 64 : held;
        /*
         * After a hold-up the sound that piled up arrives at once and the
         * buffer stays that much fuller, i.e. the sound that much later. While
         * it is well above the target the sound is played 1% faster (see
         * sp_sink_audio_cb) until it is back.
         */
        if (sp_aud.held_avg > target_us * 5 / 4) {
            sp_aud.fast = true;
        } else if (sp_aud.held_avg < target_us * 21 / 20) {
            sp_aud.fast = false;
        }
        sp_aud_delay_us = MAX(sp_aud.held_avg - 20000, 1);
    }
}

static void sp_ach_sink_rx(void *opaque)
{
    uint8_t buf[16384];
    ssize_t n = recv(sp_ach.fd, buf, sizeof(buf), 0);
    size_t off = 0;

    if (n == 0 || (n < 0 && errno != EAGAIN && errno != EINTR)) {
        sp_ach_close();
        return;
    }
    if (n < 0) {
        return;
    }
    g_byte_array_append(sp_ach.rx, buf, n);
    while (sp_ach.rx->len - off >= sizeof(SpHdr)) {
        SpHdr h;

        memcpy(&h, sp_ach.rx->data + off, sizeof(h));
        if (h.magic != SP_MAGIC || h.type != SP_AUDIO || h.a_len > (1 << 20)) {
            sp_ach_close();
            return;
        }
        if (sp_ach.rx->len - off < sizeof(h) + h.a_len) {
            break;
        }
        sp_sink_audio(&h, sp_ach.rx->data + off + sizeof(h));
        off += sizeof(h) + h.a_len;
    }
    if (off) {
        g_byte_array_remove_range(sp_ach.rx, 0, off);
    }
}

static void sp_ach_sink_accept(void *opaque)
{
    int fd = accept(sp_ach.listen_fd, NULL, NULL);

    if (fd < 0) {
        return;
    }
    sp_ach_close();     /* the newer connection wins */
    qemu_socket_set_nonblock(fd);
    sp_ach.fd = fd;
    qemu_set_fd_handler(fd, sp_ach_sink_rx, NULL, NULL);
}

static void sp_ach_sink_listen(void)
{
    struct sockaddr_un addr = { .sun_family = AF_UNIX };
    const char *path = getenv("SPACEBOX_GPU_REMOTE_AUDIO");

    if (!path || !*path) {
        return;
    }
    sp_ach.rx = g_byte_array_new();
    g_strlcpy(addr.sun_path, path, sizeof(addr.sun_path));
    unlink(path);
    sp_ach.listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sp_ach.listen_fd < 0 ||
        bind(sp_ach.listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
        listen(sp_ach.listen_fd, 1) < 0) {
        warn_report("spacebox remote gpu: cannot listen for sound on %s", path);
        return;
    }
    qemu_set_fd_handler(sp_ach.listen_fd, sp_ach_sink_accept, NULL, NULL);
}

static struct {
    int64_t total, submit, flush, submit_max, flush_max, other_max;
} sp_busy;

static void sp_dispatch(SpHdr *h, uint8_t *a, uint8_t *b)
{
    VirtIOGPU *g = sp.g;

    if (sp.mode == SP_REMOTE_SINK) {
        switch (h->type) {
        case SP_CTRL: {
            /* statistics: how long the sink spends running the guest's commands */
            int64_t t0 = g_get_monotonic_time(), d;
            uint32_t type = 0;

            if (h->a_len >= 4) {
                memcpy(&type, a, 4);
            }
            sp_sink_ctrl(g, h, a, b);
            d = g_get_monotonic_time() - t0;
            sp_busy.total += d;
            if (type == VIRTIO_GPU_CMD_SUBMIT_3D) {
                sp_busy.submit += d;
                sp_busy.submit_max = MAX(sp_busy.submit_max, d);
            } else if (type == VIRTIO_GPU_CMD_RESOURCE_FLUSH) {
                sp_busy.flush += d;
                sp_busy.flush_max = MAX(sp_busy.flush_max, d);
            } else {
                sp_busy.other_max = MAX(sp_busy.other_max, d);
            }
            break;
        }
        case SP_CURSOR:
            if (h->a_len >= sizeof(struct virtio_gpu_update_cursor) &&
                sp_sink_renderer(g)) {
                struct virtio_gpu_update_cursor c;

                memcpy(&c, a, sizeof(c));
                virtio_gpu_sp_update_cursor(g, &c);
            }
            break;
        case SP_RESET:
            sp_sink_reset(g);
            break;
        case SP_MEMWRITE:
            sp_sink_memwrite(h, a);
            break;
        case SP_EXPECT:
            if (sp.next_expect) {
                g_byte_array_unref(sp.next_expect);
            }
            sp.next_expect = g_byte_array_new();
            g_byte_array_append(sp.next_expect, a, h->a_len);
            break;
        case SP_AUDIO:
            sp_sink_audio(h, a);
            break;
        default:
            break;
        }
    } else {
        switch (h->type) {
        case SP_RESP:
            sp_source_resp(g, h, a, b);
            break;
        case SP_INPUT:
            sp_source_input(h, a);
            break;
        case SP_MEMWRITE:
            sp_source_memwrite(g, h, a);
            break;
        case SP_INFO:
            sp_source_info(h, a);
            break;
        case SP_DISPLAY:
            virtio_gpu_base_sp_notify_event(VIRTIO_GPU_BASE(g), h->arg0);
            break;
        default:
            break;
        }
    }
}

static void sp_stats(void *opaque)
{
    uint64_t wire_tx;

    qemu_mutex_lock(&sp.tx_lock);
    wire_tx = sp.wire_tx;
    sp.wire_tx = 0;
    qemu_mutex_unlock(&sp.tx_lock);
    sp_flush_report(sp.mode == SP_REMOTE_SOURCE ? "source" : "sink");
    if (sp.mode == SP_REMOTE_SINK && !sp_late_timer &&
        getenv("SPACEBOX_GPU_REMOTE_DEBUG_LOOP")) {
        /* a timer every 2 ms; only when asked for, it keeps the process awake */
        sp_late_timer = timer_new_ms(QEMU_CLOCK_REALTIME, sp_late_tick, NULL);
        sp_late_due = g_get_monotonic_time() + 2000;
        timer_mod(sp_late_timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 2);
    }
    if (sp.mode == SP_REMOTE_SOURCE || sp_late_timer) {
        fprintf(stderr, "[SPACEBOX-REMOTE] %s=%u over 12 ms=%u over 25 ms=%u over 50 ms=%u longest_ms=%.1f\n",
                sp.mode == SP_REMOTE_SOURCE ? "source answers from the sink" : "sink main loop checks",
                sp_delay.n, sp_delay.over12, sp_delay.over25, sp_delay.over50, sp_delay.max / 1000.0);
    }
    memset(&sp_delay, 0, sizeof(sp_delay));
    qemu_mutex_lock(&sp.tx_lock);
    fprintf(stderr, "[SPACEBOX-REMOTE] %s messages written=%u queue-to-written over 12 ms=%u over 25 ms=%u over 50 ms=%u longest_ms=%.1f\n",
            sp.mode == SP_REMOTE_SOURCE ? "source" : "sink",
            sp_txd.n, sp_txd.over12, sp_txd.over25, sp_txd.over50, sp_txd.max / 1000.0);
    memset(&sp_txd, 0, sizeof(sp_txd));
    qemu_mutex_unlock(&sp.tx_lock);
    fprintf(stderr, "[SPACEBOX-REMOTE] %s messages received=%u later than the fastest by over 12 ms=%u over 25 ms=%u over 50 ms=%u longest_ms=%.1f\n",
            sp.mode == SP_REMOTE_SOURCE ? "source" : "sink",
            sp_rxd.n, sp_rxd.over12, sp_rxd.over25, sp_rxd.over50, sp_rxd.max / 1000.0);
    memset(&sp_rxd, 0, sizeof(sp_rxd));
    if (sp.mode == SP_REMOTE_SINK) {
        fprintf(stderr, "[SPACEBOX-REMOTE] sink time running guest commands: total_ms=%.0f drawing_ms=%.0f (longest %.1f)"
                " screen_update_ms=%.0f (longest %.1f) other longest %.1f\n",
                sp_busy.total / 1000.0, sp_busy.submit / 1000.0, sp_busy.submit_max / 1000.0,
                sp_busy.flush / 1000.0, sp_busy.flush_max / 1000.0, sp_busy.other_max / 1000.0);
        memset(&sp_busy, 0, sizeof(sp_busy));
    }
    if (sp.mode == SP_REMOTE_SOURCE && (sp_ach.sent || sp_ach.dropped)) {
        fprintf(stderr, "[SPACEBOX-REMOTE] source sound on its own connection: messages=%" PRIu64
                " skipped=%" PRIu64 "\n", sp_ach.sent, sp_ach.dropped);
        sp_ach.sent = sp_ach.dropped = 0;
    }
    if (sp.mode == SP_REMOTE_SINK && sp_aud.voice) {
        unsigned bps = sp_aud.freq * sp_aud.channels * 2;

        uint64_t dev_requests = 0, dev_starved = 0;
        uint32_t dev_frames = 0, dev_least = 0;

#ifdef __APPLE__
        spacebox_coreaudio_stats(&dev_requests, &dev_starved, &dev_frames, &dev_least);
#endif
        fprintf(stderr, "[SPACEBOX-REMOTE] sink sound: received_ms=%" PRIu64 " played_ms=%" PRIu64
                " dropped_ms=%" PRIu64 " ran_dry=%" PRIu64 " buffered_ms=%zu target_ms=%d played_faster_ms=%" PRIu64
                " device_requests=%" PRIu64 " device_got_nothing=%" PRIu64
                " device_request_frames=%u least_waiting_frames=%u"
                " got_nothing_while_playing=%" PRIu64 " lead_ms=%d\n",
                sp_aud.rx_bytes * 1000 / bps, sp_aud.played * 1000 / bps,
                sp_aud.dropped * 1000 / bps, sp_aud.underruns, sp_aud.len * 1000 / bps,
                sp_aud.target_ms, sp_aud.fast_ms, dev_requests, dev_starved,
                dev_frames, dev_least == UINT32_MAX ? 0 : dev_least,
                sp_aud.dev_starved, (int)(sp_aud.lead_us / 1000));
        sp_aud.dev_starved = 0;
        sp_aud.fast_ms = 0;
        sp_aud.rx_bytes = sp_aud.played = sp_aud.dropped = sp_aud.underruns = 0;
    }
    if (sp.mode == SP_REMOTE_SINK) {
        fprintf(stderr, "[SPACEBOX-REMOTE] sink frame pacing: screen updates=%" PRIu64 " held for their time=%" PRIu64
                " too late for it=%" PRIu64 " shown at once after input=%" PRIu64
                " allowance_ms=%.1f usual_spacing_ms=%.1f\n",
                sp_pace.frames, sp_pace.paced, sp_pace.late_frames, sp_pace.unpaced_for_input,
                sp_pace.allowance / 1000.0, sp_pace.usual_gap / 1000.0);
        sp_pace.frames = sp_pace.paced = sp_pace.late_frames = sp_pace.unpaced_for_input = 0;
    }
    if (sp.mode == SP_REMOTE_SOURCE) {
        GHashTableIter it;
        gpointer value;
        SpPending *oldest = NULL;

        g_hash_table_iter_init(&it, sp.pending);
        while (g_hash_table_iter_next(&it, NULL, &value)) {
            SpPending *p = value;
            if (!oldest || p->seq < oldest->seq) {
                oldest = p;
            }
        }
        fprintf(stderr, "[SPACEBOX-REMOTE] source pending=%u", g_hash_table_size(sp.pending));
        if (oldest) {
            fprintf(stderr, " oldest: seq=%" PRIu64 " type=0x%x early=%d age_ms=%" PRId64,
                    oldest->seq, oldest->type, oldest->early,
                    (g_get_monotonic_time() - oldest->t0) / 1000);
        }
        fprintf(stderr, " last_seq=%" PRIu64 " answered_here=%" PRIu64
                " fence_early=%" PRIu64 " waited=%" PRIu64 " fence_out=%u rtt_ms=%.1f"
                " not_sent=%" PRIu64 " not_sent_bytes=%" PRIu64
                " early_by_age=%" PRIu64 " rtt_min_ms=%.1f\n",
                sp.seq, sp.n_local, sp.n_early, sp.n_waited, sp.fence_out,
                sp.rtt_us / 1000.0, sp.n_defer, sp.defer_bytes, sp.n_ahead,
                MIN(sp.rtt_min_cur, sp.rtt_min_prev) == INT64_MAX ? 0.0 :
                MIN(sp.rtt_min_cur, sp.rtt_min_prev) / 1000.0);
        sp.n_local = sp.n_early = sp.n_waited = sp.n_ahead = 0;
        sp.n_defer = sp.defer_bytes = 0;
        for (unsigned i = 0; i < 64; i++) {
            if (sp.ctx_qpoll[i]) {
                fprintf(stderr, "[SPACEBOX-REMOTE] source query polls answered here: "
                        "ctx=%u (%s) %u\n", i, sp.ctx_name[i], sp.ctx_qpoll[i]);
                sp.ctx_qpoll[i] = 0;
            }
        }
    } else if (sp.g) {
        struct virtio_gpu_ctrl_command *c;
        unsigned nq = 0, nf = 0;

        QTAILQ_FOREACH(c, &sp.g->cmdq, next) nq++;
        QTAILQ_FOREACH(c, &sp.g->fenceq, next) nf++;
        c = QTAILQ_FIRST(&sp.g->cmdq);
        fprintf(stderr, "[SPACEBOX-REMOTE] sink cmdq=%u fenceq=%u blocked=%d", nq, nf,
                sp.g->parent_obj.renderer_blocked);
        if (c) {
            fprintf(stderr, " cmdq_head: seq=%" PRIu64 " type=0x%x", c->sp_seq, c->cmd_hdr.type);
        }
        c = QTAILQ_FIRST(&sp.g->fenceq);
        if (c) {
            fprintf(stderr, " fenceq_head: seq=%" PRIu64 " type=0x%x flags=0x%x fence=%" PRIu64
                    " ctx=%u", c->sp_seq, c->cmd_hdr.type, c->cmd_hdr.flags,
                    (uint64_t)c->cmd_hdr.fence_id, c->cmd_hdr.ctx_id);
        }
        fprintf(stderr, "\n");
    }
    fprintf(stderr, "[SPACEBOX-REMOTE] %s ctrl=%" PRIu64 " submit=%" PRIu64
            " submit_bytes=%" PRIu64 " xfer=%" PRIu64 " xfer_bytes=%" PRIu64
            " mem=%" PRIu64 " mem_bytes=%" PRIu64 " back=%" PRIu64
            " back_bytes=%" PRIu64
            " tx_bytes=%" PRIu64 " rx_bytes=%" PRIu64 " backlog=%zu"
            " unacked=%zu reconnects=%" PRIu64 " wire_tx_bytes=%" PRIu64
            " wire_rx_bytes=%" PRIu64 " compressed=%d\n",
            sp.mode == SP_REMOTE_SOURCE ? "source" : "sink",
            sp.n_ctrl, sp.n_submit, sp.submit_bytes, sp.n_xfer, sp.xfer_bytes,
            sp.n_mem, sp.mem_bytes, sp.n_back, sp.back_bytes,
            sp.tx_bytes, sp.rx_bytes, sp_tx_backlog(), sp.tx_unacked,
            sp.n_reconnect, wire_tx, sp.wire_rx, sp.z_on);
    sp.n_ctrl = sp.n_submit = sp.submit_bytes = sp.n_xfer = sp.xfer_bytes = 0;
    sp.n_mem = sp.mem_bytes = sp.n_back = sp.back_bytes = 0;
    sp.tx_bytes = sp.rx_bytes = sp.wire_rx = 0;
    timer_mod(sp.stats_timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 5000);
}

void sp_remote_realize(VirtIOGPU *g, Error **errp)
{
    if (!sp_remote_mode()) {
        return;
    }
    sp.g = g;
    sp.rx = g_byte_array_new();
    sp.rxw = g_byte_array_new();
#ifdef SPACEBOX_ZSTD
    sp.z_d = ZSTD_createDCtx();
#endif
    qemu_mutex_init(&sp.tx_lock);
    qemu_cond_init(&sp.tx_cond);
    g_queue_init(&sp.tx_queue);
    qemu_thread_create(&sp.tx_thread, "spacebox-gpu-tx", sp_tx_thread, NULL,
                       QEMU_THREAD_DETACHED);
    do {
        sp.session = ((uint64_t)g_random_int() << 32) | g_random_int();
    } while (!sp.session);
    sp.ack_timer = timer_new_ms(QEMU_CLOCK_REALTIME, sp_ack_timer, NULL);
    timer_mod(sp.ack_timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 100);
    if (sp.stats) {
        sp.stats_timer = timer_new_ms(QEMU_CLOCK_REALTIME, sp_stats, NULL);
        timer_mod(sp.stats_timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 5000);
    }

    if (sp.mode == SP_REMOTE_SOURCE) {
        sp.pending = g_hash_table_new_full(g_int64_hash, g_int64_equal,
                                           g_free, g_free);
        sp.queries = g_hash_table_new_full(g_int64_hash, g_int64_equal,
                                           g_free, NULL);
        sp.query_bufs = g_hash_table_new(g_direct_hash, g_direct_equal);
        sp.custom_bufs = g_hash_table_new(g_direct_hash, g_direct_equal);
        sp.retry = timer_new_ms(QEMU_CLOCK_REALTIME, sp_source_connect, NULL);
        timer_mod(sp.retry, qemu_clock_get_ms(QEMU_CLOCK_REALTIME));
    } else {
        struct sockaddr_un addr = { .sun_family = AF_UNIX };

        sp.shadow = g_hash_table_new_full(g_direct_hash, g_direct_equal,
                                          NULL, g_free);
        sp.watch = g_hash_table_new_full(g_direct_hash, g_direct_equal,
                                         NULL, g_free);
        g_strlcpy(addr.sun_path, sp.path, sizeof(addr.sun_path));
        unlink(sp.path);
        sp.listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (sp.listen_fd < 0 ||
            bind(sp.listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
            listen(sp.listen_fd, 1) < 0) {
            error_setg_errno(errp, errno, "spacebox remote gpu: cannot listen "
                             "on %s", sp.path);
            return;
        }
        qemu_set_fd_handler(sp.listen_fd, sp_sink_accept, NULL, NULL);
        sp_ach_sink_listen();
        sp.input = qemu_input_handler_register(DEVICE(g), &sp_input_handler);
        qemu_input_handler_activate(sp.input);
        info_report("spacebox remote gpu: sink listening on %s", sp.path);
    }
}
