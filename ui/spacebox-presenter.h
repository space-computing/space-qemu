/* Experimental Mac presenter: private bounded snapshots, shared GL contexts.
 * Guest processing never waits for display refresh or a consumer GL fence.
 * The display-link callback only signals a condition variable. */
#ifdef CONFIG_DARWIN
#include <CoreVideo/CoreVideo.h>
#include <CoreGraphics/CoreGraphics.h>
#include <pthread/qos.h>
#include "qemu/thread.h"
/* Frames kept ahead of the one on screen. Without pacing only a few are ever
 * in use (their textures are made on first use); with pacing up to about 12. */
#define SP_SLOTS 16
/* hw/display/virtio-gpu-remote.c: when to show the frame being submitted, 0 = at once */
extern int64_t spacebox_frame_due_us;
extern int64_t spacebox_frame_gap_us;
enum SpSlotState { SP_FREE, SP_WRITING, SP_READY, SP_READING, SP_DISPLAYED };
typedef struct SpSlot {
    egl_fb fb;
    GLsync ready, consumed;
    enum SpSlotState state;
    uint64_t sequence;
    int64_t submitted;
    int64_t due;    /* show no earlier than this, 0 = at once */
    int64_t gap;    /* usual spacing of the stream this frame belongs to, 0 = none */
} SpSlot;
typedef struct SpPresenter {
    SDL_Window *window;
    SDL_GLContext output;
    QemuThread thread;
    QemuMutex lock;
    QemuCond wake;
    CVDisplayLinkRef link;
    bool stop;
    uint64_t tick, sequence, presented, discarded, busy;
    SpSlot slots[SP_SLOTS];
} SpPresenter;

static CVReturn sp_present_tick(CVDisplayLinkRef link, const CVTimeStamp *now,
                               const CVTimeStamp *out, CVOptionFlags in,
                               CVOptionFlags *flags, void *opaque)
{
    SpPresenter *p = opaque;
    qemu_mutex_lock(&p->lock);
    p->tick++;
    qemu_cond_signal(&p->wake);
    qemu_mutex_unlock(&p->lock);
    return kCVReturnSuccess;
}

static bool sp_fence_done(GLsync fence)
{
    if (!fence) return true;
    GLenum r = glClientWaitSync(fence, 0, 0);
    return r == GL_ALREADY_SIGNALED || r == GL_CONDITION_SATISFIED;
}

static bool sp_has_ready(SpPresenter *p)
{
    for (int i = 0; i < SP_SLOTS; i++) if (p->slots[i].state == SP_READY) return true;
    return false;
}

static void *sp_present_thread(void *opaque)
{
    SpPresenter *p = opaque;
    uint64_t seen = 0;
    int shown = -1;
    GLuint readfb;
    int64_t last_stats = 0, last_control = 0;
    const char *scheduler = getenv("SPACEBOX_PRESENT_SCHEDULER");
    bool native = scheduler && !strcmp(scheduler, "native");
    const char *phase_env = getenv("SPACEBOX_PRESENT_PHASE_US");
    int phase_us = phase_env ? atoi(phase_env) : 0;
    const char *phase_file = getenv("SPACEBOX_PRESENT_PHASE_CONTROL");
    /* SPACEBOX_PRESENT_PROBE=<file>: one line per frame shown, "time_us r g b" */
    FILE *probe = getenv("SPACEBOX_PRESENT_PROBE") ? fopen(getenv("SPACEBOX_PRESENT_PROBE"), "w") : NULL;
    /* SPACEBOX_PRESENT_STATS=<seconds between lines> */
    const char *stats_env = getenv("SPACEBOX_PRESENT_STATS");
    int64_t stats_us = stats_env ? (int64_t)MAX(atoi(stats_env), 1) * 1000000 : 0;
    int64_t refresh_us = 8333, last_swap = 0, held_max = 0, phase_adj = 0;
    /* where between two refreshes a frame's time is kept: this fraction of a
     * refresh before the one it is shown at (SPACEBOX_PRESENT_PHASE_TARGET) */
    double phase_target = getenv("SPACEBOX_PRESENT_PHASE_TARGET") ? atof(getenv("SPACEBOX_PRESENT_PHASE_TARGET")) : 0.5;
    bool waited_for_due = false;
    uint64_t held[7] = { 0 };
    {
        CVTime period = CVDisplayLinkGetNominalOutputVideoRefreshPeriod(p->link);
        if (!(period.flags & kCVTimeIsIndefinite) && period.timeScale > 0 && period.timeValue > 0) {
            refresh_us = period.timeValue * 1000000 / period.timeScale;
        }
    }
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INITIATED, 0);
    if (SDL_GL_MakeCurrent(p->window, p->output)) {
        fprintf(stderr, "[SPACEBOX-PRESENT-ERROR] make_current: %s\n", SDL_GetError());
        return NULL;
    }
    const char *interval = getenv("SPACEBOX_PRESENT_INTERVAL");
    SDL_GL_SetSwapInterval(interval ? atoi(interval) : 1);
    glGenFramebuffers(1, &readfb);
    for (;;) {
        qemu_mutex_lock(&p->lock);
        while (!p->stop && (native ? !sp_has_ready(p) : p->tick == seen)) qemu_cond_wait(&p->wake, &p->lock);
        if (p->stop) { qemu_mutex_unlock(&p->lock); break; }
        seen = p->tick;
        qemu_mutex_unlock(&p->lock);
        int64_t check = g_get_monotonic_time();
        if (phase_file && check - last_control > 500000) {
            FILE *f = fopen(phase_file, "r");
            int next, scheduler_mode = -1;
            if (f) {
                int fields = fscanf(f, "%d %d", &next, &scheduler_mode);
                if (fields >= 1 && next >= 0 && next <= 7000) phase_us = next;
                if (fields >= 2 && (scheduler_mode == 0 || scheduler_mode == 1)) native = scheduler_mode == 1;
                fclose(f);
            }
            last_control = check;
        }
        if (phase_us > 0 && phase_us <= 7000) g_usleep(phase_us);
        qemu_mutex_lock(&p->lock);
        if (p->stop) { qemu_mutex_unlock(&p->lock); break; }
        int pick = -1, oldest = -1, newest = -1, n_due = 0;
        bool stream = true;
        int64_t pick_now = g_get_monotonic_time(), next_due = 0;
        for (int i = 0; i < SP_SLOTS; i++) {
            SpSlot *s = &p->slots[i];
            if (s->state != SP_READY || !sp_fence_done(s->ready)) continue;
            int64_t due = s->due ? s->due + phase_adj : 0;
            if (due > pick_now) {
                if (!next_due || due < next_due) next_due = due;
                continue;
            }
            n_due++;
            if (!s->due || !s->gap) stream = false;
            if (oldest < 0 || s->sequence < p->slots[oldest].sequence) oldest = i;
            if (newest < 0 || s->sequence > p->slots[newest].sequence) newest = i;
        }
        if (n_due && (!stream || n_due >= 4)) {
            /* Frames without a time, or far behind: the newest one, now. */
            pick = newest;
        } else if (n_due) {
            /*
             * A paced stream: in order, and not sooner after the previous
             * frame than the stream's usual spacing in whole refreshes. The
             * swap lands on the refresh after the wake-up, so wake just after
             * the one before it (last_swap is a little after a refresh). When
             * a second frame's time has come as well, one refresh sooner, so
             * that being late is made up by a shorter frame instead of a
             * dropped one.
             */
            SpSlot *s = &p->slots[oldest];
            int64_t k = MAX((s->gap + refresh_us / 2) / refresh_us, 1);
            if (n_due >= 2 && k > 1) k--;
            int64_t earliest = last_swap ? last_swap + (k - 1) * refresh_us + refresh_us / 8 : 0;
            if (earliest > pick_now) {
                if (!next_due || earliest < next_due) next_due = earliest;
            } else {
                pick = oldest;
            }
        }
        if (pick < 0) {
            if (next_due) {
                waited_for_due = n_due == 0;
                qemu_cond_timedwait(&p->wake, &p->lock, (int)MAX((next_due - pick_now + 500) / 1000, 1));
            } else if (native) {
                qemu_cond_timedwait(&p->wake, &p->lock, 1);
            }
            qemu_mutex_unlock(&p->lock); continue;
        }
        int64_t pick_due = p->slots[pick].due ? p->slots[pick].due + phase_adj : 0;
        bool pick_waited = waited_for_due && pick_due;
        waited_for_due = false;
        SpSlot *s = &p->slots[pick];
        s->state = SP_READING;
        if (s->ready) { glDeleteSync(s->ready); s->ready = NULL; }
        for (int i = 0; i < SP_SLOTS; i++) {
            if (i != pick && p->slots[i].state == SP_READY &&
                p->slots[i].sequence < s->sequence) {
                p->slots[i].state = SP_FREE;
                p->discarded++;
            }
        }
        qemu_mutex_unlock(&p->lock);
        /* Only this thread touches the window's default framebuffer. */
        glBindFramebuffer(GL_READ_FRAMEBUFFER, readfb);
        glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, s->fb.texture, 0);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
        glDrawBuffer(GL_BACK);
        glViewport(0, 0, s->fb.width, s->fb.height);
        glDisable(GL_SCISSOR_TEST);
        glBlitFramebuffer(0, 0, s->fb.width, s->fb.height,
                          0, 0, s->fb.width, s->fb.height,
                          GL_COLOR_BUFFER_BIT, GL_NEAREST);
        spacebox_capture_client(s->fb.width, s->fb.height);
        uint8_t probe_px[4] = { 0 };
        if (probe) {
            /* test aid: the colour at the centre of each frame shown */
            glBindFramebuffer(GL_READ_FRAMEBUFFER, readfb);
            glReadPixels(s->fb.width / 2, s->fb.height / 2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, probe_px);
        }
        spacebox_swap(p->window, s->fb.width, s->fb.height);
        if (probe) {
            fprintf(probe, "%lld %u %u %u\n", (long long)g_get_monotonic_time(), probe_px[0], probe_px[1], probe_px[2]);
        }
        GLsync consumed = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
        glFlush();
        if (pick_waited) {
            /*
             * The swap returned at a display refresh. Keep the frames' times in
             * the middle between two refreshes, so that small timing noise does
             * not move a frame to the refresh before or after.
             */
            int64_t x = g_get_monotonic_time() - pick_due;
            if (x >= 0 && x < 2 * refresh_us) {
                phase_adj += (x - (int64_t)(refresh_us * phase_target)) / 16;
                phase_adj = MAX(-refresh_us, MIN(phase_adj, refresh_us));
            }
        }
        {
            /* How long each guest frame stayed on screen, in display refreshes. */
            int64_t t = g_get_monotonic_time();
            if (stats_us && last_swap) {
                int64_t dt = t - last_swap;
                int ticks = (int)((dt + refresh_us / 2) / refresh_us);
                ticks = ticks < 1 ? 1 : ticks > 6 ? 6 : ticks;
                held[ticks]++;
                if (dt > held_max) held_max = dt;
            }
            last_swap = t;
        }
        qemu_mutex_lock(&p->lock);
        if (shown >= 0) p->slots[shown].state = SP_FREE;
        shown = pick;
        s->consumed = consumed;
        s->state = SP_DISPLAYED;
        p->presented++;
        int64_t now = g_get_monotonic_time();
        if (stats_us && now - last_stats > stats_us) {
            fprintf(stderr, "[SPACEBOX-PRESENT-STATS] us=%lld submitted=%llu presented=%llu coalesced=%llu busy=%llu"
                    " refresh_us=%lld held1=%llu held2=%llu held3=%llu held4=%llu held5=%llu held6plus=%llu held_max_ms=%.1f\n",
                    (long long)now, (unsigned long long)p->sequence,
                    (unsigned long long)p->presented, (unsigned long long)p->discarded,
                    (unsigned long long)p->busy, (long long)refresh_us,
                    (unsigned long long)held[1], (unsigned long long)held[2],
                    (unsigned long long)held[3], (unsigned long long)held[4],
                    (unsigned long long)held[5], (unsigned long long)held[6],
                    held_max / 1000.0);
            memset(held, 0, sizeof(held));
            held_max = 0;
            last_stats = now;
        }
        qemu_mutex_unlock(&p->lock);
    }
    glFinish();
    glDeleteFramebuffers(1, &readfb);
    SDL_GL_MakeCurrent(p->window, NULL);
    return NULL;
}

void sdl2_gl_present_init(struct sdl2_console *scon)
{
    const char *mode = getenv("SPACEBOX_PRESENT_MODE");
    if (!mode || strcmp(mode, "thread")) return;
    SDL_GL_SetAttribute(SDL_GL_SHARE_WITH_CURRENT_CONTEXT, 1);
    scon->renderctx = SDL_GL_CreateContext(scon->real_window);
    if (!scon->renderctx) {
        fprintf(stderr, "[SPACEBOX-PRESENT-ERROR] producer context: %s\n", SDL_GetError());
        SDL_GL_MakeCurrent(scon->real_window, scon->winctx);
        return;
    }
    SDL_GL_SetSwapInterval(0);
    SpPresenter *p = g_new0(SpPresenter, 1);
    p->window = scon->real_window;
    p->output = scon->winctx;
    qemu_mutex_init(&p->lock);
    qemu_cond_init(&p->wake);
    CVReturn rc = CVDisplayLinkCreateWithCGDisplay(CGMainDisplayID(), &p->link);
    if (rc) {
        fprintf(stderr, "[SPACEBOX-PRESENT-ERROR] display link: %d\n", rc);
        SDL_GL_DeleteContext(scon->renderctx); scon->renderctx = NULL;
        SDL_GL_MakeCurrent(scon->real_window, scon->winctx);
        qemu_cond_destroy(&p->wake); qemu_mutex_destroy(&p->lock); g_free(p);
        return;
    }
    scon->presenter = p;
    CVDisplayLinkSetOutputCallback(p->link, sp_present_tick, p);
    qemu_thread_create(&p->thread, "sp-present", sp_present_thread, p, QEMU_THREAD_JOINABLE);
    CVDisplayLinkStart(p->link);
    fprintf(stderr, "[SPACEBOX-PRESENT] mode=thread slots=%d display=%u nonblocking_producer=1\n", SP_SLOTS, CGMainDisplayID());
}

static SpSlot *sp_present_acquire(struct sdl2_console *scon, int w, int h)
{
    SpPresenter *p = scon->presenter;
    SpSlot *pick = NULL;
    qemu_mutex_lock(&p->lock);
    for (int i = 0; i < SP_SLOTS; i++) {
        SpSlot *s = &p->slots[i];
        if (s->state == SP_FREE && sp_fence_done(s->consumed)) { pick = s; break; }
    }
    if (!pick) {
        for (int i = 0; i < SP_SLOTS; i++) {
            SpSlot *s = &p->slots[i];
            if (s->state == SP_READY && (!pick || s->sequence < pick->sequence)) pick = s;
        }
        if (pick) p->discarded++;
    }
    if (!pick) { p->busy++; qemu_mutex_unlock(&p->lock); return NULL; }
    pick->state = SP_WRITING;
    qemu_mutex_unlock(&p->lock);
    if (pick->ready) { glDeleteSync(pick->ready); pick->ready = NULL; }
    if (pick->consumed) { glDeleteSync(pick->consumed); pick->consumed = NULL; }
    if (pick->fb.width != w || pick->fb.height != h) {
        egl_fb_destroy(&pick->fb);
        egl_fb_setup_new_tex(&pick->fb, w, h);
    }
    return pick;
}

static void sp_present_submit(struct sdl2_console *scon, SpSlot *s)
{
    SpPresenter *p = scon->presenter;
    GLsync fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    glFlush();
    qemu_mutex_lock(&p->lock);
    s->ready = fence;
    s->sequence = ++p->sequence;
    s->submitted = g_get_monotonic_time();
    s->due = spacebox_frame_due_us;
    s->gap = spacebox_frame_gap_us;
    spacebox_frame_due_us = 0;
    spacebox_frame_gap_us = 0;
    s->state = SP_READY;
    qemu_cond_signal(&p->wake);
    qemu_mutex_unlock(&p->lock);
}

void sdl2_gl_present_destroy(struct sdl2_console *scon)
{
    SpPresenter *p = scon->presenter;
    if (!p) return;
    CVDisplayLinkStop(p->link);
    qemu_mutex_lock(&p->lock); p->stop = true; qemu_cond_signal(&p->wake); qemu_mutex_unlock(&p->lock);
    qemu_thread_join(&p->thread);
    SDL_GL_MakeCurrent(scon->real_window, scon->renderctx);
    glFinish();
    for (int i = 0; i < SP_SLOTS; i++) {
        if (p->slots[i].ready) glDeleteSync(p->slots[i].ready);
        if (p->slots[i].consumed) glDeleteSync(p->slots[i].consumed);
        egl_fb_destroy(&p->slots[i].fb);
    }
    CVDisplayLinkRelease(p->link);
    SDL_GL_MakeCurrent(scon->real_window, scon->winctx);
    SDL_GL_DeleteContext(scon->renderctx); scon->renderctx = NULL;
    qemu_cond_destroy(&p->wake); qemu_mutex_destroy(&p->lock);
    g_free(p); scon->presenter = NULL;
}
#else
void sdl2_gl_present_init(struct sdl2_console *scon) {}
void sdl2_gl_present_destroy(struct sdl2_console *scon) {}
#endif
