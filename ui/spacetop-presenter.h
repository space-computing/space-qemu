/* Experimental Mac presenter: private bounded snapshots, shared GL contexts.
 * Guest processing never waits for display refresh or a consumer GL fence.
 * The display-link callback only signals a condition variable. */
#ifdef CONFIG_DARWIN
#include <CoreVideo/CoreVideo.h>
#include <CoreGraphics/CoreGraphics.h>
#include <pthread/qos.h>
#include "qemu/thread.h"
#define SP_SLOTS 4
enum SpSlotState { SP_FREE, SP_WRITING, SP_READY, SP_READING, SP_DISPLAYED };
typedef struct SpSlot {
    egl_fb fb;
    GLsync ready, consumed;
    enum SpSlotState state;
    uint64_t sequence;
    int64_t submitted;
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
    const char *scheduler = getenv("SPACETOP_PRESENT_SCHEDULER");
    bool native = scheduler && !strcmp(scheduler, "native");
    const char *phase_env = getenv("SPACETOP_PRESENT_PHASE_US");
    int phase_us = phase_env ? atoi(phase_env) : 0;
    const char *phase_file = getenv("SPACETOP_PRESENT_PHASE_CONTROL");
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INITIATED, 0);
    if (SDL_GL_MakeCurrent(p->window, p->output)) {
        fprintf(stderr, "[SPACETOP-PRESENT-ERROR] make_current: %s\n", SDL_GetError());
        return NULL;
    }
    const char *interval = getenv("SPACETOP_PRESENT_INTERVAL");
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
        int pick = -1;
        for (int i = 0; i < SP_SLOTS; i++) {
            SpSlot *s = &p->slots[i];
            if (s->state == SP_READY && sp_fence_done(s->ready) &&
                (pick < 0 || s->sequence > p->slots[pick].sequence)) pick = i;
        }
        if (pick < 0) {
            if (native) qemu_cond_timedwait(&p->wake, &p->lock, 1);
            qemu_mutex_unlock(&p->lock); continue;
        }
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
        spacetop_capture_client(s->fb.width, s->fb.height);
        spacetop_swap(p->window, s->fb.width, s->fb.height);
        GLsync consumed = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
        glFlush();
        qemu_mutex_lock(&p->lock);
        if (shown >= 0) p->slots[shown].state = SP_FREE;
        shown = pick;
        s->consumed = consumed;
        s->state = SP_DISPLAYED;
        p->presented++;
        int64_t now = g_get_monotonic_time();
        if (getenv("SPACETOP_PRESENT_STATS") && now - last_stats > 10000000) {
            uint64_t bytes = 0;
            for (int i = 0; i < SP_SLOTS; i++) {
                if (p->slots[i].state != SP_WRITING) bytes += (uint64_t)p->slots[i].fb.width * p->slots[i].fb.height * 4;
            }
            fprintf(stderr, "[SPACETOP-PRESENT-STATS] us=%lld submitted=%llu presented=%llu coalesced=%llu busy=%llu textures=%d bytes=%llu\n",
                    (long long)now, (unsigned long long)p->sequence,
                    (unsigned long long)p->presented, (unsigned long long)p->discarded,
                    (unsigned long long)p->busy, SP_SLOTS, (unsigned long long)bytes);
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
    const char *mode = getenv("SPACETOP_PRESENT_MODE");
    if (!mode || strcmp(mode, "thread")) return;
    SDL_GL_SetAttribute(SDL_GL_SHARE_WITH_CURRENT_CONTEXT, 1);
    scon->renderctx = SDL_GL_CreateContext(scon->real_window);
    if (!scon->renderctx) {
        fprintf(stderr, "[SPACETOP-PRESENT-ERROR] producer context: %s\n", SDL_GetError());
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
        fprintf(stderr, "[SPACETOP-PRESENT-ERROR] display link: %d\n", rc);
        SDL_GL_DeleteContext(scon->renderctx); scon->renderctx = NULL;
        SDL_GL_MakeCurrent(scon->real_window, scon->winctx);
        qemu_cond_destroy(&p->wake); qemu_mutex_destroy(&p->lock); g_free(p);
        return;
    }
    scon->presenter = p;
    CVDisplayLinkSetOutputCallback(p->link, sp_present_tick, p);
    qemu_thread_create(&p->thread, "sp-present", sp_present_thread, p, QEMU_THREAD_JOINABLE);
    CVDisplayLinkStart(p->link);
    fprintf(stderr, "[SPACETOP-PRESENT] mode=thread slots=%d display=%u nonblocking_producer=1\n", SP_SLOTS, CGMainDisplayID());
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
