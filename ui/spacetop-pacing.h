#ifdef __aarch64__
#include <arm_acle.h>
__attribute__((target("crc")))
static uint64_t spacetop_crc_pixels(const unsigned char *p, size_t length)
{
    uint32_t a = 0, b = 0;
    for (size_t i = 0; i < length; i += 16) {
        uint64_t x, y;
        memcpy(&x, p + i, 8);
        memcpy(&y, p + i + 8, 8);
        a = __crc32cd(a, x & UINT64_C(0x00ffffff00ffffff));
        b = __crc32cd(b, y & UINT64_C(0x00ffffff00ffffff));
    }
    return ((uint64_t)a << 32) | b;
}
#else
static uint64_t spacetop_crc_pixels(const unsigned char *p, size_t length)
{
    uint64_t hash = UINT64_C(14695981039346656037);
    for (size_t i = 0; i < length; i++) { hash ^= p[i]; hash *= UINT64_C(1099511628211); }
    return hash;
}
#endif

/* Opt-in observations of this application's GL back buffer, never the desktop.
 * Each marker is black, white, then 12 little-endian bits (bench.html at DPR 2).
 * Readback is limited to one 448-pixel row. Trace disabled without both paths. */
static void spacetop_wait_frame(int fps)
{
    static int previous_fps;
    static int64_t next;
    if (fps < 30 || fps > 120) { next = 0; previous_fps = fps; return; }
    int64_t now = g_get_monotonic_time();
    int64_t period = 1000000 / fps;
    if (!next || previous_fps != fps || now > next + period) next = now;
    else next += period;
    previous_fps = fps;
    int64_t lead = SDL_GL_GetSwapInterval() > 0 ? 2000 : 0;
    int64_t wait = next - lead - now;
    if (wait > 0 && wait < 40000) g_usleep(wait);
}

static void spacetop_swap(SDL_Window *window, int width, int height)
{
    const char *path = getenv("SPACETOP_TRACE_FILE");
    const char *enable = getenv("SPACETOP_TRACE_ENABLE");
    const char *control = getenv("SPACETOP_SYNC_CONTROL");
    static FILE *trace;
    static int fps = -1;
    if (fps < 0) {
        const char *value = getenv("SPACETOP_FRAME_HZ");
        fps = value ? atoi(value) : 0;
    }
    static int64_t checked;
    int64_t now = g_get_monotonic_time();
    if (control && now - checked > 500000) {
        FILE *f = fopen(control, "r");
        int interval, new_fps = 0;
        checked = now;
        if (f) {
            int fields = fscanf(f, "%d %d", &interval, &new_fps);
            if (fields >= 1 && (new_fps == 0 || (new_fps >= 30 && new_fps <= 120))) fps = new_fps;
            if (fields >= 1 && interval >= 0 && interval <= 2 &&
                interval != SDL_GL_GetSwapInterval()) {
                int rc = SDL_GL_SetSwapInterval(interval);
                fprintf(stderr, "[SPACETOP-SYNC-CHANGE] requested=%d actual=%d rc=%d\n",
                        interval, SDL_GL_GetSwapInterval(), rc);
            }
            fclose(f);
        }
    }
    if (!path) {
        spacetop_wait_frame(fps);
        SDL_GL_SwapWindow(window);
        return;
    }
    bool pixel_trace = enable && !access(enable, F_OK);
    const char *full_path = getenv("SPACETOP_FULL_HASH_ENABLE");
    int full_hash = pixel_trace ? (full_path && !access(full_path, F_OK)) : 2;
    int marker = -1;
    uint64_t content_hash = UINT64_C(14695981039346656037);
    GLenum error = GL_NO_ERROR;
    if (pixel_trace && width == 2560 && height == 1600) {
        GLint fb, buffer, alignment, row, pack;
        unsigned char pixels[448 * 3];
        GLenum prior_error = glGetError();
        static int warned;
        if (prior_error && warned++ < 4) fprintf(stderr, "[SPACETOP-GL-PRIOR] error=0x%x\n", prior_error);
        glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &fb);
        glGetIntegerv(GL_READ_BUFFER, &buffer);
        glGetIntegerv(GL_PACK_ALIGNMENT, &alignment);
        glGetIntegerv(GL_PACK_ROW_LENGTH, &row);
        glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &pack);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
        glReadBuffer(GL_BACK);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glPixelStorei(GL_PACK_ROW_LENGTH, 0);
        glReadPixels(0, height - 16, 448, 1, GL_RGB, GL_UNSIGNED_BYTE, pixels);
        error = glGetError();
        int bits[14], valid = error == GL_NO_ERROR;
        for (int i = 0; i < 14; i++) {
            int x = (i * 32 + 16) * 3;
            int lo = MIN(pixels[x], MIN(pixels[x+1], pixels[x+2]));
            int hi = MAX(pixels[x], MAX(pixels[x+1], pixels[x+2]));
            if (hi < 48) bits[i] = 0;
            else if (lo > 207) bits[i] = 1;
            else { valid = 0; bits[i] = 0; }
        }
        if (valid && bits[0] == 0 && bits[1] == 1) {
            marker = 0;
            for (int i = 2; i < 14; i++) marker |= bits[i] << (i - 2);
        }
        /* Three video-interior rows: independent pixel-change evidence.
         * This excludes the marker in the top margin and bottom controls. */
        unsigned char content[2560 * 3];
        for (int row_index = 1; row_index <= 3; row_index++) {
            glReadPixels(0, height * row_index / 4, width, 1, GL_RGB,
                         GL_UNSIGNED_BYTE, content);
            for (int x = 0; x < width * 3; x++) {
                content_hash ^= content[x];
                content_hash *= UINT64_C(1099511628211);
            }
        }
        if (full_hash) {
            static unsigned char *full_pixels;
            if (!full_pixels) full_pixels = g_malloc((size_t)2560 * 1600 * 4);
            glReadPixels(0, 0, width, height, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, full_pixels);
            content_hash = spacetop_crc_pixels(full_pixels, (size_t)width * height * 4);
        }
        GLenum hash_error = glGetError();
        if (hash_error) error = hash_error;
        glBindFramebuffer(GL_READ_FRAMEBUFFER, fb);
        glReadBuffer(buffer);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, pack);
        glPixelStorei(GL_PACK_ALIGNMENT, alignment);
        glPixelStorei(GL_PACK_ROW_LENGTH, row);
    }
    int64_t before = g_get_monotonic_time();
    spacetop_wait_frame(fps);
    SDL_GL_SwapWindow(window);
    int64_t after = g_get_monotonic_time();
    if (!trace) {
        trace = fopen(path, "a");
        if (trace) setvbuf(trace, NULL, _IOLBF, 0);
    }
    if (trace) fprintf(trace, "%lld,%lld,%lld,%d,%d,%d,%d,%u,%016llx,%d\n",
                       (long long)after, (long long)before, (long long)now,
                       marker, width, height, SDL_GL_GetSwapInterval(), error,
                       (unsigned long long)content_hash, full_hash);
}
