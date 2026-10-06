/* Opt-in full-image observation via bounded PBOs. Hash after swapping so CPU
 * readback does not delay that frame's submission. No glFinish/wait timeout. */
#define SP_ASYNC_SLOTS 4
typedef struct SpAsyncHash {
    GLuint pbo;
    GLsync fence;
    int64_t before, after, start;
    uint64_t expected;
    bool validate;
    int interval;
    GLenum error;
} SpAsyncHash;
static void spacetop_async_swap(SDL_Window *window, int width, int height,
                               FILE *trace, int fps)
{
    static SpAsyncHash slots[SP_ASYNC_SLOTS];
    static unsigned head, tail, pending, validated, failures, overflow;
    static bool init;
    static uint64_t hashes, hash_us_total, hash_us_max;
    size_t bytes = (size_t)width * height * 4;
    bool post_swap = getenv("SPACETOP_HASH_AFTER_SWAP") != NULL;
    GLint fb, buffer, alignment, row, pack;
    glGetError();
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &fb);
    glGetIntegerv(GL_READ_BUFFER, &buffer);
    glGetIntegerv(GL_PACK_ALIGNMENT, &alignment);
    glGetIntegerv(GL_PACK_ROW_LENGTH, &row);
    glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &pack);
    if (!init) {
        for (int i = 0; i < SP_ASYNC_SLOTS; i++) {
            glGenBuffers(1, &slots[i].pbo);
            glBindBuffer(GL_PIXEL_PACK_BUFFER, slots[i].pbo);
            glBufferData(GL_PIXEL_PACK_BUFFER, bytes, NULL, GL_STREAM_READ);
        }
        init = true;
        fprintf(stderr, "[SPACETOP-ASYNC-HASH] slots=%d bytes=%zu\n", SP_ASYNC_SLOTS, bytes * SP_ASYNC_SLOTS);
    }
    SpAsyncHash *s = pending < SP_ASYNC_SLOTS ? &slots[head] : NULL;
    int64_t start = g_get_monotonic_time();
    if (s) {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
        glReadBuffer(GL_BACK);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glPixelStorei(GL_PACK_ROW_LENGTH, 0);
        if (!post_swap) {
            glBindBuffer(GL_PIXEL_PACK_BUFFER, s->pbo);
            glReadPixels(0, 0, width, height, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, 0);
            s->error = glGetError();
            s->fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
        }
        s->validate = validated + pending < 4;
        if (s->validate) {
            unsigned char *check = g_malloc(bytes);
            glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
            glReadPixels(0, 0, width, height, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, check);
            s->expected = spacetop_crc_pixels(check, bytes);
            g_free(check);
        }
    } else {
        if (overflow++ < 4) fprintf(stderr, "[SPACETOP-ASYNC-OVERFLOW] frame_not_observed\n");
    }
    glBindFramebuffer(GL_READ_FRAMEBUFFER, fb);
    glReadBuffer(buffer);
    glPixelStorei(GL_PACK_ALIGNMENT, alignment);
    glPixelStorei(GL_PACK_ROW_LENGTH, row);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pack);
    int64_t before = g_get_monotonic_time();
    spacetop_wait_frame(fps);
    SDL_GL_SwapWindow(window);
    int64_t after = g_get_monotonic_time();
    if (s && post_swap) {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
        glReadBuffer(GL_FRONT);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glPixelStorei(GL_PACK_ROW_LENGTH, 0);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, s->pbo);
        glReadPixels(0, 0, width, height, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, 0);
        s->error = glGetError();
        s->fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
        glFlush();
        glBindFramebuffer(GL_READ_FRAMEBUFFER, fb);
        glReadBuffer(buffer);
        glPixelStorei(GL_PACK_ALIGNMENT, alignment);
        glPixelStorei(GL_PACK_ROW_LENGTH, row);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, pack);
    }
    if (s) {
        s->start = start; s->before = before; s->after = after;
        s->interval = SDL_GL_GetSwapInterval();
        head = (head + 1) % SP_ASYNC_SLOTS;
        pending++;
    }
    while (pending) {
        SpAsyncHash *old = &slots[tail];
        GLenum ready = glClientWaitSync(old->fence, 0, 0);
        if (ready != GL_ALREADY_SIGNALED && ready != GL_CONDITION_SATISFIED) break;
        glBindBuffer(GL_PIXEL_PACK_BUFFER, old->pbo);
        int64_t hash_start = g_get_monotonic_time();
        unsigned char *pixels = glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0, bytes, GL_MAP_READ_BIT);
        uint64_t hash = pixels ? spacetop_crc_pixels(pixels, bytes) : 0;
        if (pixels) glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
        else old->error = GL_INVALID_OPERATION;
        uint64_t cost = g_get_monotonic_time() - hash_start;
        hashes++; hash_us_total += cost; hash_us_max = MAX(hash_us_max, cost);
        if (hashes % 1024 == 0) fprintf(stderr, "[SPACETOP-ASYNC-STATS] frames=%llu mean_hash_us=%llu max_hash_us=%llu overflow=%u\n", (unsigned long long)hashes, (unsigned long long)(hash_us_total/hashes), (unsigned long long)hash_us_max, overflow);
        if (old->validate) {
            validated++;
            if (hash != old->expected) { failures++; old->error = GL_INVALID_OPERATION; }
            fprintf(stderr, "[SPACETOP-ASYNC-VALIDATE] sample=%u matched=%d failures=%u expected=%016llx actual=%016llx\n", validated, hash == old->expected, failures, (unsigned long long)old->expected, (unsigned long long)hash);
        }
        fprintf(trace, "%lld,%lld,%lld,-1,%d,%d,%d,%u,%016llx,1\n",
                (long long)old->after, (long long)old->before, (long long)old->start,
                width, height, old->interval, old->error, (unsigned long long)hash);
        glDeleteSync(old->fence); old->fence = NULL;
        tail = (tail + 1) % SP_ASYNC_SLOTS; pending--;
    }
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pack);
}
