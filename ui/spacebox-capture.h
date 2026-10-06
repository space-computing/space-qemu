/* Optional measurement hook. Captures this client's GL back buffer only,
 * never the macOS desktop or other applications. Disabled without env paths. */
static void spacebox_capture_client(int width, int height)
{
    static int64_t last_check;
    const char *request = getenv("SPACEBOX_CAPTURE_REQUEST");
    const char *output = getenv("SPACEBOX_CAPTURE_PPM");
    int64_t now;
    GLint read_fb, read_buffer, alignment, row_length, pack_buffer;
    GLenum before, after;
    unsigned char *pixels;
    FILE *f;
    if (!request || !output) {
        return;
    }
    now = g_get_monotonic_time();
    if (now - last_check < 1000000) {
        return;
    }
    last_check = now;
    if (access(request, F_OK)) {
        return;
    }
    unlink(request);
    if (width <= 0 || height <= 0 || width > 8192 || height > 8192) {
        return;
    }
    before = glGetError();
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &read_fb);
    glGetIntegerv(GL_READ_BUFFER, &read_buffer);
    glGetIntegerv(GL_PACK_ALIGNMENT, &alignment);
    glGetIntegerv(GL_PACK_ROW_LENGTH, &row_length);
    glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &pack_buffer);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glReadBuffer(GL_BACK);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    pixels = g_malloc((size_t)width * height * 3);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);
    after = glGetError();
    glBindFramebuffer(GL_READ_FRAMEBUFFER, read_fb);
    glReadBuffer(read_buffer);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pack_buffer);
    glPixelStorei(GL_PACK_ALIGNMENT, alignment);
    glPixelStorei(GL_PACK_ROW_LENGTH, row_length);
    f = after == GL_NO_ERROR ? fopen(output, "wb") : NULL;
    if (f) {
        fprintf(f, "P6\n%d %d\n255\n", width, height);
        for (int y = height - 1; y >= 0; y--) {
            fwrite(pixels + (size_t)y * width * 3, 1, (size_t)width * 3, f);
        }
        fclose(f);
    }
    g_free(pixels);
    fprintf(stderr, "[SPACEBOX-CAPTURE] size=%dx%d gl_before=0x%x gl_read=0x%x saved=%d\n",
            width, height, before, after, !!f);
}
