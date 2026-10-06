/* Launcher command for this application's own visible window. */
static void spacebox_window_request(struct sdl2_console *scon)
{
    const char *path = getenv("SPACEBOX_WINDOW_REQUEST");
    if (!path || !scon->real_window || access(path, F_OK)) return;
    unlink(path);
    SDL_RaiseWindow(scon->real_window);
    int w, h, pw, ph;
    SDL_GetWindowSize(scon->real_window, &w, &h);
    SDL_GL_GetDrawableSize(scon->real_window, &pw, &ph);
    fprintf(stderr, "[SPACEBOX-USER-WINDOW] logical=%dx%d drawable=%dx%d shown=%d minimized=%d mouse_grab=%d\n",
            w, h, pw, ph,
            !!(SDL_GetWindowFlags(scon->real_window) & SDL_WINDOW_SHOWN),
            !!(SDL_GetWindowFlags(scon->real_window) & SDL_WINDOW_MINIMIZED),
            SDL_GetWindowGrab(scon->real_window));
}

/* Synthetic events in this SDL application's queue. Opt-in local validation;
 * does not post events to macOS or inspect any other application. */
static void spacebox_input_test(struct sdl2_console *scon)
{
    const char *path = getenv("SPACEBOX_INPUT_TEST");
    if (!path || !scon->real_window) return;
    FILE *f = fopen(path, "r");
    if (!f) return;
    char command[24];
    int a = 0, b = 0;
    int count = fscanf(f, "%23s %d %d", command, &a, &b);
    fclose(f);
    unlink(path);
    if (count < 1) return;
    static int test_x, test_y;
    SDL_Event ev = {0};
    Uint32 id = SDL_GetWindowID(scon->real_window);
    if (!strcmp(command, "key") && count == 3) {
        ev.type = b ? SDL_KEYDOWN : SDL_KEYUP;
        ev.key.windowID = id;
        ev.key.state = b ? SDL_PRESSED : SDL_RELEASED;
        ev.key.keysym.scancode = a;
        ev.key.keysym.sym = SDL_GetKeyFromScancode(a);
        SDL_PushEvent(&ev);
    } else if (!strcmp(command, "motion") && count == 3) {
        ev.type = SDL_MOUSEMOTION;
        ev.motion.windowID = id;
        test_x=a;test_y=b;
        ev.motion.x = a;
        ev.motion.y = b;
        SDL_PushEvent(&ev);
    } else if (!strcmp(command, "button") && count == 3) {
        ev.type = b ? SDL_MOUSEBUTTONDOWN : SDL_MOUSEBUTTONUP;
        ev.button.windowID=id;ev.button.button=a;ev.button.state=b?SDL_PRESSED:SDL_RELEASED;
        ev.button.x=test_x;ev.button.y=test_y;ev.button.clicks=1;SDL_PushEvent(&ev);
    } else if (!strcmp(command, "resize") && count == 3 && a>=640 && b>=400) {
        SDL_SetWindowSize(scon->real_window,a,b);
    } else if (!strcmp(command, "focusgain")) {
        ev.type=SDL_WINDOWEVENT;ev.window.windowID=id;ev.window.event=SDL_WINDOWEVENT_FOCUS_GAINED;SDL_PushEvent(&ev);
    } else if (!strcmp(command, "focuslost")) {
        ev.type = SDL_WINDOWEVENT;
        ev.window.windowID = id;
        ev.window.event = SDL_WINDOWEVENT_FOCUS_LOST;
        SDL_PushEvent(&ev);
    } else if (!strcmp(command, "raise")) {
        SDL_RaiseWindow(scon->real_window);
    }
    int w, h, pw, ph;
    SDL_GetWindowSize(scon->real_window, &w, &h);
    SDL_GL_GetDrawableSize(scon->real_window, &pw, &ph);
    fprintf(stderr, "[SPACEBOX-INPUT-TEST] command=%s a=%d b=%d absolute=%d grab=%d mouse_grab=%d logical=%dx%d drawable=%dx%d\n",
            command, a, b, qemu_input_is_absolute(scon->dcl.con), gui_grab,
            SDL_GetWindowGrab(scon->real_window), w, h, pw, ph);
}
