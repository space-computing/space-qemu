#include "qemu/osdep.h"
#import <AppKit/AppKit.h>
#include <SDL.h>
#include <SDL_syswm.h>
#include <sys/socket.h>
#include <sys/un.h>
extern int64_t spacebox_last_input_us;

static NSWindow *sp_window;
static id sp_monitor;
static SDL_Window *sp_sdl;
static FILE *sp_trace;
static uint64_t sp_event_id;
static bool sp_initialized;
static void sp_log(NSEvent *event, const char *origin)
{
    const char *path = getenv("SPACEBOX_SCROLL_TRACE");
    if (!path) return;
    if (!sp_trace) { sp_trace = fopen(path, "a"); if (sp_trace) setvbuf(sp_trace,NULL,_IOLBF,0); }
    if (sp_trace) fprintf(sp_trace,"{\"stage\":\"native\",\"id\":%llu,\"host_us\":%lld,\"origin\":\"%s\",\"x\":%.9g,\"y\":%.9g,\"delta_x\":%.9g,\"delta_y\":%.9g,\"precise\":%d,\"phase\":%lu,\"momentum\":%lu,\"inverted\":%d}\n",(unsigned long long)++sp_event_id,(long long)g_get_monotonic_time(),origin,[event scrollingDeltaX],[event scrollingDeltaY],[event deltaX],[event deltaY],[event hasPreciseScrollingDeltas],(unsigned long)[event phase],(unsigned long)[event momentumPhase],[event isDirectionInvertedFromDevice]);
}
static int sp_fd=-1;
static bool sp_ready;
static char sp_pending[65536];
static size_t sp_pending_len;
static int64_t sp_retry, sp_ping, sp_last_ready;
static uint64_t sp_sequence;
static void sp_bridge_poll(void)
{
    const char *path=getenv("SPACEBOX_SCROLL_SOCKET");
    if (!path) return;
    if (sp_fd<0 && g_get_monotonic_time()>=sp_retry) {
        sp_retry=g_get_monotonic_time()+250000;
        struct sockaddr_un addr={.sun_family=AF_UNIX};
        if (strlen(path)>=sizeof(addr.sun_path)) return;
        strcpy(addr.sun_path,path);
        sp_fd=socket(AF_UNIX,SOCK_STREAM,0);
        if(sp_fd<0)return;
        int yes=1;setsockopt(sp_fd,SOL_SOCKET,SO_NOSIGPIPE,&yes,sizeof(yes));
        fcntl(sp_fd,F_SETFL,O_NONBLOCK);
        if(connect(sp_fd,(struct sockaddr *)&addr,sizeof(addr))<0) { close(sp_fd);sp_fd=-1;return; }
        send(sp_fd,"PING\n",5,0);
    }
    if(sp_fd<0)return;
    char reply[128];ssize_t n=recv(sp_fd,reply,sizeof(reply),0);
    if(n>0) {
        if(!sp_ready) fprintf(stderr,"[SPACEBOX-SCROLL] bridge_ready=1\n");
        sp_ready=true;sp_last_ready=g_get_monotonic_time();
    }
    else if(n==0 || (n<0 && errno!=EAGAIN && errno!=EWOULDBLOCK)) { close(sp_fd);sp_fd=-1;sp_ready=false;sp_pending_len=0;return; }
    int64_t now=g_get_monotonic_time();
    if(now-sp_ping>1000000) { send(sp_fd,"PING\n",5,0);sp_ping=now; }
    if(sp_ready && now-sp_last_ready>3000000) { sp_ready=false;fprintf(stderr,"[SPACEBOX-SCROLL] bridge_ready=0 timeout=1\n"); }
    if(sp_pending_len) {
        ssize_t wrote=send(sp_fd,sp_pending,sp_pending_len,0);
        if(wrote>0) { memmove(sp_pending,sp_pending+wrote,sp_pending_len-wrote);sp_pending_len-=wrote; }
        else if(wrote<0 && errno!=EAGAIN && errno!=EWOULDBLOCK) { close(sp_fd);sp_fd=-1;sp_ready=false;sp_pending_len=0; }
    }
}
static bool sp_route(NSEvent *event, const char *origin)
{
    sp_log(event,origin);
    const char *bypass=getenv("SPACEBOX_SCROLL_BYPASS");
    if(bypass && !access(bypass,F_OK))return false;
    if(!getenv("SPACEBOX_SCROLL_SOCKET"))return false;
    sp_bridge_poll();
    if(!sp_ready)return false;
    bool precise=[event hasPreciseScrollingDeltas];
    /* Match native point deltas. macOS already applied natural direction.
     * Cocoa discrete delta uses Chromium's 40 points per Cocoa tick. */
    double x=precise ? -[event scrollingDeltaX] : -40.0*[event deltaX];
    double y=precise ? -[event scrollingDeltaY] : -40.0*[event deltaY];
    spacebox_last_input_us=g_get_monotonic_time(); /* hw/display/virtio-gpu-remote.c */
    char line[192];int n=snprintf(line,sizeof(line),"%llu %lld %.9g %.9g %d %lu %lu\n",(unsigned long long)++sp_sequence,(long long)g_get_monotonic_time(),x,y,precise,(unsigned long)[event phase],(unsigned long)[event momentumPhase]);
    if(n>0 && n<sizeof(line) && sp_pending_len+n<=sizeof(sp_pending)) {
        memcpy(sp_pending+sp_pending_len,line,n);sp_pending_len+=n;sp_bridge_poll();
    } else fprintf(stderr,"[SPACEBOX-SCROLL-ERROR] bounded queue overflow\n");
    return true; /* Exactly one path; suppress SDL's quantized wheel. */
}

/* In-process event fixture. Never posted to a system event tap. */
@interface SpScrollFixture : NSEvent {
@public double sx,sy; BOOL precise,inverted; NSUInteger ph,mp; NSWindow *win;
}
@end
@implementation SpScrollFixture
- (NSEventType)type { return NSEventTypeScrollWheel; }
- (CGFloat)scrollingDeltaX { return sx; }
- (CGFloat)scrollingDeltaY { return sy; }
- (CGFloat)deltaX { return precise ? sx/10.0 : sx; }
- (CGFloat)deltaY { return precise ? sy/10.0 : sy; }
- (BOOL)hasPreciseScrollingDeltas { return precise; }
- (BOOL)isDirectionInvertedFromDevice { return inverted; }
- (NSEventPhase)phase { return ph; }
- (NSEventPhase)momentumPhase { return mp; }
- (NSTimeInterval)timestamp { return [[NSProcessInfo processInfo] systemUptime]; }
- (NSWindow *)window { return win; }
- (NSInteger)windowNumber { return [win windowNumber]; }
- (NSPoint)locationInWindow { NSRect r=[[win contentView] bounds]; return NSMakePoint(r.size.width/2,r.size.height/2); }
- (NSEventModifierFlags)modifierFlags { return 0; }
@end

void spacebox_scroll_tick(SDL_Window *window)
{
    if (!window) return;
    @autoreleasepool {
        if (!sp_initialized) {
            SDL_SysWMinfo info; SDL_VERSION(&info.version);
            if (!SDL_GetWindowWMInfo(window,&info)) return;
            sp_window=info.info.cocoa.window; sp_sdl=window; sp_initialized=true;
            sp_monitor=[NSEvent addLocalMonitorForEventsMatchingMask:NSEventMaskScrollWheel handler:^NSEvent *(NSEvent *e) {
                if ([e window]!=sp_window) return e;
                return sp_route(e,"device") ? nil : e;
            }];
            fprintf(stderr,"[SPACEBOX-SCROLL] local_monitor=1 no_system_event_tap=1\n");
        }
        sp_bridge_poll();
        const char *path=getenv("SPACEBOX_SCROLL_TEST");
        if (!path) return;
        FILE *f=fopen(path,"r"); if (!f) return;
        double x=0,y=0; int precise=1,phase=4,momentum=0,inverted=1;
        int n=fscanf(f,"%lf %lf %d %d %d %d",&x,&y,&precise,&phase,&momentum,&inverted);
        fclose(f); unlink(path); if(n<5)return;
        SpScrollFixture *event=[[SpScrollFixture alloc] init];
        event->sx=x;event->sy=y;event->precise=precise;event->ph=phase;event->mp=momentum;event->inverted=inverted;event->win=sp_window;
        if (!sp_route(event,"fixture")) {
            /* SDL's actual Cocoa event listener is the window delegate. */
            id delegate=[sp_window delegate];
            if ([delegate respondsToSelector:@selector(scrollWheel:)]) [delegate scrollWheel:event];
            else [[sp_window contentView] scrollWheel:event];
        }
        [event release];
    }
}
