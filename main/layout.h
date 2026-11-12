#pragma once

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sdcard.h"
#include "printer_backend.h"
#include "version.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Growable page buffer ---- */

typedef struct {
    char *data;
    int   len;
    int   cap;
} html_buf_t;

static inline void html_buf_init(html_buf_t *p)
{
    p->cap = 2048;
    p->data = (char *)malloc(p->cap);
    p->len = 0;
    if (p->data) p->data[0] = '\0';
}

static inline void html_buf_printf(html_buf_t *p, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

static inline void html_buf_printf(html_buf_t *p, const char *fmt, ...)
{
    if (!p->data) return;

    va_list ap;
    va_start(ap, fmt);
    int need = vsnprintf(p->data + p->len, p->cap - p->len, fmt, ap);
    va_end(ap);

    if (need >= p->cap - p->len) {
        /* Grow to fit */
        int new_cap = p->cap;
        while (new_cap - p->len <= need)
            new_cap *= 2;
        char *new_data = (char *)realloc(p->data, new_cap);
        if (!new_data) return;
        p->data = new_data;
        p->cap = new_cap;

        va_start(ap, fmt);
        need = vsnprintf(p->data + p->len, p->cap - p->len, fmt, ap);
        va_end(ap);
    }
    p->len += need;
}

static inline void html_buf_free(html_buf_t *p)
{
    free(p->data);
    p->data = NULL;
    p->len = p->cap = 0;
}

/* ---- Shared layout helpers ---- */

/**
 * Emit <!DOCTYPE html> through opening <body>, shared CSS, nav bar, and status bar placeholder.
 * current_path highlights the active nav link (e.g. "/", "/sd", "/camera/config").
 */
static inline void layout_html_begin(html_buf_t *p, const char *title, const char *current_path)
{
    #define NAV_LINK(path, label) \
        (strcmp(current_path, path) == 0 \
            ? "<b>" label "</b>" \
            : "<a href='" path "'>" label "</a>")

    const char *sd_nav = "";
    if (sdcard_is_mounted()) {
        sd_nav = strcmp(current_path, "/sd") == 0
            ? " <b>SD</b>"
            : " <a href='/sd'>SD</a>";
    }

    int is_klipper = (printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER);
    const char *terminal_nav = "";
    if (!is_klipper) {
        terminal_nav = strcmp(current_path, "/terminal") == 0
            ? " <b>Term</b>"
            : " <a href='/terminal'>Term</a>";
    }

    const char *utils_nav = "";
    if (!is_klipper) {
        utils_nav = (strcmp(current_path, "/utils") == 0 || strncmp(current_path, "/bedlevel", 9) == 0)
            ? " <b>Utils</b>"
            : " <a href='/utils'>Utils</a>";
    }

    html_buf_printf(p,
        "<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<link rel='icon' type='image/svg+xml' href='/favicon.svg'>"
        "<title>%s</title>"
        "<style>"
        "body{font-family:sans-serif;max-width:600px;margin:0 auto;padding:0 1em}"
        "nav{background:#333;margin:0 -1em;padding:8px 1em;color:#fff;font-size:14px}"
        "nav a{color:#fff;text-decoration:none;margin:0 6px}"
        "nav b{color:#fff;margin:0 6px}"
        "nav .brand{font-weight:bold;margin-right:12px}"
        ".cpu-bars{display:inline-flex;align-items:center;float:right;gap:4px;font-size:11px;color:#aaa}"
        ".cpu-bar{width:40px;height:10px;background:#555;border-radius:2px;overflow:hidden;display:inline-block;vertical-align:middle}"
        ".cpu-bar>div{height:100%%;background:#4caf50;transition:width 1s}"
        "#pst{padding:8px 0;border-bottom:1px solid #ddd;font-size:14px;color:#666}"
        "#pst.printing{background:#e8f5e9;color:#2e7d32;padding:8px;border-radius:4px;margin:8px 0}"
        "#pst.paused{background:#fff3e0;color:#e65100;padding:8px;border-radius:4px;margin:8px 0}"
        "#pst.error{background:#ffebee;color:#c62828;padding:8px;border-radius:4px;margin:8px 0}"
        ".pbar{background:#ddd;border-radius:4px;height:16px;margin:4px 0;overflow:hidden}"
        ".pbar>div{background:#4caf50;height:100%%;transition:width .3s}"
        "#pst .pctl{float:right;margin-top:-2px}"
        "#pst .pctl button{padding:2px 10px;font-size:12px;margin-left:4px;vertical-align:middle}"
        "table{width:100%%;border-collapse:collapse}"
        "th,td{text-align:left;padding:6px 8px;border-bottom:1px solid #ddd}"
        "button,input[type=submit]{padding:6px 16px;font-size:14px;cursor:pointer;border-radius:4px;border:1px solid #ccc;background:#f5f5f5}"
        "button:hover{background:#e8e8e8}"
        ".btn-red{background:#f44336;color:#fff;border-color:#d32f2f}"
        ".btn-red:hover{background:#d32f2f}"
        ".btn-green{background:#4CAF50;color:#fff;border-color:#388E3C}"
        ".btn-green:hover{background:#388E3C}"
        ".sh{display:flex;align-items:center;justify-content:space-between}"
        ".sh h2{margin:0}"
        ".hint{font-size:13px;color:#666}"
        "</style></head><body>"
        "<nav>"
        "<span class='brand'>ESP32FDM</span>"
        "<a href='/app' style='color:#ffb366;font-weight:bold'>Nova interface</a> "
        "%s %s%s%s%s %s %s"
        "<span class='cpu-bars'>"
        "C0<span class='cpu-bar'><div id='cb0'></div></span>"
        "C1<span class='cpu-bar'><div id='cb1'></div></span>"
        "</span>"
        "</nav>"
        "<div id='pst'></div>",
        title,
        NAV_LINK("/", "Home"),
        NAV_LINK("/camera", "Cam"),
        sd_nav,
        terminal_nav,
        utils_nav,
        NAV_LINK("/logs", "Logs"),
        NAV_LINK("/settings", "Settings"));

    #undef NAV_LINK
}

/**
 * Emit status-bar polling JS, footer, and closing tags.
 */
static inline void layout_html_end(html_buf_t *p)
{
    html_buf_printf(p,
        "<script>"
        "!function(){var p=document.getElementById('pst');"
        "function u(){fetch('/api/status').then(function(r){return r.json()}).then(function(s){"
        "var a=s.state=='printing'||s.state=='paused';"
        "p.className=a?s.state:s.state=='error'?'error':'';"
        "if(a){var bt='<span class=pctl>';"
        "if(s.state=='printing')bt+='<button onclick=\"fetch(\\'/sd/pause\\',{method:\\'POST\\'})\">Pause</button>';"
        "else bt+='<button class=btn-green onclick=\"fetch(\\'/sd/resume\\',{method:\\'POST\\'})\">Resume</button>';"
        "bt+='<button class=btn-red onclick=\"if(confirm(\\'Stop print?\\'))fetch(\\'/sd/cancel\\',{method:\\'POST\\'})\">Stop</button></span>';"
        "p.innerHTML=bt+'<b>'+(s.state=='paused'?'PAUSED':'PRINTING')+':</b> '+(s.filename||'')"
        "+'<div class=pbar><div style=\"width:'+Math.min(100,s.progress||0)+'%%\"></div></div>'"
        "+~~s.progress+'%%'+(s.layer&&s.layer!='0/0'?' &middot; Layer '+s.layer:'')"
        "+'<div style=\"font-size:12px;color:#555;margin-top:2px\">'+"
        "'HE '+s.hotend[0].toFixed(0)+'/'+s.hotend[1].toFixed(0)+'\\u00b0C'"
        "+' &middot; Bed '+s.bed[0].toFixed(0)+'/'+s.bed[1].toFixed(0)+'\\u00b0C</div>';}"
        "else{var t='HE '+s.hotend[0].toFixed(0)+'/'+s.hotend[1].toFixed(0)+'\\u00b0C'"
        "+' &middot; Bed '+s.bed[0].toFixed(0)+'/'+s.bed[1].toFixed(0)+'\\u00b0C';"
        "p.innerHTML=(s.state=='error'?'<span class=pctl><button onclick=\"fetch(\\'/sd/cancel\\',{method:\\'POST\\'})\">OK</button></span>'+(s.error?s.error:s.backend+': Error'):s.usb?s.backend+': Ready':s.backend+': Disconnected')"
        "+'<div style=\"font-size:12px;color:#888\">'+t+'</div>';}"
        "}).catch(function(){})}"
        "u();setInterval(u,3000);"
        "function cs(){fetch('/api/stats').then(function(r){return r.json()}).then(function(d){"
        "var b0=document.getElementById('cb0'),b1=document.getElementById('cb1');"
        "if(b0)b0.style.width=d.cpu0+'%%';"
        "if(b1)b1.style.width=d.cpu1+'%%';"
        "var c=function(v){return v>80?'#f44336':v>50?'#ff9800':'#4caf50'};"
        "if(b0)b0.style.background=c(d.cpu0);"
        "if(b1)b1.style.background=c(d.cpu1);"
        "}).catch(function(){})}"
        "cs();setInterval(cs,3000)}()"
        "</script>"
        "<footer style='margin-top:32px;padding:12px 0;border-top:1px solid #ddd;text-align:center;font-size:12px;color:#999'>"
        "&copy; Sergey Korotkov &amp; Claude 2026 &middot; v" FW_VERSION "</footer>"
        "</body></html>");
}

#ifdef __cplusplus
}
#endif
