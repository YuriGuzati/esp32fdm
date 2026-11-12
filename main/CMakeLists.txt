set(SRCS
    "main.c"
    "wifi.c"
    "camera.c"
    "httpd.c"
    "usb_serial.cpp"
    "sdcard.c"
    "sdcard_httpd.c"
    "printer_comm.c"
    "gcode_scan.c"
    "host_print.c"
    "printer_comm_klipper.c"
    "obico_client.c"
    "rfc2217.c"
    "terminal.c"
    "logbuf.c"
    "cache.c"
    "bedlevel.c"
    "dns_server.c"
    "mdns_service.c"
    "ota.c"
    "job_history.c"
)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    # Ender Smart (Fase 4, v1): new web interface served at /app (single file, no build step)
    EMBED_FILES "web/app.html" "web/icon-192.png" "web/apple-touch-icon.png"
)
