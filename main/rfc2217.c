#include "rfc2217.h"
#include "printer_comm.h"

#include "usb_serial.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include <string.h>

static const char *TAG = "rfc2217";

/* ---- Telnet protocol constants ---- */

#define IAC   255
#define DONT  254
#define DO    253
#define WONT  252
#define WILL  251
#define SB    250  /* Subnegotiation Begin */
#define SE    240  /* Subnegotiation End */

/* COM-PORT-OPTION (RFC 2217) */
#define COMPORT_OPTION 44

/* Client-to-server subnegotiation commands */
#define CPO_SET_BAUDRATE     1
#define CPO_SET_DATASIZE     2
#define CPO_SET_PARITY       3
#define CPO_SET_STOPSIZE     4
#define CPO_SET_CONTROL      5
#define CPO_PURGE_DATA      12

/* Server-to-client subnegotiation responses (command + 100) */
#define CPO_RESP_OFFSET    100

/* SET-CONTROL values for DTR/RTS */
#define CONTROL_DTR_ON      8
#define CONTROL_DTR_OFF     9
#define CONTROL_RTS_ON     11
#define CONTROL_RTS_OFF    12

/* ---- State ---- */

typedef enum {
    TS_DATA,    /* Normal data */
    TS_IAC,     /* Received IAC */
    TS_WILL,    /* Received IAC WILL */
    TS_WONT,    /* Received IAC WONT */
    TS_DO,      /* Received IAC DO */
    TS_DONT,    /* Received IAC DONT */
    TS_SB,      /* Subnegotiation: waiting for option code */
    TS_SB_DATA, /* Subnegotiation: collecting data */
    TS_SB_IAC,  /* Subnegotiation: received IAC within SB */
} telnet_state_t;

static int s_client_sock = -1;
static StreamBufferHandle_t s_serial_rx_buf = NULL;
static bool s_dtr = true;
static bool s_rts = true;

/* Subnegotiation buffer */
#define SB_BUF_SIZE 32
static uint8_t s_sb_option;
static uint8_t s_sb_buf[SB_BUF_SIZE];
static size_t s_sb_len;

/* ---- Helpers ---- */

static void tcp_send(int sock, const uint8_t *data, size_t len)
{
    if (sock < 0) return;
    size_t sent = 0;
    while (sent < len) {
        int n = send(sock, data + sent, len - sent, 0);
        if (n <= 0) break;
        sent += n;
    }
}

static void send_iac_cmd(int sock, uint8_t cmd, uint8_t option)
{
    uint8_t buf[3] = { IAC, cmd, option };
    tcp_send(sock, buf, 3);
}

static void send_sb_response(int sock, uint8_t option, uint8_t cmd,
                              const uint8_t *data, size_t data_len)
{
    uint8_t hdr[4] = { IAC, SB, option, cmd };
    uint8_t tail[2] = { IAC, SE };
    tcp_send(sock, hdr, 4);
    if (data && data_len > 0) {
        tcp_send(sock, data, data_len);
    }
    tcp_send(sock, tail, 2);
}

/* ---- Subnegotiation handling ---- */

static void handle_subnegotiation(int sock)
{
    if (s_sb_option != COMPORT_OPTION || s_sb_len < 1) return;

    uint8_t cmd = s_sb_buf[0];
    uint8_t *params = s_sb_buf + 1;
    size_t param_len = s_sb_len - 1;

    switch (cmd) {
    case CPO_SET_BAUDRATE: {
        if (param_len < 4) break;
        uint32_t baud = ((uint32_t)params[0] << 24) |
                        ((uint32_t)params[1] << 16) |
                        ((uint32_t)params[2] << 8) |
                        ((uint32_t)params[3]);
        ESP_LOGI(TAG, "SET-BAUDRATE: %lu", (unsigned long)baud);
        if (baud > 0) {
            usb_serial_set_line_coding(baud, 8, 0, 0);
        }
        /* Echo back the confirmed baud rate */
        uint8_t resp[4] = {
            (uint8_t)(baud >> 24), (uint8_t)(baud >> 16),
            (uint8_t)(baud >> 8),  (uint8_t)(baud)
        };
        send_sb_response(sock, COMPORT_OPTION,
                         CPO_SET_BAUDRATE + CPO_RESP_OFFSET, resp, 4);
        break;
    }
    case CPO_SET_DATASIZE: {
        uint8_t datasize = (param_len >= 1) ? params[0] : 8;
        ESP_LOGI(TAG, "SET-DATASIZE: %d", datasize);
        uint8_t resp = datasize;
        send_sb_response(sock, COMPORT_OPTION,
                         CPO_SET_DATASIZE + CPO_RESP_OFFSET, &resp, 1);
        break;
    }
    case CPO_SET_PARITY: {
        uint8_t parity = (param_len >= 1) ? params[0] : 0;
        ESP_LOGI(TAG, "SET-PARITY: %d", parity);
        uint8_t resp = parity;
        send_sb_response(sock, COMPORT_OPTION,
                         CPO_SET_PARITY + CPO_RESP_OFFSET, &resp, 1);
        break;
    }
    case CPO_SET_STOPSIZE: {
        uint8_t stopsize = (param_len >= 1) ? params[0] : 1;
        ESP_LOGI(TAG, "SET-STOPSIZE: %d", stopsize);
        uint8_t resp = stopsize;
        send_sb_response(sock, COMPORT_OPTION,
                         CPO_SET_STOPSIZE + CPO_RESP_OFFSET, &resp, 1);
        break;
    }
    case CPO_SET_CONTROL: {
        if (param_len < 1) break;
        uint8_t val = params[0];
        ESP_LOGI(TAG, "SET-CONTROL: %d", val);
        switch (val) {
        case CONTROL_DTR_ON:  s_dtr = true; break;
        case CONTROL_DTR_OFF: s_dtr = false; break;
        case CONTROL_RTS_ON:  s_rts = true; break;
        case CONTROL_RTS_OFF: s_rts = false; break;
        default: break;
        }
        usb_serial_set_control_lines(s_dtr, s_rts);
        uint8_t resp = val;
        send_sb_response(sock, COMPORT_OPTION,
                         CPO_SET_CONTROL + CPO_RESP_OFFSET, &resp, 1);
        break;
    }
    case CPO_PURGE_DATA: {
        ESP_LOGI(TAG, "PURGE-DATA");
        uint8_t resp = (param_len >= 1) ? params[0] : 0;
        send_sb_response(sock, COMPORT_OPTION,
                         CPO_PURGE_DATA + CPO_RESP_OFFSET, &resp, 1);
        break;
    }
    default:
        ESP_LOGW(TAG, "Unknown CPO command: %d", cmd);
        break;
    }
}

/* ---- Telnet byte processor ---- */

static telnet_state_t s_state = TS_DATA;

static void process_byte(int sock, uint8_t byte)
{
    switch (s_state) {
    case TS_DATA:
        if (byte == IAC) {
            s_state = TS_IAC;
        } else {
            /* Regular data byte → send to printer */
            usb_serial_send(&byte, 1);
        }
        break;

    case TS_IAC:
        switch (byte) {
        case IAC:
            /* Escaped IAC (0xFF literal) → send to printer */
            usb_serial_send(&byte, 1);
            s_state = TS_DATA;
            break;
        case WILL: s_state = TS_WILL; break;
        case WONT: s_state = TS_WONT; break;
        case DO:   s_state = TS_DO;   break;
        case DONT: s_state = TS_DONT; break;
        case SB:   s_state = TS_SB;   break;
        default:
            s_state = TS_DATA;
            break;
        }
        break;

    case TS_WILL:
        if (byte == COMPORT_OPTION) {
            send_iac_cmd(sock, DO, COMPORT_OPTION);
        } else {
            send_iac_cmd(sock, DONT, byte);
        }
        s_state = TS_DATA;
        break;

    case TS_WONT:
        send_iac_cmd(sock, DONT, byte);
        s_state = TS_DATA;
        break;

    case TS_DO:
        if (byte == COMPORT_OPTION) {
            send_iac_cmd(sock, WILL, COMPORT_OPTION);
        } else {
            send_iac_cmd(sock, WONT, byte);
        }
        s_state = TS_DATA;
        break;

    case TS_DONT:
        send_iac_cmd(sock, WONT, byte);
        s_state = TS_DATA;
        break;

    case TS_SB:
        s_sb_option = byte;
        s_sb_len = 0;
        s_state = TS_SB_DATA;
        break;

    case TS_SB_DATA:
        if (byte == IAC) {
            s_state = TS_SB_IAC;
        } else if (s_sb_len < SB_BUF_SIZE) {
            s_sb_buf[s_sb_len++] = byte;
        }
        break;

    case TS_SB_IAC:
        if (byte == SE) {
            handle_subnegotiation(sock);
            s_state = TS_DATA;
        } else if (byte == IAC) {
            /* Escaped 0xFF within subnegotiation */
            if (s_sb_len < SB_BUF_SIZE) {
                s_sb_buf[s_sb_len++] = IAC;
            }
            s_state = TS_SB_DATA;
        } else {
            s_state = TS_DATA;
        }
        break;
    }
}

/* ---- TX drain task: StreamBuffer → TCP ---- */

static void rfc2217_tx_drain_task(void *arg)
{
    uint8_t buf[256];
    uint8_t out[512];  /* Worst case: every byte is 0xFF → doubled */

    while (true) {
        /* Block until data arrives — no CPU usage while idle */
        size_t n = xStreamBufferReceive(s_serial_rx_buf, buf, sizeof(buf),
                                         portMAX_DELAY);
        if (n == 0 || s_client_sock < 0) {
            /* No client — discard data so buffer doesn't fill up */
            continue;
        }

        /* IAC-escape any 0xFF bytes */
        size_t out_len = 0;
        for (size_t i = 0; i < n; i++) {
            if (buf[i] == IAC) {
                out[out_len++] = IAC;
                out[out_len++] = IAC;
            } else {
                out[out_len++] = buf[i];
            }
        }

        tcp_send(s_client_sock, out, out_len);
    }
}

/* ---- TCP server task ---- */

static void rfc2217_server_task(void *arg)
{
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_RFC2217_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "Bind failed");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_sock, 1) != 0) {
        ESP_LOGE(TAG, "Listen failed");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "RFC 2217 server listening on port %d", CONFIG_RFC2217_PORT);

    while (true) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        int client = accept(listen_sock, (struct sockaddr *)&client_addr,
                            &addr_len);
        if (client < 0) {
            ESP_LOGW(TAG, "Accept failed");
            continue;
        }

        char addr_str[16];
        inet_ntoa_r(client_addr.sin_addr, addr_str, sizeof(addr_str));

        /* Ender Smart (F15-11 / C10): exclusive with the ESP32's own print job */
        if (printer_comm_job_active()) {
            ESP_LOGW(TAG, "Refusing client %s: a print is running", addr_str);
            close(client);
            continue;
        }
        ESP_LOGI(TAG, "Client connected from %s", addr_str);

        /* Only one client at a time */
        s_client_sock = client;
        s_state = TS_DATA;
        s_dtr = true;
        s_rts = true;

        /* Set TCP_NODELAY for low-latency serial forwarding */
        int nodelay = 1;
        setsockopt(client, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        /* Announce COM-PORT-OPTION support */
        send_iac_cmd(client, WILL, COMPORT_OPTION);

        /* Read loop */
        uint8_t rx_buf[256];
        while (true) {
            int n = recv(client, rx_buf, sizeof(rx_buf), 0);
            if (n <= 0) break;
            if (printer_comm_job_active()) {
                ESP_LOGW(TAG, "Print started: disconnecting RFC 2217 client");
                break;
            }

            for (int i = 0; i < n; i++) {
                process_byte(client, rx_buf[i]);
            }
        }

        ESP_LOGI(TAG, "Client disconnected");
        s_client_sock = -1;
        close(client);
    }
}

/* ---- Public API ---- */

void rfc2217_feed_serial_data(const uint8_t *data, size_t len, void *user_ctx)
{
    if (s_serial_rx_buf && len > 0) {
        /* Best-effort write from possibly ISR context */
        xStreamBufferSend(s_serial_rx_buf, data, len, 0);
    }
}

esp_err_t rfc2217_start(void)
{
    s_serial_rx_buf = xStreamBufferCreate(4096, 1);
    if (!s_serial_rx_buf) {
        ESP_LOGE(TAG, "Failed to create stream buffer");
        return ESP_ERR_NO_MEM;
    }

    xTaskCreatePinnedToCore(rfc2217_server_task, "rfc2217_srv", 6144,
                            NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(rfc2217_tx_drain_task, "rfc2217_tx", 4096,
                            NULL, 12, NULL, 0);

    return ESP_OK;
}
