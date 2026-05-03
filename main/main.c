#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hidd.h"
#include "driver/uart.h"

#define TAG "SWITCH_CON"

// -----------------------------------------------------------------------
// Nintendo Switch Pro Controller subcommand protocol
//
// OUTPUT report format (Switch -> Controller):
//   [0] report_id (0x01 = rumble+subcommand, 0x10 = rumble only)
//   [1] timer
//   [2..9] rumble data (8 bytes)
//   [10] subcommand ID
//   [11..] subcommand data
//
// INPUT report format (Controller -> Switch):
//   [0] report_id (0x21 = subcommand reply)
//   [1] timer
//   [2] conn info + battery
//   [3..11] button/stick data (standard input)
//   [12] ACK byte (0x80 | subcommand_id)
//   [13] replied subcommand ID
//   [14..] reply data
// -----------------------------------------------------------------------

// Standard input report: button bytes + sticks in Pro Controller 12-bit format
// Stick center = 0x800. Encoding: [x_lo, x_hi|y_lo, y_hi]
static const uint8_t NEUTRAL_INPUT[9] = {
    0x00, 0x00, 0x00,  // buttons (right, shared, left)
    0x00, 0x08, 0x80,  // left stick center  (12-bit 0x800, 0x800)
    0x00, 0x08, 0x80,  // right stick center (12-bit 0x800, 0x800)
};

// SPI flash region that Switch reads to verify it's a real controller
// Offsets 0x6000-0x6012: factory calibration / device type info
static const uint8_t SPI_FLASH_DATA[0x1D] = {
    // 0x6000: Serial number (16 bytes, 0xFF = empty)
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    // 0x6010: Device type (0x03 = Pro Controller), 0x6011: unknown, 0x6012: colors
    0x03, 0x00, 0xA0,
    // placeholder bytes
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF,
};

static esp_hidd_dev_t *hid_dev = NULL;
static volatile bool bt_connected = false;
static uint8_t input_timer = 0;
static TaskHandle_t s_input_task = NULL;
static volatile esp_bt_pm_mode_t bt_link_mode = ESP_BT_PM_MD_ACTIVE;

// Queue for offloading OUTPUT report processing off the bt_hidd_events task stack
typedef struct {
    uint8_t report_id;
    uint8_t data[48];
    size_t  len;
} output_msg_t;
static QueueHandle_t s_output_queue = NULL;

// Scan mode request flags - set by callbacks, applied by scan_mode_task
typedef enum { SCAN_DISCOVERABLE, SCAN_HIDDEN } scan_mode_req_t;
static QueueHandle_t s_scan_queue = NULL;

// Set to true only after Switch sends subcommand 0x03 (Set Input Report Mode)
// Prevents 0x30 reports from flooding sniff slots during the subcommand handshake
static volatile bool s_input_reporting = false;

// Set to true while a 0x21 subcommand reply is being sent.
// input_report_task will pause 0x30 during this window to avoid competing for sniff slots.
static volatile bool s_reply_pending = false;

// -----------------------------------------------------------------------
// Controller state - updated by uart_cmd_task, read by send_input_report_30
// Button byte layout matches Pro Controller 0x30 input report:
//   btn_right : bit7=ZR  bit6=R   bit3=A   bit2=B   bit1=X   bit0=Y
//   btn_shared: bit5=CAP bit4=HOME bit3=LS  bit2=RS  bit1=+   bit0=-
//   btn_left  : bit7=ZL  bit6=L   bit3=LEFT bit2=RIGHT bit1=UP bit0=DOWN
// Sticks: 12-bit unsigned, center = 0x800
// -----------------------------------------------------------------------
typedef struct {
    volatile uint8_t  btn_right;
    volatile uint8_t  btn_shared;
    volatile uint8_t  btn_left;
    volatile uint16_t lx, ly, rx, ry;
} ctrl_state_t;

static ctrl_state_t s_ctrl = {
    .lx = 0x800, .ly = 0x800, .rx = 0x800, .ry = 0x800,
};

// btn_right bits
#define BTN_Y   0x01
#define BTN_X   0x02
#define BTN_B   0x04
#define BTN_A   0x08
#define BTN_R   0x40
#define BTN_ZR  0x80
// btn_shared bits
#define BTN_MINUS 0x01
#define BTN_PLUS  0x02
#define BTN_RS    0x04
#define BTN_LS    0x08
#define BTN_HOME  0x10
#define BTN_CAP   0x20
// btn_left bits
#define BTN_DOWN  0x01
#define BTN_UP    0x02
#define BTN_RIGHT 0x04
#define BTN_LEFT  0x08
#define BTN_L     0x40
#define BTN_ZL    0x80

// Forward declaration
static void handle_subcommand(const uint8_t *data, size_t len);

// Send standard full input report (0x30, 48 bytes) - sent at 60 Hz
static void send_input_report_30(void)
{
    if (!hid_dev || !esp_hidd_dev_connected(hid_dev)) return;
    if (s_reply_pending) return;  // Yield slot to 0x21 reply

    uint8_t buf[48] = {0};
    buf[0]  = input_timer++;  // timer
    buf[1]  = 0x8E;           // battery full + USB
    // buttons from controller state
    buf[2]  = s_ctrl.btn_right;
    buf[3]  = s_ctrl.btn_shared;
    buf[4]  = s_ctrl.btn_left;
    // left stick (12-bit little-endian)
    buf[5]  = (uint8_t)(s_ctrl.lx & 0xFF);
    buf[6]  = (uint8_t)((s_ctrl.lx >> 8) | ((s_ctrl.ly & 0x0F) << 4));
    buf[7]  = (uint8_t)(s_ctrl.ly >> 4);
    // right stick
    buf[8]  = (uint8_t)(s_ctrl.rx & 0xFF);
    buf[9]  = (uint8_t)((s_ctrl.rx >> 8) | ((s_ctrl.ry & 0x0F) << 4));
    buf[10] = (uint8_t)(s_ctrl.ry >> 4);
    // vibration report counter
    buf[11] = 0x00;
    // IMU data: 3 samples of accel+gyro (12 bytes each, little-endian int16)
    // At rest: accel_z = 0x1000 (4096 LSB = 1G at ±8G range), gyro = 0
    // Sample 1: buf[12..23]
    buf[16] = 0x00; buf[17] = 0x10;  // accel_z sample 1
    // Sample 2: buf[24..35]
    buf[28] = 0x00; buf[29] = 0x10;  // accel_z sample 2
    // Sample 3: buf[36..47]
    buf[40] = 0x00; buf[41] = 0x10;  // accel_z sample 3

    esp_hidd_dev_input_set(hid_dev, 0, 0x30, buf, sizeof(buf));
}

// -----------------------------------------------------------------------
// UART command task - reads single characters from PC via COM4 (UART0)
//
// Key bindings:
//   a/b/x/y       - A/B/X/Y buttons (tap 80ms)
//   l/r           - L/R buttons
//   z/c           - ZL/ZR buttons
//   +/-           - Plus/Minus buttons
//   h             - HOME button
//   p             - Capture button
//   w/s/q/e       - D-pad Up/Down/Left/Right
//   Hold Shift equivalent: uppercase = hold until same key again (toggle)
// -----------------------------------------------------------------------
static void uart_cmd_task(void *arg)
{
    // Install UART0 driver for RX (TX is still used by ESP_LOG)
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

    ESP_LOGI(TAG, "UART control ready. Keys: a/b/x/y l/r z/c +/- h p  dpad:w/s/q/e");

    uint8_t ch;
    while (1) {
        if (uart_read_bytes(UART_NUM_0, &ch, 1, portMAX_DELAY) <= 0) continue;

        volatile uint8_t *byte = NULL;
        uint8_t           mask = 0;

        switch (ch) {
        // Face buttons
        case 'a': byte = &s_ctrl.btn_right;  mask = BTN_A;    break;
        case 'b': byte = &s_ctrl.btn_right;  mask = BTN_B;    break;
        case 'x': byte = &s_ctrl.btn_right;  mask = BTN_X;    break;
        case 'y': byte = &s_ctrl.btn_right;  mask = BTN_Y;    break;
        // Shoulder
        case 'l': byte = &s_ctrl.btn_left;   mask = BTN_L;    break;
        case 'r': byte = &s_ctrl.btn_right;  mask = BTN_R;    break;
        case 'z': byte = &s_ctrl.btn_left;   mask = BTN_ZL;   break;
        case 'c': byte = &s_ctrl.btn_right;  mask = BTN_ZR;   break;
        // System
        case '+': byte = &s_ctrl.btn_shared; mask = BTN_PLUS;  break;
        case '-': byte = &s_ctrl.btn_shared; mask = BTN_MINUS; break;
        case 'h': byte = &s_ctrl.btn_shared; mask = BTN_HOME;  break;
        case 'p': byte = &s_ctrl.btn_shared; mask = BTN_CAP;   break;
        // D-pad (wasd-like: w=up s=down q=left e=right)
        case 'w': byte = &s_ctrl.btn_left;   mask = BTN_UP;    break;
        case 's': byte = &s_ctrl.btn_left;   mask = BTN_DOWN;  break;
        case 'q': byte = &s_ctrl.btn_left;   mask = BTN_LEFT;  break;
        case 'e': byte = &s_ctrl.btn_left;   mask = BTN_RIGHT; break;
        default: continue;
        }

        // Tap: press for 80ms then release
        *byte |= mask;
        vTaskDelay(pdMS_TO_TICKS(80));
        *byte &= ~mask;
        vTaskDelay(pdMS_TO_TICKS(20));  // brief gap between presses
    }
}

// Scan mode task - applies scan mode changes requested by BT callbacks
static void scan_mode_task(void *arg)
{
    scan_mode_req_t req;
    while (1) {
        if (xQueueReceive(s_scan_queue, &req, portMAX_DELAY) == pdTRUE) {
            if (req == SCAN_DISCOVERABLE) {
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            } else {
                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            }
        }
    }
}

// Output report handler task - processes subcommands with its own 4KB stack
static void output_handler_task(void *arg)
{
    output_msg_t msg;
    while (1) {
        if (xQueueReceive(s_output_queue, &msg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "OUTPUT report id=0x%02X len=%d", msg.report_id, msg.len);
            s_reply_pending = true;
            // report_id 0x01: [timer(1)][rumble_L(4)][rumble_R(4)][subcmd_id(1)][subcmd_data...]
            if (msg.report_id == 0x01 && msg.len >= 10) {
                handle_subcommand(&msg.data[9], msg.len - 9);
            }
            vTaskDelay(pdMS_TO_TICKS(10));  // short gap after reply before 0x30 resumes
            s_reply_pending = false;
        }
    }
}

// ~60 Hz input report task - runs while connected, only sends in active BT mode
static void input_report_task(void *arg)
{
    // Wait for L2CAP interrupt channel to be ready (FEAT/ATTE2 negotiation)
    vTaskDelay(pdMS_TO_TICKS(500));

    // Phase 1: Send 0x30 every 100ms until Switch starts the subcommand handshake.
    // Retry loop needed because the first send may silently fail if the interrupt
    // channel is not fully open yet despite CONNECT_EVENT having fired.
    while (bt_connected && !s_input_reporting) {
        if (esp_hidd_dev_connected(hid_dev)) {
            send_input_report_30();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Phase 2: Handshake complete (0x03 received) - run at 60 Hz
    while (bt_connected) {
        send_input_report_30();
        vTaskDelay(pdMS_TO_TICKS(16));
    }
    s_input_task = NULL;
    vTaskDelete(NULL);
}

// Send a subcommand reply (Input Report ID 0x21)
static void send_subcmd_reply(uint8_t subcmd_id, const uint8_t *data, size_t data_len)
{
    if (!hid_dev || !esp_hidd_dev_connected(hid_dev)) return;

    // Report 0x21: subcommand reply (48 bytes)
    uint8_t buf[48] = {0};
    buf[0]  = input_timer++;   // timer
    buf[1]  = 0x8E;            // battery full + USB connected
    // neutral button/stick state
    memcpy(&buf[2], NEUTRAL_INPUT, sizeof(NEUTRAL_INPUT));
    // buf[11] = vibration report counter (stays 0x00)
    // ACK: use 0x80|subcmd when reply has data, plain 0x80 for simple ACKs.
    // Real Pro Controller behavior: 0x82 for 0x02 (has data), 0x80 for 0x08/0x03 etc.
    buf[12] = (data && data_len > 0) ? (0x80 | subcmd_id) : 0x80;
    buf[13] = subcmd_id;        // replied subcommand ID
    if (data && data_len > 0) {
        size_t copy_len = data_len < 34 ? data_len : 34;
        memcpy(&buf[14], data, copy_len);
    }

    esp_hidd_dev_input_set(hid_dev, 0, 0x21, buf, sizeof(buf));
}

// Handle subcommand from Switch
static void handle_subcommand(const uint8_t *data, size_t len)
{
    if (len < 1) return;
    uint8_t subcmd = data[0];
    const uint8_t *subcmd_data = (len > 1) ? &data[1] : NULL;

    ESP_LOGI(TAG, "Subcommand: 0x%02X", subcmd);

    switch (subcmd) {
    case 0x00: // Get State (no-op ACK)
        send_subcmd_reply(0x00, NULL, 0);
        break;

    case 0x01: // Bluetooth Manual Pairing
        send_subcmd_reply(0x01, NULL, 0);
        break;

    case 0x02: { // Request Device Info
        uint8_t reply[11] = {
            0x03, 0x48,             // firmware version 3.72
            0x03,                   // Pro Controller type
            0x02,                   // unknown
            // MAC address (reversed)
            0x88, 0x57, 0x21, 0x37, 0x37, 0x7E,
            0x01,                   // unknown
        };
        send_subcmd_reply(0x02, reply, sizeof(reply));
        break;
    }

    case 0x03: // Set Input Report Mode
        send_subcmd_reply(0x03, NULL, 0);
        break;

    case 0x04: // Trigger Buttons Elapsed Time
        send_subcmd_reply(0x04, NULL, 0);
        break;

    case 0x08: // Set Shipment State
        send_subcmd_reply(0x08, NULL, 0);
        break;

    case 0x10: { // SPI Flash Read
        // data[0..3] = offset (little-endian), data[4] = length
        if (!subcmd_data || len < 6) {
            send_subcmd_reply(0x10, NULL, 0);
            break;
        }
        uint32_t offset = subcmd_data[0] | (subcmd_data[1] << 8) |
                          (subcmd_data[2] << 16) | (subcmd_data[3] << 24);
        uint8_t read_len = subcmd_data[4];
        ESP_LOGI(TAG, "SPI read: offset=0x%04"PRIX32" len=%d", offset, read_len);

        // Reply: echo offset(4) + length(1) + data
        uint8_t reply[35] = {0};
        reply[0] = subcmd_data[0]; reply[1] = subcmd_data[1];
        reply[2] = subcmd_data[2]; reply[3] = subcmd_data[3];
        reply[4] = read_len;
        // Fill with 0xFF (blank flash) - specific regions return real data
        memset(&reply[5], 0xFF, read_len < 30 ? read_len : 30);
        if (offset == 0x6000 && read_len <= sizeof(SPI_FLASH_DATA)) {
            memcpy(&reply[5], SPI_FLASH_DATA, read_len);
        }
        send_subcmd_reply(0x10, reply, 5 + read_len);
        break;
    }

    case 0x21: // Set NFC/IR MCU Config
        send_subcmd_reply(0x21, NULL, 0);
        break;

    case 0x30: // Set Player Lights
        ESP_LOGI(TAG, "Set player lights: 0x%02X", subcmd_data ? subcmd_data[0] : 0);
        send_subcmd_reply(0x30, NULL, 0);
        s_input_reporting = true;  // Full handshake done - start 60Hz reports
        break;

    case 0x38: // Set HOME Light
        send_subcmd_reply(0x38, NULL, 0);
        break;

    case 0x40: // Enable IMU (6-axis sensor)
        send_subcmd_reply(0x40, NULL, 0);
        break;

    case 0x41: // Set IMU Sensitivity
        send_subcmd_reply(0x41, NULL, 0);
        break;

    case 0x48: // Enable Vibration
        send_subcmd_reply(0x48, NULL, 0);
        break;

    default:
        ESP_LOGW(TAG, "Unknown subcommand: 0x%02X", subcmd);
        send_subcmd_reply(subcmd, NULL, 0);
        break;
    }
}

// Nintendo Switch Pro Controller HID Report Descriptor
// Uses vendor-specific usage page (0xFF00) with explicit report IDs
// matching the real Pro Controller protocol.
static const uint8_t pro_controller_hid_report_map[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)

    // --- Vendor-specific input reports ---
    0x06, 0x00, 0xFF,  //   Usage Page (Vendor 0xFF00)

    // Report 0x21: Subcommand reply (48 bytes)
    0x85, 0x21,        //   Report ID (0x21)
    0x09, 0x21,        //   Usage (0x21)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x30,        //   Report Count (48)
    0x81, 0x02,        //   Input (Data, Var, Abs)

    // Report 0x30: Standard full input (48 bytes, sent at 60 Hz)
    0x85, 0x30,        //   Report ID (0x30)
    0x09, 0x30,        //   Usage (0x30)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x30,        //   Report Count (48)
    0x81, 0x02,        //   Input (Data, Var, Abs)

    // Report 0x31: NFC/IR input (362 bytes)
    0x85, 0x31,        //   Report ID (0x31)
    0x09, 0x31,        //   Usage (0x31)
    0x75, 0x08,        //   Report Size (8)
    0x96, 0x6A, 0x01,  //   Report Count (362)
    0x81, 0x02,        //   Input (Data, Var, Abs)

    // --- Vendor-specific output reports (Switch -> Controller) ---

    // Report 0x01: Subcommand (48 bytes)
    0x85, 0x01,        //   Report ID (0x01)
    0x09, 0x01,        //   Usage (0x01)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x30,        //   Report Count (48)
    0x91, 0x02,        //   Output (Data, Var, Abs)

    // Report 0x10: Rumble only (9 bytes)
    0x85, 0x10,        //   Report ID (0x10)
    0x09, 0x10,        //   Usage (0x10)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x09,        //   Report Count (9)
    0x91, 0x02,        //   Output (Data, Var, Abs)

    // Report 0x12: Unknown output
    0x85, 0x12,        //   Report ID (0x12)
    0x09, 0x12,        //   Usage (0x12)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x30,        //   Report Count (48)
    0x91, 0x02,        //   Output (Data, Var, Abs)

    0xC0               // End Collection
};

static esp_hid_raw_report_map_t report_maps[] = {
    {
        .data = pro_controller_hid_report_map,
        .len  = sizeof(pro_controller_hid_report_map),
    }
};

// Nintendo Switch Pro Controller identity
static esp_hid_device_config_t hidd_config = {
    .vendor_id         = 0x057E,  // Nintendo
    .product_id        = 0x2009,  // Pro Controller
    .version           = 0x0100,
    .device_name       = "Pro Controller",
    .manufacturer_name = "Nintendo Co., Ltd.",
    .serial_number     = "000000000001",
    .report_maps       = report_maps,
    .report_maps_len   = 1,
};

// --- GAP callback: pairing / security ---
static void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Paired with: %s", param->auth_cmpl.device_name);
        } else {
            ESP_LOGE(TAG, "Pairing failed, status: %d", param->auth_cmpl.stat);
        }
        break;
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "SSP confirm request (passkey: %"PRIu32") - auto confirming", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "SSP passkey notification: %"PRIu32, param->key_notif.passkey);
        break;
    case ESP_BT_GAP_MODE_CHG_EVT:
        bt_link_mode = param->mode_chg.mode;
        ESP_LOGD(TAG, "BT link mode: %d", bt_link_mode);
        break;
    default:
        ESP_LOGD(TAG, "GAP event: %d", event);
        break;
    }
}

// --- HID device event callback ---
static void hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDD_START_EVENT:
        ESP_LOGI(TAG, "HID device started - now discoverable");
        // Do NOT call esp_bt_gap_set_scan_mode here - bt_hidd_events stack is too small
        {
            scan_mode_req_t req = SCAN_DISCOVERABLE;
            if (s_scan_queue) xQueueSend(s_scan_queue, &req, 0);
        }
        break;
    case ESP_HIDD_CONNECT_EVENT:
        ESP_LOGI(TAG, "Switch connected!");
        bt_connected = true;
        bt_link_mode = ESP_BT_PM_MD_ACTIVE;
        input_timer = 0;
        {
            scan_mode_req_t req = SCAN_HIDDEN;
            if (s_scan_queue) xQueueSend(s_scan_queue, &req, 0);
        }
        if (s_input_task == NULL) {
            xTaskCreate(input_report_task, "input_rpt", 2048, NULL, 5, &s_input_task);
        }
        break;
    case ESP_HIDD_DISCONNECT_EVENT:
        ESP_LOGI(TAG, "Disconnected: %s",
                 esp_hid_disconnect_reason_str(
                     esp_hidd_dev_transport_get(param->disconnect.dev),
                     param->disconnect.reason));
        bt_connected = false;
        s_input_reporting = false;  // Reset for next connection
        {
            scan_mode_req_t req = SCAN_DISCOVERABLE;
            if (s_scan_queue) xQueueSend(s_scan_queue, &req, 0);
        }
        break;
    case ESP_HIDD_OUTPUT_EVENT: {
        // Enqueue to output handler task - do NOT process here (bt_hidd_events stack is small)
        if (s_output_queue && param->output.length <= 48) {
            output_msg_t msg;
            msg.report_id = param->output.report_id;
            msg.len = param->output.length;
            memcpy(msg.data, param->output.data, msg.len);
            xQueueSend(s_output_queue, &msg, 0);
        }
        break;
    }
    case ESP_HIDD_STOP_EVENT:
        ESP_LOGI(TAG, "HID device stopped");
        break;
    default:
        break;
    }
}

void app_main(void)
{
    esp_err_t ret;

    // NVS init (required by BT stack)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Release BLE controller memory (unused)
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    // Bluetooth controller init (Classic BT only)
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    // Bluedroid stack init
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // GAP: register callback, set device name
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_callback));
    ESP_ERROR_CHECK(esp_bt_gap_set_device_name("Pro Controller"));

    // Class of Device: Peripheral (major=5), Gamepad (minor=2)
    esp_bt_cod_t cod = {
        .service = 1,    // bit 13 set = 0x002508 - required for Switch 12.0+
        .major   = 5,
        .minor   = 2,
        .reserved_2 = 0,
        .reserved_8 = 0,
    };
    ESP_ERROR_CHECK(esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_ALL));

    // SSP: IO capability = None -> "Just Works" pairing
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    ESP_ERROR_CHECK(esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(iocap)));

    // Create queues and tasks BEFORE hidd_dev_init so START_EVENT can use them
    s_scan_queue   = xQueueCreate(4, sizeof(scan_mode_req_t));
    s_output_queue = xQueueCreate(8, sizeof(output_msg_t));
    xTaskCreate(scan_mode_task,      "scan_mode",  2048, NULL, 5, NULL);
    xTaskCreate(output_handler_task, "output_hdl", 4096, NULL, 5, NULL);
    xTaskCreate(uart_cmd_task,       "uart_cmd",   2048, NULL, 4, NULL);

    // HID device init with Classic BT transport
    ESP_ERROR_CHECK(esp_hidd_dev_init(&hidd_config, ESP_HID_TRANSPORT_BT, hidd_event_callback, &hid_dev));

    ESP_LOGI(TAG, "Pro Controller emulator ready - pair from Switch: Controllers > Change Grip/Order");
}
