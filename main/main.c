#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"

#include "esp_twai.h"
#include "esp_twai_onchip.h"

static const char TAG[] = "main";

struct nmea_frame {
    uint32_t identifier;

    uint8_t data[8];

    size_t length;
};

static QueueHandle_t rx_queue = NULL;

static bool IRAM_ATTR twai_listener_on_error_callback(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx)
{
    ESP_EARLY_LOGW(TAG, "bus error: arb_lost:%d bit_err:%d form_err:%d ", edata->err_flags.arb_lost, edata->err_flags.bit_err, edata->err_flags.form_err);
    return false;
}

// Node state
static bool IRAM_ATTR twai_listener_on_state_change_callback(twai_node_handle_t handle, const twai_state_change_event_data_t *edata, void *user_ctx)
{
    const char *twai_state_name[] = {"error_active", "error_warning", "error_passive", "bus_off"};
    ESP_EARLY_LOGI(TAG, "state changed: %s -> %s", twai_state_name[edata->old_sta], twai_state_name[edata->new_sta]);
    return false;
}

static bool IRAM_ATTR nmea_on_received(
    twai_node_handle_t handle,
    const twai_rx_done_event_data_t *edata,
    void *user_ctx
)
{
    struct nmea_frame n2k_frame  = {0};
    BaseType_t woken = pdFALSE;

    twai_frame_t rx_frame = {
        .buffer = n2k_frame.data,
        .buffer_len = sizeof(n2k_frame.data) / sizeof(uint8_t),
    };

    if (twai_node_receive_from_isr(handle, &rx_frame) == ESP_OK) {
        n2k_frame.identifier = rx_frame.header.id;
        n2k_frame.length = rx_frame.header.dlc;

        xQueueSendFromISR(rx_queue, &n2k_frame, &woken);
    }
    return woken == pdTRUE;
}

static esp_err_t nmea_init(twai_node_handle_t *handle) {
    static twai_onchip_node_config_t node_config = {
        .io_cfg.tx = GPIO_NUM_10,
        .io_cfg.rx = GPIO_NUM_9,
        .bit_timing.bitrate = 1000000,
        .data_timing.bitrate = 2000000,
        .tx_queue_depth = 5,
        .fail_retry_cnt = -1,
    };

    ESP_RETURN_ON_ERROR(
        twai_new_node_onchip(&node_config, handle),
        TAG,
        "failed to create new node"
    );
    
    static twai_event_callbacks_t callbacks = {
        .on_rx_done = nmea_on_received,
        .on_error = twai_listener_on_error_callback,
        .on_state_change = twai_listener_on_state_change_callback,
    };

    ESP_RETURN_ON_ERROR(
        twai_node_register_event_callbacks(
            *handle,
            &callbacks,
            NULL
        ),
        TAG,
        "failed to register cbs"
    );

    ESP_RETURN_ON_ERROR(
        twai_node_enable(*handle),
        TAG,
        "failed to enable"
    );

    return ESP_OK;
}

static esp_err_t nmea_queues_init() {
    rx_queue = xQueueCreate(100, sizeof(struct nmea_frame));

    if (rx_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void app_main() {
    twai_node_handle_t handle = NULL;

    ESP_LOGI(TAG, "init");

    ESP_ERROR_CHECK(nmea_queues_init());
    ESP_ERROR_CHECK(nmea_init(&handle));
    
    ESP_LOGI(TAG, "init done");

    struct nmea_frame received_frame = {0};
    uint8_t send_buff[32] = {0};
    for (int i = 0; i < 32; i++) send_buff[i] = i;
    twai_frame_t tx_msg = {
        .header.id = 0x1,           // Message ID
        .header.ide = true,         // Use 29-bit extended ID format
        .header.fdf = true,
        .header.brs = true,
        .buffer = send_buff,        // Pointer to data to transmit
        .buffer_len = sizeof(send_buff),  // Length of data to transmit
    };
    ESP_ERROR_CHECK(twai_node_transmit(handle, &tx_msg, 0));

    for(;;) {
        BaseType_t ret = xQueueReceive(rx_queue, &received_frame, portMAX_DELAY);

        if (ret == pdTRUE) {
            ESP_LOGI(TAG, "received frame with id %" PRIx32, received_frame.identifier);
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}