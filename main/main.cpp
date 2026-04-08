#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <math.h>

#include "wifi.h"

#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"

#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "messageFormat/motorBoard.hpp"
#include "messageFormat/powerboard.hpp"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

int sock = -1;

bool connected = false;
struct sockaddr_storage source_addr;
struct receive
{
  char power;
  float linear_x;
  float linear_y;
  float linear_z;
  float AXIS[5];
  char button[5];
}__attribute__((packed));


struct canfd_frame
{
  uint16_t identifier;
  uint8_t data[64];
  size_t length;
  bool is_remote;
};

#define DEG_TO_RAD(deg) ((deg) / 180.0 * M_PI) // 度からラジアンへの変換

static QueueHandle_t rx_queue = NULL;

static const char TAG[] = "main";
void send_can(ID_Format id, uint8_t *data, size_t size, bool is_remote);


void sendMotorControlMessages(void* data)
{
  
  receive frame = *reinterpret_cast<receive*>(data);

  PowerBoard_Target powermsg = {0};
  ID_Format id;
  id.format.to_BoardType = Board_Type::PowerBoard;
  id.format.to_BoardID = 0;
  id.format.message_type = Message_Type::Target;
  powermsg.ON_OFF = frame.power;
  //MROS2_INFO("power %s", msg->data ? "on" : "off");

  send_can(id, reinterpret_cast<uint8_t *>(&powermsg), sizeof(PowerBoard_Target), false);

  id.format.to_BoardType = Board_Type::MotorBoard;
  id.format.to_BoardID = 0;
  id.format.message_type = Message_Type::Target;

  MotorBoard_Target sendmsg;
  sendmsg.mode = ControlMode::PWM_Mode;
  sendmsg.target[1] = static_cast<int>(sinf(DEG_TO_RAD(frame.linear_x)) * frame.linear_y + frame.linear_z);
  sendmsg.target[2] = static_cast<int>(sinf(DEG_TO_RAD(frame.linear_x - 120.0f)) * frame.linear_y + frame.linear_z);
  sendmsg.target[3] = static_cast<int>(sinf(DEG_TO_RAD(frame.linear_x - 240.0f)) * frame.linear_y + frame.linear_z);
  sendmsg.target[0] = static_cast<int>(frame.AXIS[4] * -500.0f);
  //ESP_LOGI(TAG, "power %d %d %d %d %d %d %d %d %d", sendmsg.target[0], sendmsg.target[1], sendmsg.target[2], sendmsg.target[3], frame.button[0], frame.button[1], frame.button[2], frame.button[3], frame.button[4]);
  ESP_LOGI(TAG, "AXIS[4] %d", sendmsg.target[0] );
  send_can(id, reinterpret_cast<uint8_t *>(&sendmsg), sizeof(MotorBoard_Target), false);
  id.format.to_BoardID = 1;
  
  /*sendmsg.target[1] = 0; 
  sendmsg.mode = ControlMode::PWM_Mode;
  if (frame.button[2] == 1) {
    sendmsg.target[0] = 100; // 後退
  } else if (frame.button[4] == 1) {
    sendmsg.target[0] = -100; // 後退
  } else {
    sendmsg.target[0] = 0; // 停止r rrggg
  }

  sendmsg.target[1] = frame.button[3];

  if (frame.AXIS[1] > 0){
      sendmsg.target[2] = 0;
      sendmsg.target[3] = static_cast<int>(frame.AXIS[3] * 300.0f);

  } else {
      sendmsg.target[3] = 0;
      sendmsg.target[2] = static_cast<int>(frame.AXIS[3] * 200.0f);
  }*/
  sendmsg.target[3] = static_cast<int>(frame.linear_z * 5.0f);
  send_can(id, reinterpret_cast<uint8_t *>(&sendmsg), sizeof(MotorBoard_Target), false);
}

void sendSafetyMotorControlMessages(void)
{
  
  PowerBoard_Target powermsg = {0};
  ID_Format id;
  id.format.to_BoardType = Board_Type::PowerBoard;
  id.format.to_BoardID = 0;
  id.format.message_type = Message_Type::Target;
  powermsg.ON_OFF = false;
  send_can(id, reinterpret_cast<uint8_t *>(&powermsg), sizeof(PowerBoard_Target), false);

  // タイムアウト時のセーフティメッセージ：すべてのtargetを0に設定
  id.format.to_BoardType = Board_Type::MotorBoard;
  id.format.from_BoardType = Board_Type::Master_Board;
  id.format.from_BoardID = 0;
  id.format.message_type = Message_Type::Target;

  MotorBoard_Target sendmsg = {};
  sendmsg.mode = ControlMode::PWM_Mode;
  sendmsg.target[0] = 0;
  sendmsg.target[1] = 0;
  sendmsg.target[2] = 0;
  sendmsg.target[3] = 0;

  // すべてのボードID(0, 1, 2)に送信
  for (int board_id = 0; board_id < 3; board_id++)
  {
    id.format.to_BoardID = board_id;
    send_can(id, reinterpret_cast<uint8_t *>(&sendmsg), sizeof(MotorBoard_Target), false);
  }
  ESP_LOGI(TAG, "Safety message sent: all targets set to 0");
}

static esp_err_t queues_init()
{
  rx_queue = xQueueCreate(100, sizeof(canfd_frame));

  if (rx_queue == NULL)
  {
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

static bool canfd_on_received(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
  canfd_frame frame = {0};
  BaseType_t woken = pdFALSE;

  twai_frame_t rx_frame;
  rx_frame.buffer = frame.data,
  rx_frame.buffer_len = sizeof(frame.data) / sizeof(uint8_t);

  if (twai_node_receive_from_isr(handle, &rx_frame) == ESP_OK)
  {
    frame.identifier = rx_frame.header.id;
    frame.length = rx_frame.header.dlc;
    xQueueSendFromISR(rx_queue, &frame, &woken);
  }
  return woken == pdTRUE;
}

static esp_err_t canfd_init(twai_node_handle_t *handle)
{
  twai_onchip_node_config_t node_config = {};
  node_config.io_cfg.tx = GPIO_NUM_10;
  node_config.io_cfg.rx = GPIO_NUM_9;
  node_config.bit_timing.bitrate = 1000000;
  node_config.data_timing.bitrate = 2000000;
  node_config.tx_queue_depth = 100,
  twai_new_node_onchip(&node_config, handle);

  twai_event_callbacks_t callbacks = {};
  callbacks.on_rx_done = canfd_on_received;

  twai_node_register_event_callbacks(*handle, &callbacks, NULL);

  twai_node_enable(*handle);
  return ESP_OK;
}

twai_node_handle_t handle = NULL;

void send_can(ID_Format id, uint8_t *data, size_t size, bool is_remote)
{
  id.format.from_BoardType = Board_Type::Master_Board;
  id.format.from_BoardID = 0;

  twai_frame_t tx_msg = {};
  tx_msg.header.id = id.id; // Message ID
  tx_msg.header.ide = true; // Use 29-bit extended ID format
  tx_msg.header.fdf = true;
  tx_msg.header.brs = true;
  tx_msg.header.rtr = is_remote;
  tx_msg.buffer = data;
  tx_msg.buffer_len = size;
  ESP_ERROR_CHECK(twai_node_transmit(handle, &tx_msg, 0));
  vTaskDelay(pdMS_TO_TICKS(10));
}

TaskHandle_t rsvHandle = NULL;

#define PORT 5000

static void udp_server_task(void *pvParameters)
{
  char rx_buffer[128];
  char addr_str[128];
  int ip_protocol = 0;
  struct sockaddr_in dest_addr;
  error_t errno;
  while (1)
  {
    struct sockaddr_in *dest_addr_ip4 = &dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(PORT);
    ip_protocol = IPPROTO_IP;
    sock = socket(AF_INET, SOCK_DGRAM, ip_protocol);
    if (sock < 0)
    {
      ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
      break;
    }
    ESP_LOGI(TAG, "Socket created");
    // Set timeout to 1 second
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0)
    {
      ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    socklen_t socklen = sizeof(source_addr);

    while (1)
    {
      int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
      if (len < 0)
      {
        // Timeout occurred (1 second with no data received)
        ESP_LOGW(TAG, "recvfrom timeout: No data received for 1 second");
        connected = false;
        sendSafetyMotorControlMessages();
      }
      else
      {
        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        rx_buffer[len] = 0;
        connected = true;
        //ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
        //ESP_LOGI(TAG, "%s", rx_buffer);
        sendMotorControlMessages(rx_buffer);

        if (err < 0)
        {
          ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
          break;
        }
      }
    }

    if (sock != -1)
    {
      ESP_LOGE(TAG, "Shutting down socket and restarting...");
      shutdown(sock, 0);
      close(sock);
    }
  }
  vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
  ESP_ERROR_CHECK(canfd_init(&handle));
  ESP_ERROR_CHECK(queues_init());
  init_wifi();
  xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);

  canfd_frame received_frame = {0};
  for (;;)
  {
    BaseType_t ret = xQueueReceive(rx_queue, &received_frame, portMAX_DELAY);

    if (ret == pdTRUE)
    {
      ID_Format id;
      id.id = received_frame.identifier;
      if (id.format.to_BoardType == Board_Type::Master_Board && id.format.from_BoardType == Board_Type::PowerBoard)
      {
        auto status = reinterpret_cast<PowerBoard_Status *>(received_frame.data);
        float msg;
        msg = status->Current;
        if (connected)
        {
          sendto(sock, &msg, sizeof(msg), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
        }
        //printf("Current %f\n", status->Current);
      }
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}
