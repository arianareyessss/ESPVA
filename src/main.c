#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include "driver/uart.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

/* =========================================================
 * CONFIGURACION
 * ========================================================= */
#define WIFI_SSID       "PATI123"
#define WIFI_PASS       "riego123"
#define PORT            8080

// Configuracion UART 
#define UART_PORT_NUM   UART_NUM_2
#define UART_BAUD_RATE  115200
#define UART_RX_PIN     16
#define UART_TX_PIN     17
#define UART_BUF_SIZE   1024

static const char *TAG = "ESP32_FAKE";

/* =========================3-3================================
 * ESTRUCTURAS DE DATOS
 * ========================================================= */
typedef struct {
    int red_count;
    int green_count;
    int blue_count;
    int other_count;
    int total_count;
    char last_color[20];
    int conveyor_speed;
    int system_status;
    uint32_t timestamp;
    int fake_active;
} conveyor_data_t;

static conveyor_data_t conveyor_data = {
    .red_count = 0,
    .green_count = 0,
    .blue_count = 0,
    .other_count = 0,
    .total_count = 0,
    .last_color = "NINGUNO",
    .conveyor_speed = 75,
    .system_status = 0,
    .timestamp = 0,
    .fake_active = 0
};

/* =========================================================
 * VARIABLES GLOBALES
 * ========================================================= */
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static QueueHandle_t uart_queue;
static int tcp_client_sock = -1;
static SemaphoreHandle_t data_mutex;
static esp_timer_handle_t fake_data_timer;

/* =========================================================
 * DECLARACIONES DE FUNCIONES (para resolver orden)
 * ========================================================= */
static void process_stm32_data(const char* data);
static void generate_fake_detection(void);
static void fake_data_timer_callback(void* arg);
static void start_fake_data_timer(void);
static void stop_fake_data_timer(void);

/* =========================================================
 * WIFI
 * ========================================================= */
static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Conectando a WiFi...");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        
        char esp_ip[16];
        uint8_t *ip_bytes = (uint8_t *)&event->ip_info.ip.addr;
        snprintf(esp_ip, sizeof(esp_ip), "%u.%u.%u.%u",
                 ip_bytes[0], ip_bytes[1], ip_bytes[2], ip_bytes[3]);
        
        ESP_LOGI(TAG, "================================================");
        ESP_LOGI(TAG, "WIFI CONECTADO");
        ESP_LOGI(TAG, "IP del ESP32: %s", esp_ip);
        ESP_LOGI(TAG, "Puerto: %d", PORT);
        ESP_LOGI(TAG, "================================================");
        ESP_LOGI(TAG, "Conecta GUI a: %s:%d", esp_ip, PORT);
        
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                               ESP_EVENT_ANY_ID,
                                               &wifi_event_handler,
                                               NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                               IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler,
                                               NULL));

    wifi_config_t wifi_config = { 0 };
    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi inicializado. SSID: %s", WIFI_SSID);
}

/* =========================================================
 * INICIALIZACION UART
 * ========================================================= */
static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, UART_BUF_SIZE,
                                        10, &uart_queue, 0));
    
    ESP_LOGI(TAG, "UART listo: Baud %d", UART_BAUD_RATE);
}

/* =========================================================
 * PROCESAR DATOS
 * ========================================================= */
static void process_stm32_data(const char* data)
{
    ESP_LOGI(TAG, "Dato: %s", data);
    
    char buffer[256];
    strncpy(buffer, data, sizeof(buffer)-1);
    buffer[sizeof(buffer)-1] = '\0';
    
    char *newline = strchr(buffer, '\r');
    if (newline) *newline = '\0';
    newline = strchr(buffer, '\n');
    if (newline) *newline = '\0';
    
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    
    if (strncmp(buffer, "COLOR:", 6) == 0) {
        char* color = buffer + 6;
        strncpy(conveyor_data.last_color, color, sizeof(conveyor_data.last_color)-1);
        conveyor_data.last_color[sizeof(conveyor_data.last_color)-1] = '\0';
        
        if (strcmp(color, "ROJO") == 0) {
            conveyor_data.red_count++;
        } else if (strcmp(color, "VERDE") == 0) {
            conveyor_data.green_count++;
        } else if (strcmp(color, "AZUL") == 0) {
            conveyor_data.blue_count++;
        } else {
            conveyor_data.other_count++;
        }
        
        conveyor_data.total_count++;
        conveyor_data.timestamp = esp_log_timestamp();
        
        ESP_LOGI(TAG, "Caja: %s | ROJO:%d VERDE:%d AZUL:%d TOTAL:%d", 
                color, conveyor_data.red_count, conveyor_data.green_count,
                conveyor_data.blue_count, conveyor_data.total_count);
    }
    else if (strncmp(buffer, "SPEED:", 6) == 0) {
        conveyor_data.conveyor_speed = atoi(buffer + 6);
        if (conveyor_data.conveyor_speed < 0) conveyor_data.conveyor_speed = 0;
        if (conveyor_data.conveyor_speed > 100) conveyor_data.conveyor_speed = 100;
        ESP_LOGI(TAG, "Velocidad: %d%%", conveyor_data.conveyor_speed);
    }
    else if (strncmp(buffer, "STATUS:", 7) == 0) {
        char* status = buffer + 7;
        if (strcmp(status, "RUNNING") == 0) {
            conveyor_data.system_status = 1;
            conveyor_data.fake_active = 1;
            ESP_LOGI(TAG, "Sistema INICIADO - Datos fake ACTIVADOS");
        } else if (strcmp(status, "STOPPED") == 0) {
            conveyor_data.system_status = 0;
            conveyor_data.fake_active = 0;
            ESP_LOGI(TAG, "Sistema DETENIDO - Datos fake DESACTIVADOS");
        } else if (strcmp(status, "ERROR") == 0) {
            conveyor_data.system_status = 2;
            ESP_LOGE(TAG, "Estado ERROR");
        }
    }
    else if (strncmp(buffer, "ERROR:", 6) == 0) {
        ESP_LOGE(TAG, "Error: %s", buffer + 6);
    }
    
    xSemaphoreGive(data_mutex);
    
    if (tcp_client_sock > 0) {
        char json_buffer[512];
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        
        snprintf(json_buffer, sizeof(json_buffer),
                "{\"type\":\"sensor_data\",\"red\":%d,\"green\":%d,\"blue\":%d,\"other\":%d,"
                "\"total\":%d,\"last_color\":\"%s\",\"speed\":%d,\"status\":%d,\"fake\":%d,\"timestamp\":%lu}\n",
                conveyor_data.red_count, conveyor_data.green_count,
                conveyor_data.blue_count, conveyor_data.other_count,
                conveyor_data.total_count, conveyor_data.last_color,
                conveyor_data.conveyor_speed, conveyor_data.system_status,
                conveyor_data.fake_active,
                (unsigned long)conveyor_data.timestamp);
        
        xSemaphoreGive(data_mutex);
        
        send(tcp_client_sock, json_buffer, strlen(json_buffer), 0);
    }
}

/* =========================================================
 * GENERAR DATOS FAKE
 * ========================================================= */
static void generate_fake_detection(void)
{
    if (conveyor_data.fake_active && conveyor_data.system_status == 1) {
        static int fake_counter = 0;
        fake_counter++;
        
        const char* colors[] = {"ROJO", "VERDE", "AZUL", "OTRO"};
        const char* color = colors[fake_counter % 4];
        
        char fake_message[64];
        snprintf(fake_message, sizeof(fake_message), "COLOR:%s\n", color);
        
        ESP_LOGI(TAG, "Dato FAKE: %s", fake_message);
        
        process_stm32_data(fake_message);
        
        if (fake_counter % 5 == 0) {
            int new_speed = 60 + (fake_counter % 40);
            snprintf(fake_message, sizeof(fake_message), "SPEED:%d\n", new_speed);
            process_stm32_data(fake_message);
        }
    }
}

/* =========================================================
 * TIMER PARA DATOS FAKE
 * ========================================================= */
static void fake_data_timer_callback(void* arg)
{
    generate_fake_detection();
}

static void start_fake_data_timer(void)
{
    esp_timer_create_args_t timer_args = {
        .callback = &fake_data_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "fake_timer"
    };
    
    esp_timer_create(&timer_args, &fake_data_timer);
    esp_timer_start_periodic(fake_data_timer, 2000000);
}

static void stop_fake_data_timer(void)
{
    if (fake_data_timer) {
        esp_timer_stop(fake_data_timer);
        esp_timer_delete(fake_data_timer);
        fake_data_timer = NULL;
    }
}

/* =========================================================
 * TAREA UART
 * ========================================================= */
static void uart_read_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Tarea UART iniciada");
    
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    process_stm32_data("STATUS:STOPPED\n");
    process_stm32_data("SPEED:75\n");
    
    uint8_t data[UART_BUF_SIZE];
    int data_len;
    
    while (1) {
        data_len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(100));
        
        if (data_len > 0) {
            data[data_len] = '\0';
            
            char *line = strtok((char*)data, "\n");
            while (line != NULL) {
                if (strlen(line) > 0) {
                    process_stm32_data(line);
                }
                line = strtok(NULL, "\n");
            }
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/* =========================================================
 * TAREA SERVIDOR TCP
 * ========================================================= */
static void handle_gui_client(int client_sock)
{
    char client_ip[16];
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    
    getpeername(client_sock, (struct sockaddr*)&client_addr, &addr_len);
    inet_ntoa_r(client_addr.sin_addr, client_ip, sizeof(client_ip));
    
    ESP_LOGI(TAG, "GUI conectada: %s", client_ip);
    tcp_client_sock = client_sock;
    
    struct timeval timeout = {.tv_sec = 50, .tv_usec = 0};
    setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(client_sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    
    char init_msg[512];
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    
    snprintf(init_msg, sizeof(init_msg),
            "{\"type\":\"init\",\"message\":\"ESP32 Listo\","
            "\"red\":%d,\"green\":%d,\"blue\":%d,\"other\":%d,\"total\":%d,"
            "\"speed\":%d,\"status\":%d,\"fake\":1}\n",
            conveyor_data.red_count, conveyor_data.green_count,
            conveyor_data.blue_count, conveyor_data.other_count,
            conveyor_data.total_count, conveyor_data.conveyor_speed,
            conveyor_data.system_status);
    
    xSemaphoreGive(data_mutex);
    
    send(client_sock, init_msg, strlen(init_msg), 0);
    
    char buffer[256];
    while (1) {
        int len = recv(client_sock, buffer, sizeof(buffer)-1, 0);
        if (len <= 0) break;
        
        buffer[len] = '\0';
        
        char *newline = strchr(buffer, '\r');
        if (newline) *newline = '\0';
        newline = strchr(buffer, '\n');
        if (newline) *newline = '\0';
        
        ESP_LOGI(TAG, "GUI Comando: %s", buffer);
        
        if (strcmp(buffer, "GET_DATA") == 0) {
            char json_buffer[512];
            xSemaphoreTake(data_mutex, portMAX_DELAY);
            
            snprintf(json_buffer, sizeof(json_buffer),
                    "{\"type\":\"data\",\"red\":%d,\"green\":%d,\"blue\":%d,\"other\":%d,"
                    "\"total\":%d,\"last_color\":\"%s\",\"speed\":%d,\"status\":%d,\"fake\":%d}\n",
                    conveyor_data.red_count, conveyor_data.green_count,
                    conveyor_data.blue_count, conveyor_data.other_count,
                    conveyor_data.total_count, conveyor_data.last_color,
                    conveyor_data.conveyor_speed, conveyor_data.system_status,
                    conveyor_data.fake_active);
            
            xSemaphoreGive(data_mutex);
            
            send(client_sock, json_buffer, strlen(json_buffer), 0);
        }
        else if (strcmp(buffer, "START") == 0) {
            process_stm32_data("STATUS:RUNNING");
            start_fake_data_timer();
            send(client_sock, "{\"type\":\"start_ack\",\"message\":\"Sistema iniciado\"}\n", 
                 strlen("{\"type\":\"start_ack\",\"message\":\"Sistema iniciado\"}\n"), 0);
        }
        else if (strcmp(buffer, "STOP") == 0) {
            process_stm32_data("STATUS:STOPPED");
            stop_fake_data_timer();
            send(client_sock, "{\"type\":\"stop_ack\",\"message\":\"Sistema detenido\"}\n", 
                 strlen("{\"type\":\"stop_ack\",\"message\":\"Sistema detenido\"}\n"), 0);
        }
        else if (strcmp(buffer, "RESET_COUNTERS") == 0) {
            xSemaphoreTake(data_mutex, portMAX_DELAY);
            conveyor_data.red_count = 0;
            conveyor_data.green_count = 0;
            conveyor_data.blue_count = 0;
            conveyor_data.other_count = 0;
            conveyor_data.total_count = 0;
            strcpy(conveyor_data.last_color, "NINGUNO");
            xSemaphoreGive(data_mutex);
            
            send(client_sock, "{\"type\":\"reset_ack\",\"message\":\"Contadores reseteados\"}\n", 
                 strlen("{\"type\":\"reset_ack\",\"message\":\"Contadores reseteados\"}\n"), 0);
        }
        else if (strcmp(buffer, "GET_STATUS") == 0) {
            char status_msg[256];
            snprintf(status_msg, sizeof(status_msg),
                    "{\"type\":\"status\",\"message\":\"OK\",\"fake\":%d,\"speed\":%d}\n",
                    conveyor_data.fake_active, conveyor_data.conveyor_speed);
            send(client_sock, status_msg, strlen(status_msg), 0);
        }
        else {
            send(client_sock, "{\"type\":\"error\",\"message\":\"Comando no valido\"}\n", 
                 strlen("{\"type\":\"error\",\"message\":\"Comando no valido\"}\n"), 0);
        }
    }
    
    tcp_client_sock = -1;
    close(client_sock);
    ESP_LOGI(TAG, "GUI desconectada: %s", client_ip);
}

static void tcp_server_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Esperando WiFi...");
    xEventGroupWaitBits(wifi_event_group,
                        WIFI_CONNECTED_BIT,
                        pdFALSE,
                        pdTRUE,
                        portMAX_DELAY);

    int server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (server_sock < 0) {
        ESP_LOGE(TAG, "Error creando socket");
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(PORT);

    if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Error bind puerto %d", PORT);
        close(server_sock);
        vTaskDelete(NULL);
        return;
    }

    if (listen(server_sock, 1) < 0) {
        ESP_LOGE(TAG, "Error listen");
        close(server_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Servidor TCP iniciado puerto %d", PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &client_len);

        if (client_sock < 0) {
            ESP_LOGE(TAG, "Error aceptando conexion");
            continue;
        }

        if (tcp_client_sock > 0) {
            close(tcp_client_sock);
            tcp_client_sock = -1;
        }

        handle_gui_client(client_sock);
    }

    close(server_sock);
    vTaskDelete(NULL);
}

/* =========================================================
 * FUNCION PRINCIPAL
 * ========================================================= */
void app_main(void)
{
    printf("\n\n\n\n\n\n");
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "ESP32 FAKE DATA - CINTA TRANSPORTADORA");
    ESP_LOGI(TAG, "Version: 1.0 - Puerto %d", PORT);
    ESP_LOGI(TAG, "================================================");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        ESP_LOGE(TAG, "Error creando mutex");
        return;
    }
    
    wifi_init_sta();
    uart_init();
    
    xTaskCreate(uart_read_task, "uart_task", 4096, NULL, 5, NULL);
    xTaskCreate(tcp_server_task, "tcp_server", 8192, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Sistema iniciado. Esperando GUI...");
    
    int heartbeat = 0;
    while (1) {
        ESP_LOGI(TAG, "Sistema activo [%d] | GUI: %s | Fake: %s",
                heartbeat++,
                (tcp_client_sock > 0) ? "Conectada" : "Esperando",
                (conveyor_data.fake_active) ? "ACTIVO" : "INACTIVO");
        
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}