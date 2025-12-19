#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include "driver/uart.h"

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
#define UART_BUF_SIZE   2048

static const char *TAG = "ESP32";

/* =========================================================
 * ESTRUCTURAS DE DATOS
 * ========================================================= */
typedef struct {
    int red_count;
    int green_count;
    int blue_count;
    int other_count;
    int total_count;
    int detection_count;
    char last_color[20];
    int conveyor_speed;
    int system_status;
    uint32_t timestamp;
    uint32_t last_stm32_time;
    int stm32_connected;
    int gui_connected;
    uint32_t last_detection_time;
} conveyor_data_t;

static conveyor_data_t conveyor_data = {
    .red_count = 0,
    .green_count = 0,
    .blue_count = 0,
    .other_count = 0,
    .total_count = 0,
    .detection_count = 0,
    .last_color = "NINGUNO",
    .conveyor_speed = 75,
    .system_status = 0,
    .timestamp = 0,
    .last_stm32_time = 0,
    .stm32_connected = 0,
    .gui_connected = 0,
    .last_detection_time = 0
};

/* =========================================================
 * VARIABLES GLOBALES
 * ========================================================= */
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static QueueHandle_t uart_queue;
static int tcp_client_sock = -1;
static SemaphoreHandle_t data_mutex;

/* =========================================================
 * DECLARACIONES DE FUNCIONES
 * ========================================================= */
static void process_stm32_data(const char* data);
static void send_to_stm32(const char* command);

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
        ESP_LOGI(TAG, "Conectando WiFi...");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        
        char esp_ip[16];
        uint8_t *ip_bytes = (uint8_t *)&event->ip_info.ip.addr;
        snprintf(esp_ip, sizeof(esp_ip), "%u.%u.%u.%u",
                 ip_bytes[0], ip_bytes[1], ip_bytes[2], ip_bytes[3]);
        
        ESP_LOGI(TAG, "WiFi CONECTADO - IP: %s:%d", esp_ip, PORT);
        
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

    ESP_LOGI(TAG, "WiFi: %s", WIFI_SSID);
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
    
    ESP_LOGI(TAG, "UART: %d bauds", UART_BAUD_RATE);
}

/* =========================================================
 * ENVIAR COMANDO A STM32
 * ========================================================= */
static void send_to_stm32(const char* command)
{
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "%s\n", command);
    uart_write_bytes(UART_PORT_NUM, buffer, strlen(buffer));
}

/* =========================================================
 * PROCESAR DATOS DEL STM32 - CORREGIDO
 * ========================================================= */
static void process_stm32_data(const char* data)
{
    char buffer[256];
    strncpy(buffer, data, sizeof(buffer)-1);
    buffer[sizeof(buffer)-1] = '\0';
    
    // Limpiar caracteres de control
    char *p = buffer;
    while (*p) {
        if (*p == '\r' || *p == '\n') *p = '\0';
        p++;
    }
    
    // Ignorar si está vacío
    if (strlen(buffer) == 0) return;
    
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    
    // Actualizar tiempo de última comunicación STM32
    conveyor_data.last_stm32_time = esp_log_timestamp();
    conveyor_data.stm32_connected = 1;
    
    int color_detected = 0;
    char detected_color[20] = {0};
    int old_total_count = conveyor_data.total_count; // Guardar valor anterior
    
    // =========================================================
    // 1. MANEJAR "DETECTADO"
    // =========================================================
    if (strstr(buffer, "DETECTADO") != NULL) {
        // Incrementar contador de detecciones
        conveyor_data.detection_count++;
        conveyor_data.last_detection_time = esp_log_timestamp();
        
        ESP_LOGI(TAG, "Detección recibida (Total: %d)", conveyor_data.detection_count);
        
        // Enviar a GUI inmediatamente
        if (tcp_client_sock > 0) {
            send(tcp_client_sock, "DETECTADO\n", 10, 0);
            ESP_LOGI(TAG, "-> GUI: DETECTADO");
        }
    }
    // =========================================================
    // 2. MANEJAR "COLOR:"
    // =========================================================
    else if (strncmp(buffer, "COLOR:", 6) == 0) {
        char* color_ptr = buffer + 6;
        
        // Limpiar posibles espacios extras
        while (*color_ptr == ' ') color_ptr++;
        
        // Copiar color a buffer seguro
        strncpy(detected_color, color_ptr, sizeof(detected_color)-1);
        detected_color[sizeof(detected_color)-1] = '\0';
        
        // Copiar a estructura
        strncpy(conveyor_data.last_color, detected_color, sizeof(conveyor_data.last_color)-1);
        conveyor_data.last_color[sizeof(conveyor_data.last_color)-1] = '\0';
        
        // Verificar si hubo detección reciente (dentro de 2 segundos)
        uint32_t current_time = esp_log_timestamp();
        uint32_t time_since_detection = current_time - conveyor_data.last_detection_time;
        
        if (time_since_detection <= 2000) {  // 2 segundos
            // Contar el color solo si hubo detección reciente
            if (strcmp(detected_color, "ROJO") == 0) {
                conveyor_data.red_count++;
                color_detected = 1;
            } else if (strcmp(detected_color, "VERDE") == 0) {
                conveyor_data.green_count++;
                color_detected = 1;
            } else if (strcmp(detected_color, "AZUL") == 0) {
                conveyor_data.blue_count++;
                color_detected = 1;
            } else {
                conveyor_data.other_count++;
                color_detected = 1;
            }
            
            if (color_detected) {
                conveyor_data.total_count++;
                ESP_LOGI(TAG, "Color %s CONTADO (Total: %d)", detected_color, conveyor_data.total_count);
            }
        } else {
            ESP_LOGW(TAG, "Color %s IGNORADO - Sin detección reciente", detected_color);
        }
        
        conveyor_data.timestamp = current_time;
        
        // Enviar mensaje de color a GUI
        if (tcp_client_sock > 0) {
            char color_msg[64];
            int len = snprintf(color_msg, sizeof(color_msg), "Color: %s\n", detected_color);
            if (len > 0 && len < sizeof(color_msg)) {
                send(tcp_client_sock, color_msg, len, 0);
                ESP_LOGI(TAG, "-> GUI: %s", color_msg);
            }
        }
        
        // Log cada 5 colores
        static int color_log_counter = 0;
        color_log_counter++;
        if (color_log_counter % 5 == 0) {
            ESP_LOGI(TAG, "Color: %s (Total: %d)", detected_color, conveyor_data.total_count);
        }
    }
    // =========================================================
    // 3. MANEJAR OTROS COMANDOS
    // =========================================================
    else if (strncmp(buffer, "SPEED:", 6) == 0) {
        int new_speed = atoi(buffer + 6);
        if (new_speed != conveyor_data.conveyor_speed) {
            conveyor_data.conveyor_speed = new_speed;
            if (conveyor_data.conveyor_speed < 0) conveyor_data.conveyor_speed = 0;
            if (conveyor_data.conveyor_speed > 100) conveyor_data.conveyor_speed = 100;
            ESP_LOGI(TAG, "Velocidad: %d%%", conveyor_data.conveyor_speed);
        }
    }
    else if (strncmp(buffer, "STATUS:", 7) == 0) {
        char* status = buffer + 7;
        int old_status = conveyor_data.system_status;
        
        if (strcmp(status, "RUNNING") == 0 || strcmp(status, "START") == 0) {
            conveyor_data.system_status = 1;
            if (old_status != 1) ESP_LOGI(TAG, "Sistema INICIADO");
        } else if (strcmp(status, "STOPPED") == 0 || strcmp(status, "STOP") == 0) {
            conveyor_data.system_status = 0;
            if (old_status != 0) ESP_LOGI(TAG, "Sistema DETENIDO");
        } else if (strcmp(status, "ERROR") == 0) {
            conveyor_data.system_status = 2;
            ESP_LOGE(TAG, "ERROR del sistema");
        }
    }
    else if (strncmp(buffer, "RESET", 5) == 0) {
        conveyor_data.red_count = 0;
        conveyor_data.green_count = 0;
        conveyor_data.blue_count = 0;
        conveyor_data.other_count = 0;
        conveyor_data.total_count = 0;
        conveyor_data.detection_count = 0;
        strcpy(conveyor_data.last_color, "NINGUNO");
        ESP_LOGI(TAG, "Contadores RESET");
    }
    else if (strncmp(buffer, "ERROR:", 6) == 0) {
        ESP_LOGE(TAG, "Error: %s", buffer + 6);
    }
    else if (strncmp(buffer, "GET_STATUS", 10) == 0) {
        // STM32 solicita estado del ESP32
        char response[128];
        snprintf(response, sizeof(response),
                "ESP32_STATUS|R:%d|V:%d|A:%d|T:%d|D:%d|S:%d|V:%d\n",
                conveyor_data.red_count, conveyor_data.green_count,
                conveyor_data.blue_count, conveyor_data.total_count,
                conveyor_data.detection_count,
                conveyor_data.system_status, conveyor_data.conveyor_speed);
        
        send_to_stm32(response);
    }
    else if (strncmp(buffer, "HELLO", 5) == 0 || strncmp(buffer, "STM32_READY", 11) == 0) {
        ESP_LOGI(TAG, "STM32 conectado");
        send_to_stm32("ESP32_READY");
    }
    else if (strlen(buffer) > 2) {
        // Verificar si es texto legible
        int is_printable = 1;
        for (int i = 0; buffer[i]; i++) {
            if (buffer[i] < 32 && buffer[i] != '\t') {
                is_printable = 0;
                break;
            }
        }
        if (is_printable) {
            ESP_LOGI(TAG, "STM32: %s", buffer);
        }
    }
    
    // Verificar si hubo cambio en el conteo
    int count_changed = (old_total_count != conveyor_data.total_count);
    
    xSemaphoreGive(data_mutex);
    
    // =========================================================
    // ENVIAR JSON ACTUALIZADO A GUI SI SE DETECTÓ UN COLOR
    // =========================================================
    if (tcp_client_sock > 0 && (color_detected || count_changed)) {
        char json_buffer[512];
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        
        snprintf(json_buffer, sizeof(json_buffer),
                "{\"type\":\"update\",\"red\":%d,\"green\":%d,\"blue\":%d,"
                "\"other\":%d,\"total\":%d,\"detections\":%d,\"last_color\":\"%s\","
                "\"speed\":%d,\"status\":%d}\n",
                conveyor_data.red_count, conveyor_data.green_count,
                conveyor_data.blue_count, conveyor_data.other_count,
                conveyor_data.total_count, conveyor_data.detection_count,
                conveyor_data.last_color,
                conveyor_data.conveyor_speed, conveyor_data.system_status);
        
        xSemaphoreGive(data_mutex);
        
        send(tcp_client_sock, json_buffer, strlen(json_buffer), 0);
        ESP_LOGI(TAG, "-> GUI: JSON enviado (ROJO:%d, VERDE:%d, AZUL:%d, OTRO:%d, TOTAL:%d)", 
                conveyor_data.red_count, conveyor_data.green_count,
                conveyor_data.blue_count, conveyor_data.other_count,
                conveyor_data.total_count);
    }
}

/* =========================================================
 * TAREA DE LECTURA UART
 * ========================================================= */
static void uart_read_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Iniciando comunicación UART...");
    
    // Esperar y enviar ready
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    send_to_stm32("ESP32_READY");
    ESP_LOGI(TAG, "Esperando STM32...");
    
    uint8_t data[128];
    char line_buffer[64];
    int line_index = 0;
    
    while (1) {
        // Leer con timeout
        int data_len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(100));
        
        if (data_len > 0) {
            data[data_len] = '\0';
            
            for (int i = 0; i < data_len; i++) {
                char c = data[i];
                
                // Filtrar caracteres no ASCII
                if (c == 0 || c == 0xFF || (c < 32 && c != '\t' && c != '\n' && c != '\r')) {
                    continue;
                }
                
                if (c == '\n' || c == '\r') {
                    if (line_index > 0) {
                        line_buffer[line_index] = '\0';
                        process_stm32_data(line_buffer);
                        line_index = 0;
                    }
                } else if (line_index < sizeof(line_buffer) - 1) {
                    line_buffer[line_index++] = c;
                }
            }
        }
        
        // Verificar conexión STM32
        static uint32_t last_check = 0;
        uint32_t now = xTaskGetTickCount();
        
        if (now - last_check > 30000 / portTICK_PERIOD_MS) {
            xSemaphoreTake(data_mutex, portMAX_DELAY);
            
            uint32_t current_time = esp_log_timestamp();
            if (current_time - conveyor_data.last_stm32_time > 30000) {
                if (conveyor_data.stm32_connected) {
                    conveyor_data.stm32_connected = 0;
                    ESP_LOGW(TAG, "STM32 desconectado");
                    
                    if (tcp_client_sock > 0) {
                        char disconnect_msg[] = "{\"type\":\"alert\",\"msg\":\"STM32 desconectado\"}\n";
                        send(tcp_client_sock, disconnect_msg, strlen(disconnect_msg), 0);
                    }
                }
            }
            
            xSemaphoreGive(data_mutex);
            last_check = now;
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

/* =========================================================
 * MANEJO DE CLIENTE GUI
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
    conveyor_data.gui_connected = 1;
    
    struct timeval timeout = {.tv_sec = 30, .tv_usec = 0};
    setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(client_sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    
    // Mensaje de bienvenida
    char init_msg[256];
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    snprintf(init_msg, sizeof(init_msg),
            "{\"type\":\"init\",\"stm32\":%d,\"speed\":%d,\"status\":%d,\"detections\":%d}\n",
            conveyor_data.stm32_connected, conveyor_data.conveyor_speed, 
            conveyor_data.system_status, conveyor_data.detection_count);
    xSemaphoreGive(data_mutex);
    
    send(client_sock, init_msg, strlen(init_msg), 0);
    
    char buffer[128];
    while (1) {
        int len = recv(client_sock, buffer, sizeof(buffer)-1, 0);
        if (len <= 0) break;
        
        buffer[len] = '\0';
        
        // Limpiar
        char *p = buffer;
        while (*p) {
            if (*p == '\r' || *p == '\n') *p = '\0';
            p++;
        }
        
        // Log comandos importantes
        if (strcmp(buffer, "START") == 0 || strcmp(buffer, "STOP") == 0 || 
            strncmp(buffer, "SET_SPEED:", 10) == 0) {
            ESP_LOGI(TAG, "GUI: %s", buffer);
        }
        
        if (strcmp(buffer, "GET_DATA") == 0) {
            char json[256];
            xSemaphoreTake(data_mutex, portMAX_DELAY);
            snprintf(json, sizeof(json),
                    "{\"type\":\"data\",\"red\":%d,\"green\":%d,\"blue\":%d,"
                    "\"other\":%d,\"total\":%d,\"detections\":%d,\"color\":\"%s\",\"speed\":%d}\n",
                    conveyor_data.red_count, conveyor_data.green_count,
                    conveyor_data.blue_count, conveyor_data.other_count,
                    conveyor_data.total_count, conveyor_data.detection_count,
                    conveyor_data.last_color, 
                    conveyor_data.conveyor_speed);
            xSemaphoreGive(data_mutex);
            send(client_sock, json, strlen(json), 0);
        }
        else if (strcmp(buffer, "START") == 0) {
            send_to_stm32("START");
            send(client_sock, "{\"type\":\"ok\",\"cmd\":\"start\"}\n", 30, 0);
        }
        else if (strcmp(buffer, "STOP") == 0) {
            send_to_stm32("STOP");
            send(client_sock, "{\"type\":\"ok\",\"cmd\":\"stop\"}\n", 29, 0);
        }
        else if (strncmp(buffer, "SET_SPEED:", 10) == 0) {
            int speed = atoi(buffer + 10);
            if (speed >= 0 && speed <= 100) {
                char cmd[32];
                snprintf(cmd, sizeof(cmd), "SPEED:%d", speed);
                send_to_stm32(cmd);
                xSemaphoreTake(data_mutex, portMAX_DELAY);
                conveyor_data.conveyor_speed = speed;
                xSemaphoreGive(data_mutex);
                send(client_sock, "{\"type\":\"ok\",\"cmd\":\"speed\"}\n", 30, 0);
            }
        }
        else if (strcmp(buffer, "RESET_COUNTERS") == 0) {
            send_to_stm32("RESET");
            xSemaphoreTake(data_mutex, portMAX_DELAY);
            conveyor_data.red_count = 0;
            conveyor_data.green_count = 0;
            conveyor_data.blue_count = 0;
            conveyor_data.other_count = 0;
            conveyor_data.total_count = 0;
            conveyor_data.detection_count = 0;
            strcpy(conveyor_data.last_color, "NINGUNO");
            xSemaphoreGive(data_mutex);
            send(client_sock, "{\"type\":\"ok\",\"cmd\":\"reset\"}\n", 30, 0);
        }
        else if (strcmp(buffer, "GET_STATUS") == 0) {
            char status_msg[128];
            xSemaphoreTake(data_mutex, portMAX_DELAY);
            snprintf(status_msg, sizeof(status_msg),
                    "{\"type\":\"status\",\"speed\":%d,\"stm32\":%d,\"detections\":%d}\n",
                    conveyor_data.conveyor_speed, conveyor_data.stm32_connected,
                    conveyor_data.detection_count);
            xSemaphoreGive(data_mutex);
            send(client_sock, status_msg, strlen(status_msg), 0);
        }
        else if (strcmp(buffer, "PING_STM32") == 0) {
            send_to_stm32("PING");
            send(client_sock, "{\"type\":\"ping_sent\"}\n", 22, 0);
        }
        else {
            send(client_sock, "{\"type\":\"error\",\"msg\":\"cmd invalido\"}\n", 40, 0);
        }
    }
    
    tcp_client_sock = -1;
    conveyor_data.gui_connected = 0;
    close(client_sock);
    ESP_LOGI(TAG, "GUI desconectada");
}

/* =========================================================
 * TAREA SERVIDOR TCP
 * ========================================================= */
static void tcp_server_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Esperando WiFi...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    int server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (server_sock < 0) {
        ESP_LOGE(TAG, "Error socket");
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
        ESP_LOGE(TAG, "Error bind");
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

    ESP_LOGI(TAG, "Servidor listo puerto %d", PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &client_len);

        if (client_sock < 0) {
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
    // Configurar logs
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("ESP32", ESP_LOG_INFO);
    
    printf("\n\n");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Sistema Cinta Transportadora");
    ESP_LOGI(TAG, "Con detección por sensor");
    ESP_LOGI(TAG, "Web: puerto %d", PORT);
    ESP_LOGI(TAG, "========================================");
    
    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    
    // Crear mutex
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        ESP_LOGE(TAG, "Error mutex");
        return;
    }
    
    // Inicializar
    wifi_init_sta();
    uart_init();
    
    // Crear tareas
    xTaskCreate(uart_read_task, "uart", 4096, NULL, 5, NULL);
    xTaskCreate(tcp_server_task, "server", 4096, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "Sistema listo");
    
    // Loop principal
    int counter = 0;
    while (1) {
        vTaskDelay(60000 / portTICK_PERIOD_MS);
        
        // Mostrar estado resumido
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        ESP_LOGI(TAG, "Estado [%d]: STM32=%s, GUI=%s, Detecciones=%d, Cajas=%d", 
                counter++,
                conveyor_data.stm32_connected ? "OK" : "NO",
                conveyor_data.gui_connected ? "OK" : "NO",
                conveyor_data.detection_count,
                conveyor_data.total_count);
        xSemaphoreGive(data_mutex);
    }
}