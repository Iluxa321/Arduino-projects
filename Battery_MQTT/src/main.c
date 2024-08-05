#include "main.h"

#define CONTROLLER_INIT {{0, 0, 0, 0, 0, true}, NULL, NULL, NULL, NULL}

#define CURRENT_PI  {0.8, 1.5*10e-3, 0, 90, 0, true}
#define VOLTAGE_PI  {0.8, 6.5*10e-3, 0, 50, 0, true}

#define ADC_U   ADC1_CHANNEL_4
#define ADC_I   ADC1_CHANNEL_5
#define I_VD    ADC1_CHANNEL_7
#define BUTTON  GPIO_NUM_4
#define RST_BUTTON GPIO_NUM_21
#define LED     GPIO_NUM_2
#define H_PWM   GPIO_NUM_5
#define L_PWM   GPIO_NUM_18
#define FAULT   GPIO_NUM_19
#define PWM_EN  GPIO_NUM_26
#define RST     GPIO_NUM_27

#define CURRENT 15.0
#define FRAME_SIZE  24
#define VREF    1107
#define GAIN1     5.255
#define GAIN2     1.516
#define RING_BUF_LEN    64




typedef struct mqttData {
    char topic[20];
    uint8_t topic_len;
    char data[10];
    uint8_t data_len;
}mqttData;

typedef enum dataCode {
    NONE,
    LOAD_VOLTAGE,
    LOAD_CURRENT,
    VD_CURRENT,
    RUN_STOP,
    INFO

}dataCode;

typedef enum controllerState {
    C_NONE,
    C_BUCK,
    C_BOOST
}controllerState;

typedef struct controllerData {
    dataCode code;
    int data;
}controllerData;

typedef struct controller_t{
    pi_t pi;
    xQueueHandle actual_data_queue;
    xQueueHandle desired_data_queue;
    void (*start) (void);
    void (*stop) (void);
}controller_t;


/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
esp_mqtt_client_handle_t client;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";
static const char *MQTT_TAG = "MQTT_EVENT";

static int s_retry_num = 0;

// float U_coef[3] = {0.4018, 6.2849, -3.1056};
// float U_coef[2] = {13.613, -6.5192};
float U_coef[2] = {13.592, -6.5213};
float I_coef[2] = {10.059*GAIN1, -2.515*10.059};
float Ivd_coef[2] = {10.059*GAIN2, -2.515*10.059};

int16_t buffer_u[RING_BUF_LEN] = {};
int16_t buffer_i[RING_BUF_LEN] = {};
int16_t buffer_duty[RING_BUF_LEN] = {};
controllerData c_data;

xQueueHandle xCurrentQueue;
xQueueHandle xVoltageQueue;
xQueueHandle xSetDataQueue;
xQueueHandle xPublishQueue;
xSemaphoreHandle xBinarySemaphore;
xSemaphoreHandle xPwmFault;




uint16_t str2Int(const char *s, uint16_t len) {
    uint16_t res = 0;
    for(int i = 0; i < len; i++) {
        res = res * 10 + s[i] - '0';
    }
    return res;
}

float piHandler(pi_t *pi, float setpoint, float current_point) {
    float err = setpoint - current_point;
    float P = err * pi->kp;
    pi->I = pi->I + err * pi->ki * pi->Si;
    float res = P + pi->I;
    float tmp_res = res;
    if(res > pi->S_max)       res = pi->S_max;
    else if(res < pi->S_min)  res = pi->S_min;
    pi->Si = (tmp_res == res) ? true : false;
    return res;
}

uint16_t ringFilterHandler(RingFilter *rb, int16_t data) {
    rb->sum = rb->sum - rb->p_buf[rb->i] + data;
    rb->p_buf[rb->i] = data;
    rb->i = (rb->i + 1) % rb->len;
    return rb->sum / rb->len;
}

float dataCalculation(float data, float *coef, uint16_t n) {
    float res = 0;
    for(int i = 0; i < n; i++) {
        res += coef[i] * (pow(data, n-1-i));
    }
    return res;
}

static void IRAM_ATTR gpio_isr_handler(void *arg) {

    xSemaphoreGiveFromISR(xPwmFault, NULL);
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(MQTT_TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        esp_mqtt_client_subscribe(client, "Charge/btn/", 0);
        esp_mqtt_client_subscribe(client, "Set_current/", 0);
        esp_mqtt_client_subscribe(client, "Controller_fun/", 0);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DATA");
        // printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        // printf("DATA=%.*s\r\n", event->data_len ,event->data);
        if(strncmp(event->topic, "Charge/btn/", event->topic_len) == 0) {
            xSemaphoreGive(xBinarySemaphore);
            
        }
        if(strncmp(event->topic, "Set_current/", event->topic_len) == 0) {
            float data = ((float) str2Int(event->data, event->data_len)) / 1000;
            xQueueSend(xSetDataQueue, (void*)&data, 0);
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(MQTT_TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGI(MQTT_TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGI(MQTT_TAG, "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
                     strerror(event->error_handle->esp_transport_sock_errno));
        } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGI(MQTT_TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
        } else {
            ESP_LOGW(MQTT_TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
        }
        break;
    default:
        ESP_LOGI(MQTT_TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void mqttPublishTask(void *pvParameter) {
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t) pvParameter;
    controllerData cntrl_data;
    char str_data[20];
    while(true) {
        xQueueReceive(xPublishQueue, (void *) &cntrl_data, portMAX_DELAY);
        sprintf(str_data, "%d", cntrl_data.data);
        switch (cntrl_data.code)
        {
        case NONE: {
            break;
        }
        case LOAD_VOLTAGE: {
            esp_mqtt_client_publish(client, "Plot/voltage/", (const char *) str_data, 0, 0, 0);
            break;
        }
        case LOAD_CURRENT: {
            esp_mqtt_client_publish(client, "Plot/current/", (const char *) str_data, 0, 0, 0);
            break;
        }
        case VD_CURRENT: {
            esp_mqtt_client_publish(client, "Plot/current_vd/", (const char *) str_data, 0, 0, 0);
            break;
        }
        case RUN_STOP: {
            
            esp_mqtt_client_publish(client, "Charge/state/", (const char *) str_data, 0, 0, 0);
            break;
        }
        case INFO: {
            if(cntrl_data.data == 0) {
                esp_mqtt_client_publish(client, "Info/", (const char *) "RESET", 0, 0, 0);
            }
            if(cntrl_data.data == 1) {
                esp_mqtt_client_publish(client, "Info/", (const char *) "WORK", 0, 0, 0);
            }
        }
        }

    }
}


esp_mqtt_client_handle_t mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URI,
        .port = 8883,
        .username = "Iluxa",
        .password = "123456789",
        .client_id = "ESP_CLIENT",
        .cert_pem = ca_root,
    };

    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    return client;
}

void wifi_init_sta(void)
{
    
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}



void buttonReadTask(void *pvParameter) {
    uint8_t button_cnt = 0;
    uint8_t rst_button_cnt = 0;
    while(true) {
        if(!gpio_get_level(BUTTON)) {
            button_cnt++;
            if(button_cnt == 10) {
                
                if(xSemaphoreGive(xBinarySemaphore) == pdFALSE) {
                    printf("Button Error\n");
                }
            }
        }
        else {
            button_cnt = 0;
        }
        if(!gpio_get_level(RST_BUTTON)) {
            rst_button_cnt++;
            if(rst_button_cnt == 10) {
                
                gpio_set_level(RST, 1);
                vTaskDelay(1*portTICK_RATE_MS);
                gpio_set_level(RST, 0);
            }
        }
        else {
            rst_button_cnt = 0;
        }
        vTaskDelay(10*portTICK_RATE_MS);
    }
}


void buckBoostController(void *pvParameter) {
    controller_t controller;
    memcpy((void *) &controller, pvParameter, sizeof(controller_t));
    uint8_t dc_mode = true; 
    float desired_data;
    float actual_data;
    float ref = 2;
    controllerData cntrl_data;
    xQueueReceive(controller.desired_data_queue, &desired_data, portMAX_DELAY);
    // printf("DATA = %f\n", desired_data);
    while(1) {
        xQueueReceive(controller.actual_data_queue, &actual_data, portMAX_DELAY);
        xQueueReceive(controller.desired_data_queue, &desired_data, 0);
        portBASE_TYPE xbuttonStatus = xSemaphoreTake(xBinarySemaphore, 0);
        portBASE_TYPE err = xSemaphoreTake(xPwmFault, 0);
        actual_data = abs(actual_data);
        // printf(">ADC_U:%f\n", actual_data);
        if(xbuttonStatus == pdTRUE) {
            if(dc_mode == true) {
                cntrl_data.code = RUN_STOP;
                cntrl_data.data = 0;
                xQueueSend(xPublishQueue, (void *) &cntrl_data, 0);
                gpio_set_level(LED, 0);
                controller.stop();
                dc_mode = false;
            }
            else {
                cntrl_data.code = RUN_STOP;
                cntrl_data.data = 1;
                xQueueSend(xPublishQueue, (void *) &cntrl_data, 0);
                cntrl_data.code = INFO;
                cntrl_data.data = 1;
                xQueueSend(xPublishQueue, (void *) &cntrl_data, 0);
                gpio_set_level(LED, 1);
                controller.start();
                dc_mode = true;
            }
        }
        if(err == pdTRUE) {
            controller.stop();
            gpio_set_level(LED, 0);
            dc_mode = false;
        }
        if(dc_mode == true) {
            if(ref < desired_data)
                ref += 0.2;
            else if(ref > desired_data) {
                ref -= 0.2;
            }
            if(abs(ref - desired_data) < 0.25) {
                ref = desired_data;
            }
            float duty = piHandler(&controller.pi, ref, actual_data);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, duty);
            printf(">DUTY:%f\n", duty);
        }
        else {
            controller.pi.I = 5;
            ref = 2;
        }

    }
}

void dcControllerTask(void *pvParameter) {
    pi_t buck_pi = {0.8, 1.5*10e-3, 0, 50, 0, true};
    uint8_t dc_mode = true; 
    float i_ref = 5;
    float current_data;
    float set_current = CURRENT;
    controllerData cntrl_data;
    while(true) {
        xQueueReceive(xCurrentQueue, &current_data, portMAX_DELAY);
        xQueueReceive(xSetDataQueue, &set_current, 0);
        // printf("CUR = %f \n", set_current);
        portBASE_TYPE xbuttonStatus = xSemaphoreTake(xBinarySemaphore, 0);
        portBASE_TYPE err = xSemaphoreTake(xPwmFault, 0);
        if(xbuttonStatus == pdTRUE) {
            if(dc_mode == true) {
                cntrl_data.code = RUN_STOP;
                cntrl_data.data = 0;
                xQueueSend(xPublishQueue, (void *) &cntrl_data, 0);
                gpio_set_level(LED, 0);
                pwmStop();
                dc_mode = false;
            }
            else {
                cntrl_data.code = RUN_STOP;
                cntrl_data.data = 1;
                xQueueSend(xPublishQueue, (void *) &cntrl_data, 0);
                cntrl_data.code = INFO;
                cntrl_data.data = 1;
                xQueueSend(xPublishQueue, (void *) &cntrl_data, 0);
                gpio_set_level(LED, 1);
                pwmStart();
                dc_mode = true;
            }
        }
        if(err == pdTRUE) {
            pwmStop();
            gpio_set_level(LED, 0);
            dc_mode = false;
        }
        if(dc_mode == true) {
            if(i_ref < set_current)
                i_ref += 0.2;
            else if(i_ref > set_current) {
                i_ref -= 0.2;
            }
            if(abs(i_ref - set_current) < 0.25) {
                i_ref = set_current;
            }
            float duty = piHandler(&buck_pi, i_ref, current_data);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, duty);
            // printf(">DUTY:%f\n", duty);
        }
        else {
            buck_pi.I = 5;
            i_ref = 5;
        }
        
    }
}

void adcReadTask(void *pvParameter) {
    esp_adc_cal_characteristics_t adc_chars, ivd_chars;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_10, 1100, &adc_chars); // используется ESP_ADC_CAL_VAL_EFUSE_VREF
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_10, 1100, &ivd_chars); // используется ESP_ADC_CAL_VAL_EFUSE_VREF
    RingFilter rf_u = {RING_BUF_LEN, 0, 0, buffer_u};
    RingFilter rf_i = {RING_BUF_LEN, 0, 0, buffer_i};
    RingFilter rf_ivd = {RING_BUF_LEN, 0, 0, buffer_duty};
    memset(rf_u.p_buf, 0, RING_BUF_LEN);
    memset(rf_i.p_buf, 0, RING_BUF_LEN);
    memset(rf_ivd.p_buf, 0, RING_BUF_LEN);
    TickType_t xLastWakeTime = xTaskGetTickCount(); 
    uint8_t cnt = 0;
    controllerData cntrl_data;
    while(1) {
        uint16_t voltage = 0;
        uint16_t current = 0;
        uint16_t i_vd = 0;
        for(int i = 0; i < 8; i++) {
            voltage += (uint16_t)adc1_get_raw(ADC_U);
            current += (uint16_t)adc1_get_raw(ADC_I);
            i_vd += (uint16_t)adc1_get_raw(I_VD);
        }
        voltage = voltage >> 3;
        current = current >> 3;
        i_vd = i_vd >> 3;
        voltage = ringFilterHandler(&rf_u, (int16_t)voltage);
        current = ringFilterHandler(&rf_i, (int16_t)current);
        i_vd = ringFilterHandler(&rf_ivd, (int16_t)i_vd);
        float u_data = (float)esp_adc_cal_raw_to_voltage(voltage,  &adc_chars) / 1000.0;
        float i_data = (float)esp_adc_cal_raw_to_voltage(current,  &adc_chars) / 1000.0;
        float ivd_data = (float)esp_adc_cal_raw_to_voltage(i_vd,  &ivd_chars) / 1000.0;
        u_data = dataCalculation(u_data, U_coef, 2);
        i_data = dataCalculation(i_data, I_coef, 2);
        ivd_data = dataCalculation(ivd_data, Ivd_coef, 2);
        xQueueSend(xCurrentQueue, (void*)&i_data, 0);
        xQueueSend(xVoltageQueue, (void*)&u_data, 0);

        if(cnt == 1) {
            cntrl_data.code = LOAD_VOLTAGE;
            cntrl_data.data = (int)(u_data*1000);
            xQueueSend(xPublishQueue, (void *) &cntrl_data, 0);
            
        }
        else if(cnt == 10) {
            cntrl_data.code = LOAD_CURRENT;
            cntrl_data.data = (int)(i_data*1000);
            xQueueSend(xPublishQueue, (void *) &cntrl_data, 0);
        }
        else if(cnt == 20) {
            cntrl_data.code = VD_CURRENT;
            cntrl_data.data = (int)(ivd_data*1000);
            xQueueSend(xPublishQueue, (void *) &cntrl_data, 0);
            
            cnt = 0;
        }


        cnt++;

        printf(">ADC_U:%f\n", u_data);
        printf(">ADC_I:%f\n", i_data);
        printf(">ADC_IVD:%f\n", ivd_data);
        vTaskDelayUntil(&xLastWakeTime, 10*portTICK_RATE_MS);
    }
}

void pwmInit(void) {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, H_PWM);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, L_PWM);
    // Защита по току
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_FAULT_0, FAULT);
    // gpio_pullup_en(FAULT);
    // gpio_set_intr_type(FAULT, GPIO_INTR_NEGEDGE);
    // gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    

    mcpwm_config_t pwm_config = {
        .frequency = 60000, 
        .cmpr_a = 0.0,     // duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    // ESP_ERROR_CHECK(mcpwm_fault_init(MCPWM_UNIT_0, MCPWM_LOW_LEVEL_TGR, MCPWM_SELECT_F0));
    // ESP_ERROR_CHECK(mcpwm_fault_set_oneshot_mode(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_F0, MCPWM_ACTION_FORCE_HIGH, MCPWM_ACTION_FORCE_HIGH));
    // gpio_isr_handler_add(FAULT, gpio_isr_handler, (void*) FAULT);
    // printf("Freq = %d\n", mcpwm_get_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0));
    pwmStop();
   
}

void pwmStart(void) {
    
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);  
    mcpwm_fault_set_oneshot_mode(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_F0, MCPWM_ACTION_FORCE_LOW, MCPWM_ACTION_FORCE_LOW);
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 50, 50);
}

void pwmStart2(void) {
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);  
    mcpwm_fault_set_oneshot_mode(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_F0, MCPWM_ACTION_FORCE_LOW, MCPWM_ACTION_FORCE_LOW);
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_LOW_COMPLIMENT_MODE, 50, 50);
}


void pwmStop(void) {
    mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B);
}

void gpioInit(void) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1 << BUTTON) | (1 << RST_BUTTON);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    gpio_set_direction(LED, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(PWM_EN, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(RST, GPIO_MODE_INPUT_OUTPUT);

}


// void dmaAdcInit(adc_channel_t *channel, uint32_t channel_num) {

//     uint32_t adc1_chan_mask = BIT(4) | BIT(5) | BIT(7);

//     adc_digi_init_config_t adc_dma_config = {
//         .max_store_buf_size = 10000,
//         .conv_num_each_intr = 48,
//         .adc1_chan_mask = adc1_chan_mask,
//         .adc2_chan_mask = 0,
//     };

//     ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));
//      adc_digi_configuration_t dig_cfg = {
//         .conv_limit_en = true,
//         .conv_limit_num = 40,
//         .sample_freq_hz = 10 * 1000,
//         .conv_mode = ADC_CONV_SINGLE_UNIT_1,
//         .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
//     };
//     dig_cfg.pattern_num = channel_num;
//     adc_digi_pattern_config_t adc_pattern[channel_num];
    
//     for(int i = 0; i < channel_num; i++) {
//         adc_pattern[i].atten = ADC_ATTEN_DB_0;
//         adc_pattern[i].bit_width = 10;
//         adc_pattern[i].channel = channel[i];
//         adc_pattern[i].unit = 0;                // UNIT 1
//     }
//     dig_cfg.adc_pattern = adc_pattern;
//     ESP_ERROR_CHECK(adc_digi_controller_configure(&dig_cfg));

// }

void adcInit(void) {
    adc_set_clk_div(1);
    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(ADC_U, ADC_ATTEN_DB_0);   // GPIO 32
    adc1_config_channel_atten(ADC_I, ADC_ATTEN_DB_0);   // GPIO 33
    adc1_config_channel_atten(I_VD, ADC_ATTEN_DB_11);   // GPIO 35
}

void app_main() {
    
    xCurrentQueue = xQueueCreate(5, sizeof(float));
    xVoltageQueue = xQueueCreate(5, sizeof(float));
    xSetDataQueue = xQueueCreate(1, sizeof(float));
    xPublishQueue = xQueueCreate(10, sizeof(controllerData));
    vSemaphoreCreateBinary(xBinarySemaphore);
    vSemaphoreCreateBinary(xPwmFault);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    client = mqtt_app_start();

    // xTaskCreate(grafUpdateTask, "Graf Update", 2048, (void *)client, 1, NULL);

    if(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("ESP_ADC_CAL_VAL_EFUSE_VREF:  support\n");
    }
    if(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
        printf("ESP_ADC_CAL_VAL_EFUSE_TP:  support\n");
    else 
        printf("ESP_ADC_CAL_VAL_EFUSE_TP: NOT support\n");
    if(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_DEFAULT_VREF) == ESP_OK) 
        printf("ESP_ADC_CAL_VAL_DEFAULT_VREF:  support\n");
    else 
        printf("ESP_ADC_CAL_VAL_DEFAULT_VREF: NOT  support\n");
    if(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP_FIT) == ESP_OK) 
        printf("ESP_ADC_CAL_VAL_EFUSE_TP_FIT:  support\n");
    else 
        printf("ESP_ADC_CAL_VAL_EFUSE_TP_FIT:  support\n");

    // adc_channel_t channel[3] = {ADC_U, ADC_I, DUTY};
    // dmaAdcInit(channel, 3);

    

    adcInit();
    pwmInit();
    gpioInit();
    // printf("Vout=0.728 Vin = %f\n", dataCalculation(0.728, U_coef, 3));
    // printf("Vout=0.864 Vin = %f\n", dataCalculation(0.864, U_coef, 3));
    // printf("Vout=1.035 Vin = %f\n", dataCalculation(0.7, U_coef, 3));
    adc_vref_to_gpio(ADC_UNIT_2, GPIO_NUM_12);
    xTaskCreate(mqttPublishTask, "mqttPublishTask", 2048, (void *)client, 1, NULL);
    xTaskCreate(buttonReadTask, "buttonReadTask",2048, NULL, 1, NULL);
    // xTaskCreate(dcControllerTask, "dcControllerTask",4096, NULL, 1, NULL);
    xTaskCreate(adcReadTask, "adcReadTask",4096, NULL, 1, NULL);

    static controller_t cntrl = {CURRENT_PI, NULL, NULL, pwmStart, pwmStop};
    cntrl.actual_data_queue = xCurrentQueue;
    cntrl.desired_data_queue = xSetDataQueue;
    xTaskCreate(buckBoostController, "CurrentCntrl", 4096, (void *) &cntrl, 1, NULL);
    float tmp = 15;
    xQueueSend(xSetDataQueue, (void *) &tmp, 0);

    gpio_set_level(PWM_EN, 1);
    gpio_set_level(RST, 1);
    vTaskDelay(100*portTICK_RATE_MS);
    gpio_set_level(RST, 0);
    
    c_data.code = INFO;
    c_data.data = 0;
    xQueueSend(xPublishQueue, (void *) &c_data, 0);
    




    // printf("Vref = %d \n", chars.vref);
    // printf("Type %d \n", val);

    
    // while(1) {
    //     uint16_t adc_data = ringFilterHandler(&rb, (int16_t)adc1_get_raw(ADC1_CHANNEL_4));
    //     printf(">ADC:%d\n", adc_data * 2256 / 4095);
    //     printf(">ADC_CAL:%d\n", (uint16_t)esp_adc_cal_raw_to_voltage(adc_data,  &chars));
    //     vTaskDelay(100 * portTICK_RATE_MS);
    // }
}