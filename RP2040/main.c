/**
 * Analizador de vibraciones distribuido
 * RP2040 (Pico SDK + FreeRTOS)
 *
 * - Núcleo 0: tareas FreeRTOS
 *     * mpuTask: adquiere muestras del MPU6050 a 1 kHz
 *     * commTask: envía características por UART al ESP32
 *     * motorTask: controla la velocidad del motor a pasos 
 *     * uartRxTask: recibe datos para cambiar la velocidad
 * - Núcleo 1: función core1_main para el calculo de la FFT
 */

// ===================== FreeRTOS =====================
#include "FreeRTOS.h"
#include "task.h"

// ===================== Librerías estándar C =====================
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>     // atoi
#include <stdio.h>      // snprintf
#include <math.h>       // cosf, sinf, sqrtf

// ===================== Librerías RP2040 =====================
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

// ===================== Configuración Hardware =====================

// I2C para MPU6050 en GPIO2 (SDA) y GPIO3 (SCL) usando I2C1
#define I2C_PORT        i2c1
#define I2C_SDA_PIN     2
#define I2C_SCL_PIN     3
#define MPU6050_ADDR    0x68

// UART hacia ESP32 (DualMCU: RP2040 GPIO0/1 ↔ ESP32 GPIO17/16)
#define UART_PORT       uart0
#define UART_TX_PIN     0     // conectado al ESP32 RX
#define UART_RX_PIN     1     // conectado al ESP32 TX
#define UART_BAUDRATE   115200

// Frecuencia de muestreo y tamaño de bloque
#define FS_HZ           1000        // [Hz] de muestreo
#define N_MUESTRAS      2048        // Tamaño de bloque para FFT (potencia de 2)

// Constante PI para funciones trigonométricas
#define PI_F            3.14159265358979323846f

// CONVERSIÓN MPU6050 A g (±4 g)
#define MPU6050_LSB_PER_G  8192.0f  

// Motor a pasos 28BYJ-48 
#define MOTOR_IN1_PIN   28
#define MOTOR_IN2_PIN   26
#define MOTOR_IN3_PIN   23
#define MOTOR_IN4_PIN   21

// Periodos de paso en ms derivados de la "velocidad" (0-100%)
#define MOTOR_STEP_PERIOD_MS_MIN   20   // Motor al máximo (~100%)
#define MOTOR_STEP_PERIOD_MS_MAX   200  // Motor al mínimo (~1%)

// ===================== Tipos de datos =====================

// Esta estructura guarda lo que se mandará al ESP32: Frecuencia pico,
// amplitud pico y energía RMS en tres bandas
typedef struct {
    float frec_pico;
    float amp_pico;
    float rms_bajo;
    float rms_medio;
    float rms_alto;
    float t_fft;
} vib_mediciones_t;

// Job para núcleo 1: guarda un puntero a cuál buffer debe procesar el core 1
typedef struct {
    const int16_t *buffer;
} core1_job_t;

// ===================== Variables globales =====================

// Doble buffer de muestras
static int16_t g_buffers[2][N_MUESTRAS];

// Job para núcleo 1
static core1_job_t g_core1_job;

// Últimas características calculadas por núcleo 1
static vib_mediciones_t g_mediciones;

// Semáforos Pico SDK para coordinar núcleos
static semaphore_t sem_core1_inicio;      // Núcleo 0 -> Núcleo 1 (nuevo bloque listo)
static semaphore_t sem_core1_hecho;       // Núcleo 1 -> Núcleo 0 (terminó de procesar bloque)
static semaphore_t sem_mediciones_listo;  // Núcleo 1 -> commTask (mediciones listas)

// Velocidad del motor (0–100 %)
static volatile uint8_t g_velocidad_motor = 0;

// ===================== Prototipos =====================

// Núcleo 1
void core1_main(void);

// Tareas FreeRTOS (núcleo 0)
static void mpuTask(void *pvParameters);
static void commTask(void *pvParameters);
static void motorTask(void *pvParameters);
static void uartRxTask(void *pvParameters);

// Funciones auxiliares para semáforos
static inline void sem_adquisicion_bloqueo(semaphore_t *s);
static inline void sem_aviso(semaphore_t *s);

// MPU6050
static void    mpu6050_escribir_reg(uint8_t reg, uint8_t val);
static uint8_t mpu6050_leer_reg(uint8_t reg);
static void    mpu6050_leer_bytes(uint8_t reg, uint8_t *buf, size_t len);
static void    mpu6050_iniciar(void);
static int16_t mpu6050_leer_acel_z(void);

// FFT + procesamiento
static void fft_radix2(float *xr, float *xi, int n);
static void algoritmo_fft(const int16_t *muestras, vib_mediciones_t *out);

// Motor a pasos
static void motor_configurar_pines(void);
static void motor_apagar(void);
static void motor_aplicar_paso(uint8_t paso);

// Procesamiento de comandos por UART
static void procesar_comando_uart(const char *linea);

// ===================== Implementación semáforos auxiliares =====================

static inline void sem_adquisicion_bloqueo(semaphore_t *s)
{
    sem_acquire_blocking(s);
}

static inline void sem_aviso(semaphore_t *s)
{
    sem_release(s);
}

// ===================== main (núcleo 0) =====================

int main(void)
{
    stdio_init_all();

    // --- I2C para MPU6050 ---
    i2c_init(I2C_PORT, 400000);  // 400 kHz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    mpu6050_iniciar();

    // --- UART hacia ESP32 ---
    uart_init(UART_PORT, UART_BAUDRATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // --- Pines del motor a pasos ---
    motor_configurar_pines();
    motor_apagar();

    // --- Semáforos entre nucleos para mediciones ---
    sem_init(&sem_core1_inicio, 0, 1);
    sem_init(&sem_core1_hecho,  1, 1);
    sem_init(&sem_mediciones_listo, 0, 1);

    // --- Lanzar núcleo 1 ---
    multicore_launch_core1(core1_main);

    // --- Crear tareas FreeRTOS en núcleo 0 ---
    xTaskCreate(mpuTask,   "mpuTask",   configMINIMAL_STACK_SIZE + 256, NULL, 2, NULL);
    xTaskCreate(commTask,  "commTask",  configMINIMAL_STACK_SIZE + 256, NULL, 1, NULL);
    xTaskCreate(motorTask, "motorTask", configMINIMAL_STACK_SIZE + 256, NULL, 2, NULL);
    xTaskCreate(uartRxTask,"uartRxTask",configMINIMAL_STACK_SIZE + 256, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true) {
        for (;;);
    }
}

// ===================== Tarea de adquisición (núcleo 0) =====================

static void mpuTask(void *pvParameters)
{
    (void) pvParameters;

    const TickType_t period_ticks = pdMS_TO_TICKS(1000 / FS_HZ);
    TickType_t last_wake = xTaskGetTickCount();

    int buffer_seleccionado = 0;  // 0 ó 1
    size_t idx = 0;

    for (;;)
    {
        vTaskDelayUntil(&last_wake, period_ticks);

        g_buffers[buffer_seleccionado][idx] = mpu6050_leer_acel_z();
        idx++;

        if (idx >= N_MUESTRAS) {
            idx = 0;

            sem_adquisicion_bloqueo(&sem_core1_hecho);

            g_core1_job.buffer = g_buffers[buffer_seleccionado];

            buffer_seleccionado = 1 - buffer_seleccionado;

            sem_aviso(&sem_core1_inicio);
        }
    }
}

// ===================== Tarea de comunicación (núcleo 0) =====================

static void commTask(void *pvParameters)
{
    (void) pvParameters;

    vib_mediciones_t local;
    char trama[128];

    for (;;)
    {
        sem_adquisicion_bloqueo(&sem_mediciones_listo);

        local = g_mediciones;

        int n = snprintf(
            trama,
            sizeof(trama),
            "VIB,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            local.frec_pico,
            local.amp_pico,   
            local.rms_bajo,   
            local.rms_medio,  
            local.rms_alto,   
            local.t_fft
        );

        for (int i = 0; i < n; i++) {
            uart_putc_raw(UART_PORT, trama[i]);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ===================== Tarea de control del motor (núcleo 0) =====================

static void motorTask(void *pvParameters)
{
    (void) pvParameters;

    TickType_t last_wake = xTaskGetTickCount();
    uint8_t paso_actual = 0;

    for (;;)
    {
        uint8_t vel = g_velocidad_motor;
        uint32_t periodo_ms;

        if (vel == 0) {
            motor_apagar();
            vTaskDelay(pdMS_TO_TICKS(50));
            last_wake = xTaskGetTickCount();
            continue;
        } else {
            periodo_ms = MOTOR_STEP_PERIOD_MS_MAX -
                         (uint32_t)vel *
                         (MOTOR_STEP_PERIOD_MS_MAX - MOTOR_STEP_PERIOD_MS_MIN) / 100U;

            if (periodo_ms < MOTOR_STEP_PERIOD_MS_MIN) {
                periodo_ms = MOTOR_STEP_PERIOD_MS_MIN;
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(periodo_ms));

        motor_aplicar_paso(paso_actual);
        paso_actual = (paso_actual + 1) & 0x03;  // 0..3
    }
}

// ===================== Tarea de recepción UART (núcleo 0) =====================

static void uartRxTask(void *pvParameters)
{
    (void) pvParameters;

    char buffer[32];
    int idx = 0;

    for (;;)
    {
        if (uart_is_readable(UART_PORT)) {
            char c = uart_getc(UART_PORT);

            if (c == '\r') {
                continue;
            } else if (c == '\n') {
                buffer[idx] = '\0';
                if (idx > 0) {
                    procesar_comando_uart(buffer);
                }
                idx = 0;
            } else {
                if (idx < (int)(sizeof(buffer) - 1)) {
                    buffer[idx++] = c;
                } else {
                    idx = 0;
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

// ===================== Núcleo 1 – FFT + mediciones =====================

void core1_main(void)
{
    while (true)
    {
        sem_adquisicion_bloqueo(&sem_core1_inicio);

        const int16_t *muestras = g_core1_job.buffer;

        uint64_t t_ini_us = time_us_64();
        algoritmo_fft(muestras, &g_mediciones);
        uint64_t t_fin_us = time_us_64();

        uint64_t dt_us = t_fin_us - t_ini_us;
        g_mediciones.t_fft = (float)dt_us / 1000.0f;

        sem_aviso(&sem_mediciones_listo);
        sem_aviso(&sem_core1_hecho);
    }
}

// ===================== MPU6050 – funciones básicas =====================

static void mpu6050_escribir_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
}

static uint8_t mpu6050_leer_reg(uint8_t reg)
{
    uint8_t val;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, &val, 1, false);
    return val;
}

static void mpu6050_leer_bytes(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buf, len, false);
}

static void mpu6050_iniciar(void)
{
    mpu6050_escribir_reg(0x6B, 0x00); // PWR_MGMT_1
    mpu6050_escribir_reg(0x1C, 0x08); // ACCEL_CONFIG ±4g
}

static int16_t mpu6050_leer_acel_z(void)
{
    uint8_t buf[2];
    mpu6050_leer_bytes(0x3F, buf, 2); 
    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
    return raw; // cuentas LSB
}

// ===================== FFT radix-2 Cooley–Tukey =====================

static void fft_radix2(float *xr, float *xi, int n)
{
    int j = 0;
    for (int i = 0; i < n; i++) {
        if (i < j) {
            float tmp_r = xr[i];
            float tmp_i = xi[i];
            xr[i] = xr[j];
            xi[i] = xi[j];
            xr[j] = tmp_r;
            xi[j] = tmp_i;
        }
        int m = n >> 1;
        while (m >= 1 && j >= m) {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    for (int step = 1; step < n; step <<= 1) {
        int jump = step << 1;
        float delta_angle = -PI_F / (float)step;
        float w_m_r = cosf(delta_angle);
        float w_m_i = sinf(delta_angle);

        for (int k = 0; k < n; k += jump) {
            float w_r = 1.0f;
            float w_i = 0.0f;
            for (int m = 0; m < step; m++) {
                int i0 = k + m;
                int i1 = i0 + step;

                float t_r = w_r * xr[i1] - w_i * xi[i1];
                float t_i = w_r * xi[i1] + w_i * xr[i1];

                float u_r = xr[i0];
                float u_i = xi[i0];

                xr[i0] = u_r + t_r;
                xi[i0] = u_i + t_i;
                xr[i1] = u_r - t_r;
                xi[i1] = u_i - t_i;

                float tmp_r = w_r * w_m_r - w_i * w_m_i;
                float tmp_i = w_r * w_m_i + w_i * w_m_r;
                w_r = tmp_r;
                w_i = tmp_i;
            }
        }
    }
}

// ===================== Procesamiento de bloque (FFT + RMS) =====================

static void algoritmo_fft(const int16_t *muestras, vib_mediciones_t *out)
{
    static float xr[N_MUESTRAS];
    static float xi[N_MUESTRAS];

    // Calcular el promedio de la mediciones en el eje
    float promedio_eje_z = 0.0f;
    for (int i = 0; i < N_MUESTRAS; i++) {
        promedio_eje_z += (float)muestras[i];
    }
    promedio_eje_z /= (float)N_MUESTRAS;

    // Ventana de Hamming
    for (int i = 0; i < N_MUESTRAS; i++) {
        float w   = 0.54f - 0.46f * cosf(2.0f * PI_F * (float)i / (float)(N_MUESTRAS - 1));
        float x_g = ((float)muestras[i] - promedio_eje_z) / MPU6050_LSB_PER_G;  // sin DC
        xr[i] = x_g * w;
        xi[i] = 0.0f;
    }

    // algorito de la FFT
    fft_radix2(xr, xi, N_MUESTRAS);

    int   n2    = N_MUESTRAS / 2;
    float f_res = (float)FS_HZ / (float)N_MUESTRAS;

    float mag_max   = 0.0f;
    int   idx_max   = 0;
    float rms_bajo  = 0.0f;
    float rms_medio = 0.0f;
    float rms_alto  = 0.0f;

    float f_baja_max = 50.0f;
    float f_media_max = 200.0f;

    for (int k = 1; k < n2; k++) {
        float re  = xr[k];
        float im  = xi[k];
        float mag = sqrtf(re * re + im * im);  

        if (mag > mag_max) {
            mag_max = mag;
            idx_max = k;
        }

        float f    = f_res * (float)k;
        float mag2 = mag * mag;

        if (f <= f_baja_max)      rms_bajo  += mag2;
        else if (f <= f_media_max) rms_medio += mag2;
        else                     rms_alto  += mag2;
    }

    rms_bajo  = sqrtf(rms_bajo);   
    rms_medio = sqrtf(rms_medio);  
    rms_alto  = sqrtf(rms_alto);   

    out->frec_pico  = idx_max * f_res; 
    out->amp_pico   = mag_max;         
    out->rms_bajo   = rms_bajo;        
    out->rms_medio  = rms_medio;       
    out->rms_alto   = rms_alto;        
}

// ===================== Motor a pasos =====================

static const uint8_t tabla_pasos[4][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1},
};

static void motor_configurar_pines(void)
{
    gpio_init(MOTOR_IN1_PIN);
    gpio_init(MOTOR_IN2_PIN);
    gpio_init(MOTOR_IN3_PIN);
    gpio_init(MOTOR_IN4_PIN);

    gpio_set_dir(MOTOR_IN1_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR_IN2_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR_IN3_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR_IN4_PIN, GPIO_OUT);
}

static void motor_apagar(void)
{
    gpio_put(MOTOR_IN1_PIN, 0);
    gpio_put(MOTOR_IN2_PIN, 0);
    gpio_put(MOTOR_IN3_PIN, 0);
    gpio_put(MOTOR_IN4_PIN, 0);
}

static void motor_aplicar_paso(uint8_t paso)
{
    paso &= 0x03;
    gpio_put(MOTOR_IN1_PIN, tabla_pasos[paso][0]);
    gpio_put(MOTOR_IN2_PIN, tabla_pasos[paso][1]);
    gpio_put(MOTOR_IN3_PIN, tabla_pasos[paso][2]);
    gpio_put(MOTOR_IN4_PIN, tabla_pasos[paso][3]);
}

// ===================== Procesamiento de comandos UART =====================

static void procesar_comando_uart(const char *linea)
{
    if (strncmp(linea, "VEL,", 4) == 0) {
        const char *p = linea + 4;
        int valor = atoi(p);
        if (valor < 0)   valor = 0;
        if (valor > 100) valor = 100;
        g_velocidad_motor = (uint8_t)valor;
    }
}

