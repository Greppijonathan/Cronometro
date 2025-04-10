/*
Implementar un cronómetro utilizando más de una tarea de FreeRTOS que muestre en pantalla el valor de la cuenta actual
con una resolución de décimas de segundo.El cronómetro debe iniciar y detener la cuenta al presionar un pulsador conectado
a la placa. Si está detenido, al presionar un segundo pulsador debe volver a cero.Mientras la cuenta está activa un led RGB
debe parpadear en verde y cuando está detenida debe permanecer en rojo.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ili9341.h"
#include "digitos.h"
#include "stdio.h"
#include "driver/gpio.h"

// Definiciones generales
#define DIGITO_ANCHO 60
#define DIGITO_ALTO 100
#define DIGITO_ENCENDIDO ILI9341_BLUE2
#define DIGITO_APAGADO 0x3800
#define DIGITO_FONDO ILI9341_BLACK

#define RGB_ROJO GPIO_NUM_1
#define RGB_VERDE GPIO_NUM_8
#define RGB_AZUL GPIO_NUM_10
#define LED_OFF 0

#define TEC1_Pausa GPIO_NUM_4
#define TEC2_Reiniciar GPIO_NUM_6

// Estados del cronómetro
typedef enum
{
    ESTADO_CORRIENDO,
    ESTADO_PAUSADO,
    ESTADO_REINICIAR
} EstadoCronometro;

volatile EstadoCronometro estadoActual = ESTADO_PAUSADO; // Se inicializa en pausa por defecto

// Estructura de los dígitos
typedef struct
{
    int decenasSegundos;
    int unidadesSegundos;
    int decimasSegundo;
} DigitosCronometro;

DigitosCronometro digitosActuales = {0, 0, 0};
SemaphoreHandle_t semaforoAccesoDigitos;

/*****************************************FUNCIONES AUXILIARES**********************************************/
/**
 * @brief  Accede a la estructura en donde se almacenan los digitos del semaforo y se actualiza el conteo del tiempo
 *         La estructura que contiene los dígitos del cronómetro es un recurso compartido, por eso se usa el MUTEX
 */
void ActualizarDigitos()
{
    if (xSemaphoreTake(semaforoAccesoDigitos, portMAX_DELAY)) // Se toma recurso, se actualiza el MUTEX
    {
        digitosActuales.decimasSegundo = (digitosActuales.decimasSegundo + 1) % 10;

        if (digitosActuales.decimasSegundo == 0)
        {
            digitosActuales.unidadesSegundos = (digitosActuales.unidadesSegundos + 1) % 10;

            if (digitosActuales.unidadesSegundos == 0)
            {
                digitosActuales.decenasSegundos = (digitosActuales.decenasSegundos + 1) % 6;
            }
        }
        xSemaphoreGive(semaforoAccesoDigitos); // Se libera el MUTEX
    }
}

/**
 * @brief Resetea los dígitos del cronómetro a 00:00
 *
 */
void ReiniciarCronometro()
{
    if (xSemaphoreTake(semaforoAccesoDigitos, portMAX_DELAY))
    {
        digitosActuales.unidadesSegundos = 0;
        digitosActuales.decenasSegundos = 0;
        digitosActuales.decimasSegundo = 0;
        xSemaphoreGive(semaforoAccesoDigitos);
    }
}

/**
 * @brief Contiene la logica para prender el color correspondiente del led RGB
 *
 * @param estadoLed recibe el estado del led RGB
 */
void PrenderLedVerde(bool estadoLed)
{
    gpio_set_level(RGB_VERDE, estadoLed);
    gpio_set_level(RGB_ROJO, LED_OFF);
    gpio_set_level(RGB_AZUL, LED_OFF);
}

/**
 * @brief Prende el led rojo del RGB
 *
 * Cambia el estado del led rojo del RGB. Si estadoLed es verdadero, prende el led rojo,
 * si es falso, lo apaga.
 *
 * @param estadoLed estado del led rojo
 */
void PrenderLedRojo(bool estadoLed)
{
    gpio_set_level(RGB_VERDE, LED_OFF);
    gpio_set_level(RGB_ROJO, estadoLed);
    gpio_set_level(RGB_AZUL, LED_OFF);
}

/**
 * @brief Configura los pines del led RGB como salidas
 *
 * Inicializa los pines del led RGB como salidas para poder
 * controlar el color del led.
 */
void ConfigurarSalidasLed(void)
{
    gpio_set_direction(RGB_ROJO, GPIO_MODE_OUTPUT);
    gpio_set_direction(RGB_VERDE, GPIO_MODE_OUTPUT);
    gpio_set_direction(RGB_AZUL, GPIO_MODE_OUTPUT);
}

/**
 * @brief Configura los pines de las teclas como entradas con pull-up
 *
 * Inicializa los pines de las teclas como entradas con pull-up.
 * La tecla de pausa se configura en el pin GPIO_NUM_4 y la tecla de reiniciar
 * se configura en el pin GPIO_NUM_6.
 */
void ConfigurarTeclas(void)
{
    gpio_set_direction(TEC1_Pausa, GPIO_MODE_INPUT);
    gpio_set_pull_mode(TEC1_Pausa, GPIO_PULLUP_ONLY);
    gpio_set_direction(TEC2_Reiniciar, GPIO_MODE_INPUT);
    gpio_set_pull_mode(TEC2_Reiniciar, GPIO_PULLUP_ONLY);
}
/*****************************************FUNCIONES DE LAS TAREAS*****************************************/

/**
 * @brief Maneja el estado actual del cronómetro
 *
 * Tarea que se encarga de manejar el estado actual del cronómetro,
 * actualizando los dígitos y cambiando el estado de los LEDs según el caso.
 * La tarea utiliza un semáforo para acceder a los valores actuales del cronómetro y
 * se ejecuta a una tasa de 1 Hz.
 * @param parametros No se utiliza
 */
void ManejarEstadoCronometro(void *parametros)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    bool estadoLed = true;
    while (1)
    {
        switch (estadoActual)
        {
        case ESTADO_CORRIENDO:
            ActualizarDigitos();
            PrenderLedVerde(estadoLed);
            break;

        case ESTADO_PAUSADO:
            PrenderLedRojo(estadoLed);
            break;

        case ESTADO_REINICIAR:
            PrenderLedRojo(estadoLed);
            ReiniciarCronometro();
            estadoActual = ESTADO_PAUSADO; // Luego de un reinicio se deja por defecto el creonometro pausado
            break;
        }
        estadoLed = !estadoLed;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Función que se encarga de escanear los pulsadores y de
 *        realizar las acciones correspondientes según el estado
 *        actual del cronómetro.
 *
 * @param parametros No se utiliza
 */
void EscanearPulsadores(void *parametros)
{
    bool estadoAnteriorTEC1 = true;
    bool estadoAnteriorTEC2 = true;

    while (1)
    {
        bool estadoActualTEC1 = gpio_get_level(TEC1_Pausa);
        if (!estadoActualTEC1 && estadoAnteriorTEC1)
        {
            estadoActual = (estadoActual == ESTADO_CORRIENDO) ? ESTADO_PAUSADO : ESTADO_CORRIENDO;
        }
        estadoAnteriorTEC1 = estadoActualTEC1;

        bool estadoActualTEC2 = gpio_get_level(TEC2_Reiniciar);
        if (!estadoActualTEC2 && estadoAnteriorTEC2)
        {
            if (estadoActual == ESTADO_PAUSADO)
            {
                estadoActual = ESTADO_REINICIAR;
            }
        }
        estadoAnteriorTEC2 = estadoActualTEC2;

        vTaskDelay(pdMS_TO_TICKS(50)); // Manejo del rebote
    }
}

/**
 * @brief Actualiza la pantalla con los valores actuales del cronómetro
 *
 * Tarea que se encarga de actualizar la pantalla TFT con los valores actuales del
 * cronómetro, comparando con los valores previamente mostrados y actualizando solo
 * los dígitos que cambiaron. Adicionalmente, dibuja los separadores estáticos.
 * La tarea utiliza un semáforo para acceder a los valores actuales del cronómetro y
 * actualiza la pantalla a una tasa de 4 Hz.
 *@param parametros No se utiliza
 * */
void ActualizarPantalla(void *parametros)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    ILI9341Init();
    ILI9341Rotate(ILI9341_Landscape_1);

    panel_t Segundos = CrearPanel(60, (240 - DIGITO_ALTO) / 2, 2, DIGITO_ALTO, DIGITO_ANCHO, DIGITO_ENCENDIDO, DIGITO_APAGADO, DIGITO_FONDO);
    panel_t Decimas = CrearPanel(220, (240 - DIGITO_ALTO) / 2, 2, DIGITO_ALTO, DIGITO_ANCHO, DIGITO_ENCENDIDO, DIGITO_APAGADO, DIGITO_FONDO);

    struct digitos_previos
    {
        int decenasSegundosAnterior;
        int unidadesSegundosAnterior;
        int decimasSegundosAnterior;
    } digitosPrevios = {-1, -1, -1}; // Inicializamos con valores "inválidos"

    while (1)
    {
        if (xSemaphoreTake(semaforoAccesoDigitos, portMAX_DELAY)) // Se accede al recurso compartido, se toma el semaforo
        {
            // Logica para detectar cambio en los digitos y mostrarlos
            if (digitosActuales.decimasSegundo != digitosPrevios.decimasSegundosAnterior)
            {
                DibujarDigito(Decimas, 0, digitosActuales.decimasSegundo);
                digitosPrevios.decimasSegundosAnterior = digitosActuales.decimasSegundo;
            }
            if (digitosActuales.unidadesSegundos != digitosPrevios.unidadesSegundosAnterior)
            {
                DibujarDigito(Segundos, 1, digitosActuales.unidadesSegundos);
                digitosPrevios.unidadesSegundosAnterior = digitosActuales.unidadesSegundos;
            }
            if (digitosActuales.decenasSegundos != digitosPrevios.decenasSegundosAnterior)
            {
                DibujarDigito(Segundos, 0, digitosActuales.decenasSegundos);
                digitosPrevios.decenasSegundosAnterior = digitosActuales.decenasSegundos;
            }
            //   ILI9341DrawFilledCircle(120, 140, 2, DIGITO_ENCENDIDO);
            //   ILI9341DrawFilledCircle(120, 100, 2, DIGITO_ENCENDIDO);
            ILI9341DrawFilledCircle(200, 160, 4, DIGITO_ENCENDIDO);

            xSemaphoreGive(semaforoAccesoDigitos); // Se libera el recurso compartid, se libera el semaforo
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50)); // Tiempo de actualización de pantalla
    }
}
/***************************************** app_main()*****************************************/
void app_main(void)
{
    // Se crea el semaforo que se va a utilizar en las diferentes zonas del codigo
    semaforoAccesoDigitos = xSemaphoreCreateMutex();
    // configuraciones iniciales
    ConfigurarSalidasLed();
    ConfigurarTeclas();
    // Se crean las tareas
    xTaskCreate(ManejarEstadoCronometro, "ManejarEstadoCronometro", 2048, NULL, 2, NULL);
    xTaskCreate(EscanearPulsadores, "EscanearPulsadores", 1024, NULL, 1, NULL);
    xTaskCreate(ActualizarPantalla, "ActualizarPantalla", 4096, NULL, 1, NULL);
}