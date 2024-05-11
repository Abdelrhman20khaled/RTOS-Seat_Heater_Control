/********************************************************************
 *                                                                  *
 * Project : Control the Heat of the driver and passenger Seat      *
 *                                                                  *
 * File Name : RTOS Application for Modifying the Seat Heat Degree  *
 *                                                                  *
 * Created By: Abdelrhman Khaled Sobhi                              *
 *                                                                  *
 *******************************************************************/

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "queue.h"

/*
 * MCAL includes:
 * 1- GPIO => for initializing the pins for button and sensors and display.
 * 2- UART => for displaying on terminal.
 * 3- GTPM => timer needed for switching and time measurements.
 *
 */

#include "gpio.h"
#include "uart0.h"
#include "GPTM.h"
#include "tm4c123gh6pm_registers.h"

/**********************************************************
 *                   Important Defines                    *
 **********************************************************/

#define ADC_CODE_FOR_2_DEG              (184)
#define ADC_CODE_FOR_3_DEG              (257)
#define ADC_CODE_FOR_5_DEG              (459)
#define ADC_CODE_FOR_10_DEG             (917)
#define ADC_CODE_FOR_25_DEG             (2292)
#define ADC_CODE_FOR_30_DEG             (2750)
#define ADC_CODE_FOR_35_DEG             (3209)
#define ADC_CODE_FOR_40_DEG             (3667)
#define TEMP_MAX_VALUE                  (3667)
#define TEMP_MIN_VALUE                  (459)
#define DRIVER_QUEUE_ERROR_FLAG         ( 1UL << 0UL )
#define PASSENGER_QUEUE_ERROR_FLAG      ( 1UL << 1UL )
#define TOTAL_NUMBER_OF_TASKS_WITH_IDEL (12U)

/*
 * Delay Function
 */
#define NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND 369

void Delay_MS(unsigned long long n)
{
    volatile unsigned long long count = 0;
    while(count++ < (NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND * n) );
}

/**********************************************************
 *           ENUM include the status of Heater            *
 **********************************************************/
typedef enum{
    Heater_Off,Heater_Low,Heater_Medium,Heater_High
}Heater_Level;

/**********************************************************
 *           ENUM include the speed of Heater             *
 **********************************************************/
typedef enum{
    Intensity_Disable,Intensity_Low,Intensity_Medium,Intensity_High
}Heater_Speed;

/**********************************************************
 *         Declares the Tasks Needed in this APP          *
 **********************************************************/

static void prvSetupHardware( void );
void vPeriodic_driver_TeampRead(void *pvParameters);
void vPeriodic_passnegerSensor_TeampRead(void *pvParameters);
void vDriver_Seat_Button(void *pvParameters);
void vPassenger_Seat_Button(void *pvParameters);
void vSteering_Button(void *pvParameters);
void vDriverSensor_displayData(void *pvParameters);
void vPassengerSensor_displayData(void *pvParameters);
void vDriverSeat_Temperature(void *pvParameters);
void vPassengerSeat_Temperature(void *pvParameters);
void vSeat_sensorEfficiency(void *pvParameters);
void vRunTimeMeasurementsTask(void *pvParameters);

/**********************************************************
 *                  Some Handler Needed                   *
 **********************************************************/
/*the MUTEX for controlling flow of data */
xSemaphoreHandle xUART0_Avilabel;

/*the event group used for control action*/
EventGroupHandle_t xEvent_Group;

/* The handler of the Queues used to store the reading of ADC0 */
xQueueHandle xDriver_Sensor_Reads_Queue;

/* The handler of the Queues used to store the reading of ADC1 */
xQueueHandle xPassenger_Sensor_Reads_Queue;

/* This Queue used for storing the error reading by the driver seat sensors */
xQueueHandle xError_passengerSensor_Reads_Queue;

/* This Queue used for storing the error reading by the driver seat sensors */
xQueueHandle xError_driverSensor_Reads_Queue;

/**********************************************************
 *                  All Task Handler Needed               *
 **********************************************************/

/* 1- The Handler pass to the task reads the value of driver sensor */
xTaskHandle xPeriodic_driverSensor_Read;
/* 2- The Handler pass to the task reads the value of passenger sensor */
xTaskHandle xPeriodic_passengerSensor_Read;
/* 3- The Handler pass to the task checks if the button of driver was pressed or not */
xTaskHandle xPeriodic_seat1Button_Pressed;
/* 4- The Handler pass to the task checks if the button of passenger was pressed or not */
xTaskHandle xPeriodic_seat2Button_Pressed;
/* 5- The Handler pass to the task checks if the button of passenger was pressed or not */
xTaskHandle xPeriodic_steeringButton_Pressed;
/* 6- The Handler pass to the task used for displaying the data about driver sensor */
xTaskHandle xPeriodic_driverSensor_displayData;
/* 7- The Handler pass to the task used for displaying the data about passenger sensor */
xTaskHandle xPeriodic_passengerSensor_displayData;
/* 8- This handler passed to the function used for controlling the function*/
xTaskHandle xDriver_SeatTemp;
/* 9- This handler passed to the function used for controlling the function*/
xTaskHandle xPassenger_SeatTemp;
/* 10- This handler passed to the function used for checking efficiency of the seats sensors */
xTaskHandle xSensor_efficiency;
/* 11- This handler of Run Time Measurements */
xTaskHandle xRunTimeMeasurements_Handler;


/**********************************************************
 *             Global Variable Needed                     *
 **********************************************************/

/*The variable for storing the value return from the ADC0 (driver sensor)*/
static uint32 ADC0_read;
/*The variable for storing the value return from the ADC1 (passenger sensor)*/
static uint32 ADC1_read;
/*The variable for storing the value current temperature of driver seat */
uint32 Driversensor_currentRead = ADC_CODE_FOR_25_DEG;
/*The variable for storing the value current temperature of passenger seat */
uint32 Passengersensor_currentRead = ADC_CODE_FOR_25_DEG;
/*The required level of heating set by the user (driver)*/
uint8 Seat1_heater_Status = Heater_Off;
/*The required level of heating set by the user (passenger)*/
uint8 Seat2_heater_Status = Heater_Off;
/*The required level of heating set by the user (driver)*/
uint8 Seat1_heater_Speed = Intensity_Disable;
/*The required level of heating set by the user (passenger)*/
uint8 Seat2_heater_Speed= Intensity_Disable;

/*******************************************************************************
 *             Array used for Calculating Run Time Measurements                *
 ******************************************************************************/

uint32 ullTasksOutTime[TOTAL_NUMBER_OF_TASKS_WITH_IDEL];
uint32 ullTasksInTime[TOTAL_NUMBER_OF_TASKS_WITH_IDEL];
uint32 ullTasksTotalTime[TOTAL_NUMBER_OF_TASKS_WITH_IDEL];


int main(void)
{
    /* Setup the hardware for use with the TIVA C board */
    prvSetupHardware();

    /* Create the MUTEX needed for control the button tasks */
    xUART0_Avilabel = xSemaphoreCreateMutex();

    /* Create the Event Group */
    xEvent_Group = xEventGroupCreate();
    /*
     * Create queue that stores the driver seat sensor current and previous
     * reading from sensor of driver seat (two values only)
     */
    xDriver_Sensor_Reads_Queue = xQueueCreate(2,sizeof(uint32));
    /*
     * Create queue that stores the passenger seat sensor current and previous
     * reading from sensor of driver seat (two values only)
     */
    xPassenger_Sensor_Reads_Queue = xQueueCreate(2,sizeof(uint32));
    /* Create queue that stores the error reading from sensor of driver seat */
    xError_driverSensor_Reads_Queue = xQueueCreate(5,sizeof(uint32));
    /* Create queue that stores the error reading from sensor of passenger seat */
    xError_passengerSensor_Reads_Queue = xQueueCreate(5,sizeof(uint32));

    /* Create Tasks needed for the project here */

    xTaskCreate
    (
            vDriverSensor_displayData,
            "Display All Data of Driver Sensor - Task",
            256,
            NULL,
            5,
            &xPeriodic_driverSensor_displayData
    );

    xTaskCreate
    (
            vPassengerSensor_displayData,
            "Display All Data of Passenger Sensor - Task",
            256,
            NULL,
            5,
            &xPeriodic_passengerSensor_displayData
    );

    xTaskCreate
    (
            vPeriodic_driver_TeampRead,
            "Driver Temperature Sensor Reading - Task",
            256,
            NULL,
            3,
            &xPeriodic_driverSensor_Read
    );

    xTaskCreate
    (
            vPeriodic_passnegerSensor_TeampRead,
            "passenger Temperature Sensor Reading Task",
            256,
            NULL,
            3,
            &xPeriodic_passengerSensor_Read
    );

    xTaskCreate
    (
            vDriver_Seat_Button,
            "Seat1 Temperature Button - Task",
            256,
            NULL,
            3,
            &xPeriodic_seat1Button_Pressed
    );

    xTaskCreate
    (
            vPassenger_Seat_Button,
            "Seat2 Temperature Button - Task",
            256,
            NULL,
            2,
            &xPeriodic_seat2Button_Pressed
    );

    xTaskCreate
    (
            vSteering_Button,
            "Steering Temperature Button - Task",
            256,
            NULL,
            3,
            &xPeriodic_steeringButton_Pressed
    );

    xTaskCreate
    (
            vDriverSeat_Temperature,
            "Temperature of Driver Seat - Task",
            256,
            NULL,
            1,
            &xDriver_SeatTemp
    );

    xTaskCreate
    (
            vPassengerSeat_Temperature,
            "Temperature of Passenger Seat - Task",
            256,
            NULL,
            1,
            &xPassenger_SeatTemp
    );

    xTaskCreate
    (
            vSeat_sensorEfficiency,
            "Efficiency of Passenger Sensor - Task",
            256,
            NULL,
            3,
            &xSensor_efficiency
    );

    xTaskCreate
    (
            vRunTimeMeasurementsTask,
            " Run Time Measurements - Task",
            256,
            NULL,
            3,
            &xRunTimeMeasurements_Handler
    );

    /*
     * Tags used for calculating Run Time Measurements
     */

    /*1*/  vTaskSetApplicationTaskTag(xPeriodic_driverSensor_displayData,     (void *)  1);
    /*2*/  vTaskSetApplicationTaskTag(xPeriodic_passengerSensor_displayData,  (void *)  2);
    /*3*/  vTaskSetApplicationTaskTag(xPeriodic_driverSensor_Read,            (void *)  3);
    /*4*/  vTaskSetApplicationTaskTag(xPeriodic_passengerSensor_Read,         (void *)  4);
    /*5*/  vTaskSetApplicationTaskTag(xPeriodic_seat1Button_Pressed,          (void *)  5);
    /*6*/  vTaskSetApplicationTaskTag(xPeriodic_seat2Button_Pressed,          (void *)  6);
    /*7*/  vTaskSetApplicationTaskTag(xPeriodic_steeringButton_Pressed,       (void *)  7);
    /*8*/  vTaskSetApplicationTaskTag(xDriver_SeatTemp,                       (void *)  8);
    /*9*/  vTaskSetApplicationTaskTag(xPassenger_SeatTemp,                    (void *)  9);
    /*10*/ vTaskSetApplicationTaskTag(xSensor_efficiency,                     (void *) 10);
    /*11*/ vTaskSetApplicationTaskTag(xRunTimeMeasurements_Handler,           (void *) 11);

    /* Now all the tasks have been started - start the scheduler.*/
    vTaskStartScheduler();

    /* Should never reach here! */
    for (;;);

}


static void prvSetupHardware( void )
{
    /* Place here any needed HW initialization such as GPIO, UART, etc..  */
    GPIO_BuiltinButtonsLedsInit();
    GPTM_WTimer0Init();
    UART0_Init();
    Driver_TempSensor_Init();
    Passenegr_TempSensor_Init();
}

void vPeriodic_driver_TeampRead(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        /* Without Interrupt */
        ADC0_read = Driver_TempSensor_readChannel();

        /* Repeat this task each 500 m second */
        xTaskDelayUntil(&xLastWakeTime, 500);
    }
}

void vPeriodic_passnegerSensor_TeampRead(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        /* Without Interrupt */
        ADC1_read = Passenger_TempSensor_readChannel();

        /* Repeat this task each 500 m second */
        xTaskDelayUntil(&xLastWakeTime, 500);
    }
}

void vDriver_Seat_Button(void *pvParameters)
{
    uint8 Button_Status;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        Button_Status = GPIO_SW1GetState();

        if(Button_Status == PRESSED)
        {
            /*
             * if the button pressed only one time the current status of heater move step to high:
             * 1- if off    -> low
             * 2- if low    -> medium
             * 3- if medium -> high
             * 4- if high   -> off
             *
             */
            Seat1_heater_Status++;

            if(Seat1_heater_Status == Heater_High)
            {
                Seat1_heater_Status = Heater_Off;
            }
            else
            {
                /*No Action Required*/
            }

            /* This loop to ensure that the heater change it is level one step per one press */
            while(Button_Status != RELEASED);
        }
        else
        {
            /*No Action Required*/
        }

        /* Repeat this task each 250 m second */
        xTaskDelayUntil(&xLastWakeTime, 250);

    }

}

void vPassenger_Seat_Button(void *pvParameters)
{
    uint8 Button_Status;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        Button_Status = GPIO_SW2GetState();

        if(Button_Status == PRESSED)
        {
            /*
             * if the button pressed only one time the current status of heater move step to high:
             * 1- if off    -> low
             * 2- if low    -> medium
             * 3- if medium -> high
             * 4- if high   -> off
             *
             */
            Seat2_heater_Status++;

            if(Seat2_heater_Status == Heater_High)
            {
                Seat2_heater_Status = Heater_Off;
            }
            else
            {
                /*No Action Required*/
            }

            /* This loop to ensure that the heater change it is level one step per one press */
            while(Button_Status != RELEASED);
        }
        else
        {
            /*No Action Required*/
        }

        /* Repeat this task each 250 m second */
            xTaskDelayUntil(&xLastWakeTime, 250);

    }

}

void vSteering_Button(void *pvParameters)
{
    /*
     * NOTE: Steering button also controlling the heater of driver seat
     *       as this function " vSeat1_Button " but steering has a high priority
     *
     *       If the value set by button1 from this function " vSeat1_Button " and then
     *       the steering button change it, the value must override and changed to the
     *       value set by steering system.
     *
     */

    uint8 Button_Status = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        Button_Status = GPIO_ExtraSW_GetState();

        if(Button_Status == PRESSED)
        {
            /*
             * if the button pressed only one time the current status of heater move step to high:
             * 1- if off    -> low
             * 2- if low    -> medium
             * 3- if medium -> high
             * 4- if high   -> off
             *
             */
            Seat1_heater_Status++;

            if(Seat1_heater_Status == Heater_High)
            {
                Seat1_heater_Status = Heater_Off;
            }
            else
            {
                /*No Action Required*/
            }

            /* This loop to ensure that the heater change it is level one step per one press */
            while(Button_Status != RELEASED);
        }
        else
        {
            /*No Action Required*/
        }

        /* Repeat this task each 500 m second */
            xTaskDelayUntil(&xLastWakeTime, 250);
    }

}

void vDriverSeat_Temperature(void *pvParameters)
{
    /*
     * From the TIVAC manual we can find that:
     *
     * TEMP = 147.5 - ((75 * (VREFP - VREFN) × ADCCODE) / 4096)
     *
     * ADCCODE = ( -(TEMP - 147.5) * 4096 ) / (75 * 3.3)
     *
     * Heater status :
     * a. Low    --> 25°C.
     * b. Medium --> 30°C.
     * d. High   --> 35°C.
     *
     */
    uint32 current_tempRead = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        /**************************************************************
         * NOTE: the level of heater determines by another task.      *
         **************************************************************/
        current_tempRead = ADC0_read;

        /*****************************************************************************************
         * NOTE: the Sensor always give read to make it easy to detect if there is an issue in it*
         ****************************************************************************************/

        /*
         * Check if the sensor read less than 5 degrees or greater than 40 degrees
         * in this case send this value to the corresponding errors queue of this sensor.
         *
         */
        if( current_tempRead < TEMP_MIN_VALUE || current_tempRead > TEMP_MAX_VALUE)
        {
            /*
             * If the queue not full and have a space add the un-needed value to queue,
             * if full set the error flag and another task will handle this error and
             * then resume the task that handle this bug.
             *
             */
            if( uxQueueSpacesAvailable(xError_driverSensor_Reads_Queue) != 0 )
            {
                xQueueSend(xError_driverSensor_Reads_Queue,&current_tempRead,portMAX_DELAY);
            }
            else
            {
                /* For making sure that the error will be detected */
                xEventGroupSetBits(xEvent_Group, DRIVER_QUEUE_ERROR_FLAG);

                /* This task will be resumed if any error happen in any sensor */
                vTaskResume(xSensor_efficiency);
            }
        }
        else
        {
            /* No Action Required */
        }

        /* Check on the speed of the motor */
        if(Seat1_heater_Status == Heater_Off)
        {
            /*
             * No Action Required because the feature is off, and the temperature is not controlled
             * so the heater will be disabled
             */
            Seat1_heater_Speed = Intensity_Disable;
        }
        else if( Seat1_heater_Status != Heater_Off)
        {

            switch(Seat1_heater_Status)
            {
            case Heater_Low:

                /* From requirements the temperature match with heater  in low level is 25 degrees */
                Driversensor_currentRead = ADC_CODE_FOR_25_DEG;

                /*
                 * 1- If the current temperature is less than the desired temperature by 10°C or more the heater
                 *    should be enabled with the high intensity.
                 * 2- If the current temperature is less than the desired temperature by 5°C to 10°C the heater
                 *    should be enabled with a medium intensity
                 * 3- If the current temperature is less than the desired temperature by 2°C to 5°C the heater should
                 *    be enabled with a low intensity.
                 * 4- If the current temperature is more than the desired temperature the heater should be
                 *    disabled.
                 * 5- The heater shall be enabled once again if the temperature becomes less than the desired
                 *    temperature by 3°C.
                 *
                 */
                if( Driversensor_currentRead - current_tempRead == ADC_CODE_FOR_10_DEG )
                {
                    Seat1_heater_Speed = Intensity_High;
                    GPIO_GreenLedOn();
                    GPIO_BlueLedOn();
                }
                else if( (Driversensor_currentRead - current_tempRead >= ADC_CODE_FOR_5_DEG) && (Driversensor_currentRead - current_tempRead <= ADC_CODE_FOR_10_DEG))
                {
                    Seat1_heater_Speed = Intensity_Medium;
                    GPIO_GreenLedOff();
                    GPIO_BlueLedOn();
                }
                else if( (Driversensor_currentRead - current_tempRead >= ADC_CODE_FOR_2_DEG) && (Driversensor_currentRead - current_tempRead <= ADC_CODE_FOR_5_DEG))
                {
                    Seat1_heater_Speed = Intensity_Low;
                    GPIO_GreenLedOn();
                    GPIO_BlueLedOff();
                }
                else if(Driversensor_currentRead < current_tempRead)
                {
                    Seat1_heater_Speed = Intensity_Disable;
                    GPIO_GreenLedOff();
                    GPIO_BlueLedOff();
                }
                else
                {
                    /* No Action Required */
                }

                break;

            case Heater_Medium:

                /* From requirements the temperature match with heater  in low level is 30 degrees */
                Driversensor_currentRead = ADC_CODE_FOR_30_DEG;

                /*
                 * 1- If the current temperature is less than the desired temperature by 10°C or more the heater
                 *    should be enabled with the high intensity.
                 * 2- If the current temperature is less than the desired temperature by 5°C to 10°C the heater
                 *    should be enabled with a medium intensity
                 * 3- If the current temperature is less than the desired temperature by 2°C to 5°C the heater should
                 *    be enabled with a low intensity.
                 * 4- If the current temperature is more than the desired temperature the heater should be
                 *    disabled.
                 * 5- The heater shall be enabled once again if the temperature becomes less than the desired
                 *    temperature by 3°C.
                 *
                 */
                if( Driversensor_currentRead - current_tempRead == ADC_CODE_FOR_10_DEG )
                {
                    Seat1_heater_Speed = Intensity_High;
                    GPIO_GreenLedOn();
                    GPIO_BlueLedOn();
                }
                else if( (Driversensor_currentRead - current_tempRead >= ADC_CODE_FOR_5_DEG) && (Driversensor_currentRead - current_tempRead <= ADC_CODE_FOR_10_DEG))
                {
                    Seat1_heater_Speed = Intensity_Medium;
                    GPIO_GreenLedOff();
                    GPIO_BlueLedOn();
                }
                else if( (Driversensor_currentRead - current_tempRead >= ADC_CODE_FOR_2_DEG) && (Driversensor_currentRead - current_tempRead <= ADC_CODE_FOR_5_DEG))
                {
                    Seat1_heater_Speed = Intensity_Low;
                    GPIO_GreenLedOn();
                    GPIO_BlueLedOff();
                }
                else if(Driversensor_currentRead < current_tempRead)
                {
                    Seat1_heater_Speed = Intensity_Disable;
                    GPIO_GreenLedOff();
                    GPIO_BlueLedOff();
                }
                else
                {
                    /* No Action Required */
                }


                break;

            case Heater_High:

                /* From requirements the temperature match with heater  in low level is 35 degrees */
                Driversensor_currentRead = ADC_CODE_FOR_35_DEG;

                /*
                 * 1- If the current temperature is less than the desired temperature by 10°C or more the heater
                 *    should be enabled with the high intensity.
                 * 2- If the current temperature is less than the desired temperature by 5°C to 10°C the heater
                 *    should be enabled with a medium intensity
                 * 3- If the current temperature is less than the desired temperature by 2°C to 5°C the heater should
                 *    be enabled with a low intensity.
                 * 4- If the current temperature is more than the desired temperature the heater should be
                 *    disabled.
                 * 5- The heater shall be enabled once again if the temperature becomes less than the desired
                 *    temperature by 3°C.
                 *
                 */
                if( Driversensor_currentRead - current_tempRead == ADC_CODE_FOR_10_DEG )
                {
                    Seat1_heater_Speed = Intensity_High;
                    GPIO_GreenLedOn();
                    GPIO_BlueLedOn();
                }
                else if( (Driversensor_currentRead - current_tempRead >= ADC_CODE_FOR_5_DEG) && (Driversensor_currentRead - current_tempRead <= ADC_CODE_FOR_10_DEG))
                {
                    Seat1_heater_Speed = Intensity_Medium;
                    GPIO_GreenLedOff();
                    GPIO_BlueLedOn();
                }
                else if( (Driversensor_currentRead - current_tempRead >= ADC_CODE_FOR_2_DEG) && (Driversensor_currentRead - current_tempRead <= ADC_CODE_FOR_5_DEG))
                {
                    Seat1_heater_Speed = Intensity_Low;
                    GPIO_GreenLedOn();
                    GPIO_BlueLedOff();
                }
                else if(Driversensor_currentRead < current_tempRead)
                {
                    Seat1_heater_Speed = Intensity_Disable;
                    GPIO_GreenLedOff();
                    GPIO_BlueLedOff();
                }
                else
                {
                    /* No Action Required */
                }

                break;

            }/* End of Switch on Heater Level*/

        }/* End of condition if the temperature is in range of temperatures that sensor can read */

        /* Repeat this task each 500 m second */
        xTaskDelayUntil(&xLastWakeTime, 500);

    }/* End of a Task */
}

void vPassengerSeat_Temperature(void *pvParameters)
{
    /*
     * From the TIVAC manual we can find that:
     *
     * TEMP = 147.5 - ((75 * (VREFP - VREFN) × ADCCODE) / 4096)
     *
     * ADCCODE = ( -(TEMP - 147.5) * 4096 ) / (75 * 3.3)
     *
     * Heater status :
     * a. Low    --> 25°C.
     * b. Medium --> 30°C.
     * d. High   --> 35°C.
     *
     */
    uint32 current_tempRead = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        /**************************************************************
         * NOTE: the level of heater determines by another task.      *
         **************************************************************/
        current_tempRead = ADC1_read;

        /*****************************************************************************************
         * NOTE: the Sensor always give read to make it easy to detect if there is an issue in it*
         ****************************************************************************************/

        /*
         * Check if the sensor read less than 5 degrees or greater than 40 degrees
         * in this case send this value to the corresponding errors queue of this sensor.
         *
         */
        if( current_tempRead < TEMP_MIN_VALUE || current_tempRead > TEMP_MAX_VALUE)
        {
            /*
             * If the queue not full and have a space add the un-needed value to queue,
             * if full set the error flag and another task will handle this error and
             * then resume the task that handle this bug.
             *
             */
            if( uxQueueSpacesAvailable(xError_passengerSensor_Reads_Queue) != 0 )
            {
                xQueueSend(xError_passengerSensor_Reads_Queue,&current_tempRead,portMAX_DELAY);
            }
            else
            {
                /* For making sure that the error will be detected */
                xEventGroupSetBits(xEvent_Group, PASSENGER_QUEUE_ERROR_FLAG);

                /* This task will be resumed if any error happen in any sensor */
                vTaskResume(xSensor_efficiency);
            }
        }
        else
        {
            /* No Action Required */
        }

        /* Check on the speed of the motor */
        if(Seat2_heater_Status == Heater_Off)
        {
            /*
             * No Action Required because the feature is off, and the temperature is not controlled
             * so the heater will be disabled
             */
            Seat2_heater_Speed = Intensity_Disable;
        }
        else if( Seat2_heater_Status != Heater_Off)
        {

            switch(Seat2_heater_Status)
            {
            case Heater_Low:

                /* From requirements the temperature match with heater  in low level is 25 degrees */
                Passengersensor_currentRead = ADC_CODE_FOR_25_DEG;

                /*
                 * 1- If the current temperature is less than the desired temperature by 10°C or more the heater
                 *    should be enabled with the high intensity.
                 * 2- If the current temperature is less than the desired temperature by 5°C to 10°C the heater
                 *    should be enabled with a medium intensity
                 * 3- If the current temperature is less than the desired temperature by 2°C to 5°C the heater should
                 *    be enabled with a low intensity.
                 * 4- If the current temperature is more than the desired temperature the heater should be
                 *    disabled.
                 * 5- The heater shall be enabled once again if the temperature becomes less than the desired
                 *    temperature by 3°C.
                 *
                 */
                if( Passengersensor_currentRead - current_tempRead == ADC_CODE_FOR_10_DEG )
                {
                    Seat2_heater_Speed = Intensity_High;
                    GPIO_GreenLedOn();
                    GPIO_BlueLedOn();
                }
                else if( (Passengersensor_currentRead - current_tempRead >= ADC_CODE_FOR_5_DEG) && (Passengersensor_currentRead - current_tempRead <= ADC_CODE_FOR_10_DEG))
                {
                    Seat2_heater_Speed = Intensity_Medium;
                    GPIO_GreenLedOff();
                    GPIO_BlueLedOn();
                }
                else if( (Passengersensor_currentRead - current_tempRead >= ADC_CODE_FOR_2_DEG) && (Passengersensor_currentRead - current_tempRead <= ADC_CODE_FOR_5_DEG))
                {
                    Seat2_heater_Speed = Intensity_Low;
                    GPIO_GreenLedOn();
                    GPIO_BlueLedOff();
                }
                else if(Passengersensor_currentRead < current_tempRead)
                {
                    Seat2_heater_Speed = Intensity_Disable;
                    GPIO_GreenLedOff();
                    GPIO_BlueLedOff();
                }
                else
                {
                    /* No Action Required */
                }

                break;

            case Heater_Medium:

                /* From requirements the temperature match with heater in low level is 30 degrees */
                Passengersensor_currentRead = ADC_CODE_FOR_30_DEG;

                /*
                 * 1- If the current temperature is less than the desired temperature by 10°C or more the heater
                 *    should be enabled with the high intensity.
                 * 2- If the current temperature is less than the desired temperature by 5°C to 10°C the heater
                 *    should be enabled with a medium intensity
                 * 3- If the current temperature is less than the desired temperature by 2°C to 5°C the heater should
                 *    be enabled with a low intensity.
                 * 4- If the current temperature is more than the desired temperature the heater should be
                 *    disabled.
                 * 5- The heater shall be enabled once again if the temperature becomes less than the desired
                 *    temperature by 3°C.
                 *
                 */
                if( Passengersensor_currentRead - current_tempRead == ADC_CODE_FOR_10_DEG )
                {
                    Seat2_heater_Speed = Intensity_High;
                    GPIO_GreenLedOn();
                    GPIO_BlueLedOn();
                }
                else if( (Passengersensor_currentRead - current_tempRead >= ADC_CODE_FOR_5_DEG) && (Passengersensor_currentRead - current_tempRead <= ADC_CODE_FOR_10_DEG))
                {
                    Seat2_heater_Speed = Intensity_Medium;
                    GPIO_GreenLedOff();
                    GPIO_BlueLedOn();
                }
                else if( (Passengersensor_currentRead - current_tempRead >= ADC_CODE_FOR_2_DEG) && (Passengersensor_currentRead - current_tempRead <= ADC_CODE_FOR_5_DEG))
                {
                    Seat2_heater_Speed = Intensity_Low;
                    GPIO_GreenLedOn();
                    GPIO_BlueLedOff();
                }
                else if(Passengersensor_currentRead < current_tempRead)
                {
                    Seat2_heater_Speed = Intensity_Disable;
                    GPIO_GreenLedOff();
                    GPIO_BlueLedOff();
                }
                else
                {
                    /* No Action Required */
                }


                break;

            case Heater_High:

                /* From requirements the temperature match with heater  in low level is 35 degrees */
                Passengersensor_currentRead = ADC_CODE_FOR_35_DEG;

                /*
                 * 1- If the current temperature is less than the desired temperature by 10°C or more the heater
                 *    should be enabled with the high intensity.
                 * 2- If the current temperature is less than the desired temperature by 5°C to 10°C the heater
                 *    should be enabled with a medium intensity
                 * 3- If the current temperature is less than the desired temperature by 2°C to 5°C the heater should
                 *    be enabled with a low intensity.
                 * 4- If the current temperature is more than the desired temperature the heater should be
                 *    disabled.
                 * 5- The heater shall be enabled once again if the temperature becomes less than the desired
                 *    temperature by 3°C.
                 *
                 */
                if( Passengersensor_currentRead - current_tempRead == ADC_CODE_FOR_10_DEG )
                {
                    Seat2_heater_Speed = Intensity_High;
                    GPIO_GreenLedOn();
                    GPIO_BlueLedOn();
                }
                else if( (Passengersensor_currentRead - current_tempRead >= ADC_CODE_FOR_5_DEG) && (Passengersensor_currentRead - current_tempRead <= ADC_CODE_FOR_10_DEG))
                {
                    Seat2_heater_Speed = Intensity_Medium;
                    GPIO_GreenLedOff();
                    GPIO_BlueLedOn();
                }
                else if( (Passengersensor_currentRead - current_tempRead >= ADC_CODE_FOR_2_DEG) && (Passengersensor_currentRead - current_tempRead <= ADC_CODE_FOR_5_DEG))
                {
                    Seat2_heater_Speed = Intensity_Low;
                    GPIO_GreenLedOn();
                    GPIO_BlueLedOff();
                }
                else if(Passengersensor_currentRead < current_tempRead)
                {
                    Seat2_heater_Speed = Intensity_Disable;
                    GPIO_GreenLedOff();
                    GPIO_BlueLedOff();
                }
                else
                {
                    /* No Action Required */
                }

                break;

            }/* End of Switch on Heater Level*/

        }/* End of condition if the temperature is in range of temperatures that sensor can read */

        /* Repeat this task each 500 m second */
        xTaskDelayUntil(&xLastWakeTime, 500);

    }/* End of a Task */
}

void vDriverSensor_displayData(void *pvParameters)
{

    uint32 Real_Temp;
    uint32 Current_ADC_Readig = ADC0_read;
    uint8 heater_state = Seat1_heater_Status;
    uint8 heating_level = Seat1_heater_Speed;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        /* Check if the Source Available or Not */
        if( xSemaphoreTake(xUART0_Avilabel,portMAX_DELAY) == pdTRUE )
        {

            UART0_SendString("The Current Temperature of Driver Seat is : ");
            Real_Temp = 147.5 - ( ( 75 * 3.3 * Current_ADC_Readig ) / 4096);
            UART0_SendInteger(Real_Temp);
            UART0_SendString(" C\n");

            UART0_SendString("The Heating Level of Driver Seat is : ");
            switch(heating_level)
            {
            case Heater_Off:
                UART0_SendString("OFF\n");
                break;
            case Heater_Low:
                UART0_SendString("LOW\n");
                break;
            case Heater_Medium:
                UART0_SendString("MEDIUM\n");
                break;
            case Heater_High:
                UART0_SendString("HIGH\n");
                break;
            }

            UART0_SendString("The Heater State of Driver Seat is : ");
            switch(heater_state)
            {
            case Intensity_Disable:
                UART0_SendString("Disable\n");
                break;
            case Intensity_Low:
                UART0_SendString("Enable -> Low\n");
                break;
            case Intensity_Medium:
                UART0_SendString("Enable -> Medium\n");
                break;
            case Intensity_High:
                UART0_SendString("Enable -> High\n");
                break;
            }

            UART0_SendString("===================================================\n");

            /* Finally the task must gives the MUTEX */
            xSemaphoreGive(xUART0_Avilabel);
        }

        /* Repeat this task each 1000 m second */
        xTaskDelayUntil(&xLastWakeTime, 1000);
    }
}

void vPassengerSensor_displayData(void *pvParameters)
{
    uint32 Real_Temp;
    uint32 Current_ADC_Readig = ADC1_read;
    uint8 heater_state = Seat2_heater_Status;
    uint8 heating_level = Seat2_heater_Speed;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        /* Check if the UART Available or Not */
        if( xSemaphoreTake(xUART0_Avilabel,portMAX_DELAY) == pdTRUE )
        {

            UART0_SendString("The Current Temperature of Passenger Seat is : ");
            Real_Temp = 147.5 - ( ( 75 * 3.3 * Current_ADC_Readig ) / 4096);
            UART0_SendInteger(Real_Temp);
            UART0_SendString(" C\n");

            UART0_SendString("The Heating Level of Passenger Seat is ");
            switch(heating_level)
            {
            case Heater_Off:
                UART0_SendString("OFF\n");
                break;
            case Heater_Low:
                UART0_SendString("LOW\n");
                break;
            case Heater_Medium:
                UART0_SendString("MEDIUM\n");
                break;
            case Heater_High:
                UART0_SendString("HIGH\n");
                break;
            }

            UART0_SendString("The Heater State of Passenger Seat is : ");
            switch(heater_state)
            {
            case Intensity_Disable:
                UART0_SendString("Disable\n");
                break;
            case Intensity_Low:
                UART0_SendString("Enable -> Low\n");
                break;
            case Intensity_Medium:
                UART0_SendString("Enable -> Medium\n");
                break;
            case Intensity_High:
                UART0_SendString("Enable -> High\n");
                break;
            }

            UART0_SendString("===================================================\n");

            /* Finally the task must gives the MUTEX */
            xSemaphoreGive(xUART0_Avilabel);
        }
        /* Repeat this task each 1000 m second */
        xTaskDelayUntil(&xLastWakeTime, 1000);
    }
}

void vSeat_sensorEfficiency(void *pvParameters)
{
    EventBits_t xEventGroupValue;
    const EventBits_t xBitsToWaitFor = ( DRIVER_QUEUE_ERROR_FLAG | PASSENGER_QUEUE_ERROR_FLAG);

    for(;;)
    {
        xEventGroupValue = xEventGroupWaitBits(xEvent_Group, xBitsToWaitFor, pdTRUE, NULL, portMAX_DELAY);

        if( (xEventGroupValue & DRIVER_QUEUE_ERROR_FLAG) != 0)
        {
            /* Check if the UART Available or Not */
            if(xSemaphoreTake(xUART0_Avilabel, portMAX_DELAY) == pdTRUE )
            {
                if( (xEventGroupValue & DRIVER_QUEUE_ERROR_FLAG) == pdTRUE )
                {
                    /* Give Notification to the driver that is an error occurs */
                    UART0_SendString("!WAR0! : You Should Check Sensor of Heater of Driver Seat\n");
                    UART0_SendString(" The Driver Seat Stop From Controlling The Temperature\n");

                    GPIO_GreenLedOff();
                    GPIO_BlueLedOff();
                    GPIO_RedLedOn();

                    /*****************************************************************************************
                     * NOTE: The driver sensor now out of control you should the control ECU.                *
                     ****************************************************************************************/
                    /*
                     * suspend the task that control the temperature of the heater of the driver
                     * that means this task not scheduled again (user losses controlling on sensor).
                     *
                     */
                    vTaskSuspend(xDriver_SeatTemp);
                }

                if( (xEventGroupValue && PASSENGER_QUEUE_ERROR_FLAG) == pdTRUE )
                {
                    /* Give Notification to the passenger that is an error occurs */
                    UART0_SendString("!WAR1! : You Should Check Sensor of Heater of Passenger Seat \n");
                    UART0_SendString(" The Passenger Seat Stop From Controlling The Temperature\n");

                    GPIO_GreenLedOff();
                    GPIO_BlueLedOff();
                    GPIO_RedLedOn();

                    /*****************************************************************************************
                     * NOTE: The passenger sensor now out of control you should the control ECU.             *
                     ****************************************************************************************/
                    /*
                     * suspend the task that control the temperature of the heater of the passenger
                     * that means this task not scheduled again (user losses controlling on sensor).
                     *
                     */
                    vTaskSuspend(xPassenger_SeatTemp);
                }

                xSemaphoreGive(xUART0_Avilabel);
            }
        }
    }

}

void vRunTimeMeasurementsTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        uint8 ucCounter, ucCPU_Load;
        uint32 TotalTasksTime = 0;

        if( xSemaphoreTake(xUART0_Avilabel,portMAX_DELAY) )
        {
            for(ucCounter = 1; ucCounter < 4; ucCounter++)
            {
                TotalTasksTime += ullTasksTotalTime[ucCounter];
            }

            ucCPU_Load = (TotalTasksTime * 100) /  GPTM_WTimer0Read();

            UART0_SendString("=======================================================================\n");

            UART0_SendString("The CPU Load of the System is ");
            UART0_SendInteger(ucCPU_Load);
            UART0_SendString(" %\n");

            UART0_SendString("Each Task Time : \n");

            UART0_SendString("The Time Consumed by Task - vDriverSensor_displayData is ");
            UART0_SendInteger(ullTasksTotalTime[1]);
            UART0_SendString(" sec\n");

            UART0_SendString("The Time Consumed by Task - vPassengerSensor_displayData is ");
            UART0_SendInteger(ullTasksTotalTime[2]);
            UART0_SendString(" sec\n");

            UART0_SendString("The Time Consumed by Task - vPeriodic_driver_TeampRead is");
            UART0_SendInteger(ullTasksTotalTime[3]);
            UART0_SendString(" sec\n");

            UART0_SendString("The Time Consumed by Task - vPeriodic_passnegerSensor_TeampRead is ");
            UART0_SendInteger(ullTasksTotalTime[4]);
            UART0_SendString(" %\n");

            UART0_SendString("The Time Consumed by Task - vDriver_Seat_Button is ");
            UART0_SendInteger(ullTasksTotalTime[5]);
            UART0_SendString(" sec\n");

            UART0_SendString("The Time Consumed by Task - vPassenger_Seat_Button is ");
            UART0_SendInteger(ullTasksTotalTime[6]);
            UART0_SendString(" sec\n");

            UART0_SendString("The Time Consumed by Task - vSteering_Button is");
            UART0_SendInteger(ullTasksTotalTime[7]);
            UART0_SendString(" sec\n");

            UART0_SendString("The Time Consumed by Task - vDriverSeat_Temperature is ");
            UART0_SendInteger(ullTasksTotalTime[8]);
            UART0_SendString(" %\n");

            UART0_SendString("The Time Consumed by Task - vPassengerSeat_Temperature is ");
            UART0_SendInteger(ullTasksTotalTime[9]);
            UART0_SendString(" sec\n");

            UART0_SendString("The Time Consumed by Task - vSeat_sensorEfficiency is ");
            UART0_SendInteger(ullTasksTotalTime[10]);
            UART0_SendString(" sec\n");

            UART0_SendString("The Time Consumed by Task - vRunTimeMeasurementsTask is");
            UART0_SendInteger(ullTasksTotalTime[11]);
            UART0_SendString(" sec\n");

            UART0_SendString("==========================End of Functions Time =======================");

            xSemaphoreGive(xUART0_Avilabel);
        }

        /* Repeat this task each 3 seconds */
        xTaskDelayUntil(&xLastWakeTime, 1000);
    }
}
/*--------------------------- End of the Application --------------------------------*/
