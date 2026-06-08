// working 3mA in deep sleep

/*============================================================================
 * KnockLock — PSoC 6 / PSoC Creator 4.4
 *
 * Hardware
 *   - MG996R servo motor          → PWM_Servo component
 *   - LIS3DH accelerometer        → SensorBus (I2C) component
 *   - Adafruit 4x4 matrix keypad  → GPIO (Port 10 / Port 5)
 *   - Status LED                  → LED component (active-low GPIO)
 *   - UART terminal               → UART component
 *   - Push button                 → BUTTON GPIO pin (active-low, falling edge)
 *
 * Lock flow
 *   1. User enters PIN on keypad and presses '#'.
 *   2. If PIN matches, LED blinks to prompt knock sequence.
 *   3. Accelerometer detects knocks; inter-knock intervals are recorded.
 *   4. If knock pattern matches, servo moves to UNLOCK position.
 *   5. Lock STAYS UNLOCKED until:
 *        a. Button press  → toggles lock state
 *        b. '*' on keypad → locks immediately
 *============================================================================*/

#include "project.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>      /* sqrtf */

/*---------------------------------------------------------------------------
 * User-configurable credentials
 *---------------------------------------------------------------------------*/
#define SECRET_PIN          "1234"      /* keypad PIN (terminate with '#') */

/*---------------------------------------------------------------------------
 * Servo (MG996R) — pulse widths match Arduino Servo.h defaults
 *---------------------------------------------------------------------------*/
#define MIN_PULSE_WIDTH     544u        /* µs — 0 °                        */
#define MAX_PULSE_WIDTH     2400u       /* µs — 180 °                      */
#define SERVO_MAX_DEG       180u
#define SERVO_LOCK_DEG      45u  // good: 135u
#define SERVO_UNLOCK_DEG    135u   // good: 45u

/*---------------------------------------------------------------------------
 * Keypad (Adafruit 4×4) pin mapping
 *---------------------------------------------------------------------------*/
/* Row pins */
#define ROW0_PORT   GPIO_PRT10
#define ROW0_PIN    2u
#define ROW1_PORT   GPIO_PRT10
#define ROW1_PIN    1u
#define ROW2_PORT   GPIO_PRT10
#define ROW2_PIN    0u
#define ROW3_PORT   GPIO_PRT5
#define ROW3_PIN    6u
/* Column pins */
#define COL0_PORT   GPIO_PRT10
#define COL0_PIN    6u
#define COL1_PORT   GPIO_PRT10
#define COL1_PIN    5u
#define COL2_PORT   GPIO_PRT10
#define COL2_PIN    4u
#define COL3_PORT   GPIO_PRT10
#define COL3_PIN    3u

static const char keyMap[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

/* Knock detection */
#define KNOCK_THRESHOLD        10.0f   /* m/s² spike needed to register knock */
#define KNOCK_DEBOUNCE_MS      200    /* minimum ms before next knock         */
#define TARGET_DELAY_MS        1000   /* target inter-knock gap (ms)          */
#define TOLERANCE_MS           500    /* ± ms human timing tolerance          */

/* Knock state machine states */
#define STATE_LOCKED_WAIT_K1   0
#define STATE_LOCKED_WAIT_K2   1
#define STATE_UNLOCKED         2

/* PIN entry inactivity timeout */
#define PIN_TIMEOUT_MS         10000u /* 20 seconds of no keypresses → sleep  */

/*---------------------------------------------------------------------------
 * Global lock state — single source of truth
 *---------------------------------------------------------------------------*/
static volatile bool isUnlocked = false;

/*---------------------------------------------------------------------------
 * I2C transfer state
 *---------------------------------------------------------------------------*/
static cy_stc_scb_i2c_master_xfer_config_t register_setting;
static uint8 rbuff[2];
static uint8 wbuff[2];

/*===========================================================================
 * Button interrupt — now a toggle
 *===========================================================================*/
static volatile bool buttonToggleFlag = false;

#define BUTTON_PORT     GPIO_PRT12
#define BUTTON_PIN      7u

static const cy_stc_sysint_t buttonIntCfg = {
    .intrSrc      = ioss_interrupts_gpio_12_IRQn,
    .intrPriority = 3u
};

static void Button_ISR(void)
{
    Cy_GPIO_ClearInterrupt(BUTTON_PORT, BUTTON_PIN);
    NVIC_ClearPendingIRQ(ioss_interrupts_gpio_12_IRQn);
    buttonToggleFlag = true;
}

static void Button_Init(void)
{
    Cy_GPIO_SetInterruptEdge(BUTTON_PORT, BUTTON_PIN, CY_GPIO_INTR_FALLING);
    Cy_GPIO_SetInterruptMask(BUTTON_PORT, BUTTON_PIN, 1u);
    Cy_SysInt_Init(&buttonIntCfg, Button_ISR);
    NVIC_ClearPendingIRQ(ioss_interrupts_gpio_12_IRQn);
    NVIC_EnableIRQ(ioss_interrupts_gpio_12_IRQn);
}

/*===========================================================================
 * Low-level helpers
 *===========================================================================*/

/* ---------- LED helpers ---------- */
static void RedLED_On(void)  { Cy_GPIO_Write(EXT_RED_LED_PORT,   EXT_RED_LED_NUM,   1); }
static void RedLED_Off(void) { Cy_GPIO_Write(EXT_RED_LED_PORT,   EXT_RED_LED_NUM,   0); }
static void RedLED_Blink(uint8_t times, uint32_t on_ms, uint32_t off_ms)
{
    for (uint8_t i = 0u; i < times; i++)
    { RedLED_On(); CyDelay(on_ms); RedLED_Off(); CyDelay(off_ms); }
}

static void GreenLED_On(void)  { Cy_GPIO_Write(EXT_GREEN_LED_PORT, EXT_GREEN_LED_NUM, 1); }
static void GreenLED_Off(void) { Cy_GPIO_Write(EXT_GREEN_LED_PORT, EXT_GREEN_LED_NUM, 0); }
static void GreenLED_Blink(uint8_t times, uint32_t on_ms, uint32_t off_ms)
{
    for (uint8_t i = 0u; i < times; i++)
    { GreenLED_On(); CyDelay(on_ms); GreenLED_Off(); CyDelay(off_ms); }
}


/* ---------- Servo ---------- */
static uint32_t AngleToCompare(uint32_t degrees)
{
    if (degrees > SERVO_MAX_DEG) degrees = SERVO_MAX_DEG;
    return MIN_PULSE_WIDTH +
           (degrees * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)) / SERVO_MAX_DEG;
}

static void Servo_SetAngle(uint32_t degrees)
{
    PWM_Servo_SetCompare0(AngleToCompare(degrees));
}

/*---------------------------------------------------------------------------
 * Lock / Unlock actions — single point for servo + LED + UART
 *---------------------------------------------------------------------------*/
static void Lock_Engage(void)
{
    Servo_SetAngle(SERVO_LOCK_DEG);
    GreenLED_Off();
    RedLED_Off();
    isUnlocked = false;
    UART_PutString("Lock ENGAGED.\r\n");
}

static void Lock_Disengage(void)
{
    Servo_SetAngle(SERVO_UNLOCK_DEG);
    GreenLED_On();
    RedLED_Off();
    isUnlocked = true;
    UART_PutString("Lock DISENGAGED.\r\n");
}

/* ---------- Keypad ---------- */
static void Rows_Idle(void)
{
    Cy_GPIO_Set(ROW0_PORT, ROW0_PIN);
    Cy_GPIO_Set(ROW1_PORT, ROW1_PIN);
    Cy_GPIO_Set(ROW2_PORT, ROW2_PIN);
    Cy_GPIO_Set(ROW3_PORT, ROW3_PIN);
}

static void Row_Select(uint8_t row)
{
    Rows_Idle();
    switch (row)
    {
        case 0u: Cy_GPIO_Clr(ROW0_PORT, ROW0_PIN); break;
        case 1u: Cy_GPIO_Clr(ROW1_PORT, ROW1_PIN); break;
        case 2u: Cy_GPIO_Clr(ROW2_PORT, ROW2_PIN); break;
        case 3u: Cy_GPIO_Clr(ROW3_PORT, ROW3_PIN); break;
        default: break;
    }
}

static uint8_t Col_Read(uint8_t col)
{
    switch (col)
    {
        case 0u: return (uint8_t)Cy_GPIO_Read(COL0_PORT, COL0_PIN);
        case 1u: return (uint8_t)Cy_GPIO_Read(COL1_PORT, COL1_PIN);
        case 2u: return (uint8_t)Cy_GPIO_Read(COL2_PORT, COL2_PIN);
        case 3u: return (uint8_t)Cy_GPIO_Read(COL3_PORT, COL3_PIN);
        default: return 1u;
    }
}

static char Keypad_GetKey(void)
{
    for (uint8_t row = 0u; row < 4u; row++)
    {
        Row_Select(row);
        Cy_SysLib_DelayUs(10u);
        for (uint8_t col = 0u; col < 4u; col++)
        {
            if (Col_Read(col) == 0u)
            {
                Rows_Idle();
                return keyMap[row][col];
            }
        }
    }
    Rows_Idle();
    return '\0';
}

/*---------------------------------------------------------------------------
 * Keypad debounce
 *---------------------------------------------------------------------------*/
#define KEY_DEBOUNCE_MS     20u
#define KEY_POLL_INTERVAL   5u
#define KEY_STABLE_COUNT    (KEY_DEBOUNCE_MS / KEY_POLL_INTERVAL)

typedef enum {
    KD_IDLE = 0,
    KD_DEBOUNCE_PRESS,
    KD_PRESSED,
    KD_DEBOUNCE_RELEASE
} KeyDebounceState;

static char Keypad_Debounce(void)
{
    static KeyDebounceState state       = KD_IDLE;
    static char             candidate   = '\0';
    static uint8_t          stableCount = 0u;

    char current = Keypad_GetKey();

    switch (state)
    {
        case KD_IDLE:
            if (current != '\0')
            {
                candidate   = current;
                stableCount = 1u;
                state       = KD_DEBOUNCE_PRESS;
            }
            break;

        case KD_DEBOUNCE_PRESS:
            if (current == candidate)
            {
                stableCount++;
                if (stableCount >= KEY_STABLE_COUNT)
                {
                    stableCount = 0u;
                    state       = KD_PRESSED;
                    return candidate;
                }
            }
            else
            {
                candidate   = current;
                stableCount = (current != '\0') ? 1u : 0u;
                if (current == '\0') state = KD_IDLE;
            }
            break;

        case KD_PRESSED:
            if (current == '\0')
            {
                stableCount = 1u;
                state       = KD_DEBOUNCE_RELEASE;
            }
            break;

        case KD_DEBOUNCE_RELEASE:
            if (current == '\0')
            {
                stableCount++;
                if (stableCount >= KEY_STABLE_COUNT)
                {
                    stableCount = 0u;
                    state       = KD_IDLE;
                }
            }
            else
            {
                stableCount = 0u;
            }
            break;

        default:
            state = KD_IDLE;
            break;
    }

    return '\0';
}

static char Keypad_WaitKey(void)
{
    char key;
    do
    {
        key = Keypad_Debounce();
        if (key == '\0') CyDelay(KEY_POLL_INTERVAL);
    } while (key == '\0');
    return key;
}

/*---------------------------------------------------------------------------
 * Keypad_Poll
 *
 * Non-blocking version of Keypad_Debounce for use in the unlocked idle loop.
 * Returns the confirmed key or '\0' if nothing new.  Call every
 * KEY_POLL_INTERVAL ms.
 *---------------------------------------------------------------------------*/
static char Keypad_Poll(void)
{
    return Keypad_Debounce();   /* re-uses the same debounce state machine  */
}

/* ---------- I2C / LIS3DH ---------- */
static int WaitForOperation()
{
    uint32 timeout = 20000;
    while(0 != (SensorBus_MasterGetStatus() & CY_SCB_I2C_MASTER_BUSY))
    {
        CyDelayUs(1);
        if(--timeout == 0) return -1;
    }
    return 0;
}

static void WriteRegister(uint8 reg_addr, uint8 data)
{
    wbuff[0] = reg_addr;
    wbuff[1] = data;
    register_setting.buffer = wbuff;
    register_setting.bufferSize = 2;
    register_setting.xferPending = false;
    SensorBus_MasterWrite(&register_setting);
    if(WaitForOperation() != 0) {
        printf("CRITICAL ERROR: I2C Write Timed Out (Bus Stuck)\r\n");
    }
}

static uint8 ReadRegister(uint8 reg_addr)
{
    wbuff[0] = reg_addr;
    register_setting.buffer = wbuff;
    register_setting.bufferSize = 1;
    register_setting.xferPending = true;
    SensorBus_MasterWrite(&register_setting);
    WaitForOperation();
    register_setting.buffer = rbuff;
    register_setting.xferPending = false;
    SensorBus_MasterRead(&register_setting);
    WaitForOperation();
    return rbuff[0];
}



/*===========================================================================
 * Power management
 *===========================================================================*/
static void LIS3DH_PowerDown(void)  { WriteRegister(0x20, 0x07); }
static void LIS3DH_PowerUp(void)
{
    WriteRegister(0x20, 0x57);
    WriteRegister(0x23, 0x88);
    CyDelay(70u);
}

static volatile bool keyWakeFlag = false;

static void Keypad_WakeISR(void)
{
    keyWakeFlag = true;
    Cy_GPIO_ClearInterrupt(COL0_PORT, COL0_PIN);
    Cy_GPIO_ClearInterrupt(COL1_PORT, COL1_PIN);
    Cy_GPIO_ClearInterrupt(COL2_PORT, COL2_PIN);
    Cy_GPIO_ClearInterrupt(COL3_PORT, COL3_PIN);
    NVIC_ClearPendingIRQ(ioss_interrupts_gpio_10_IRQn);
}

static const cy_stc_sysint_t wakeIntCfg = {
    .intrSrc      = ioss_interrupts_gpio_10_IRQn,
    .intrPriority = 7u
};

static void Keypad_EnableWakeInterrupt(void)
{
    Cy_GPIO_Clr(ROW0_PORT, ROW0_PIN);
    Cy_GPIO_Clr(ROW1_PORT, ROW1_PIN);
    Cy_GPIO_Clr(ROW2_PORT, ROW2_PIN);
    Cy_GPIO_Clr(ROW3_PORT, ROW3_PIN);
    Cy_GPIO_SetInterruptEdge(COL0_PORT, COL0_PIN, CY_GPIO_INTR_FALLING);
    Cy_GPIO_SetInterruptEdge(COL1_PORT, COL1_PIN, CY_GPIO_INTR_FALLING);
    Cy_GPIO_SetInterruptEdge(COL2_PORT, COL2_PIN, CY_GPIO_INTR_FALLING);
    Cy_GPIO_SetInterruptEdge(COL3_PORT, COL3_PIN, CY_GPIO_INTR_FALLING);
    Cy_GPIO_SetInterruptMask(COL0_PORT, COL0_PIN, 1u);
    Cy_GPIO_SetInterruptMask(COL1_PORT, COL1_PIN, 1u);
    Cy_GPIO_SetInterruptMask(COL2_PORT, COL2_PIN, 1u);
    Cy_GPIO_SetInterruptMask(COL3_PORT, COL3_PIN, 1u);
    keyWakeFlag = false;
    NVIC_ClearPendingIRQ(ioss_interrupts_gpio_10_IRQn);
    NVIC_EnableIRQ(ioss_interrupts_gpio_10_IRQn);
}

static void Keypad_DisableWakeInterrupt(void)
{
    NVIC_DisableIRQ(ioss_interrupts_gpio_10_IRQn);
    Cy_GPIO_SetInterruptMask(COL0_PORT, COL0_PIN, 0u);
    Cy_GPIO_SetInterruptMask(COL1_PORT, COL1_PIN, 0u);
    Cy_GPIO_SetInterruptMask(COL2_PORT, COL2_PIN, 0u);
    Cy_GPIO_SetInterruptMask(COL3_PORT, COL3_PIN, 0u);
    Rows_Idle();
}

/*---------------------------------------------------------------------------
 * System_EnterDeepSleep
 * Only called when the lock is in the LOCKED state.
 *---------------------------------------------------------------------------*/
static void System_EnterDeepSleep(void)
{
    UART_PutString("Idle - press any key or button to wake.\r\n");

    Servo_SetAngle(SERVO_LOCK_DEG);
    CyDelay(500);

    LIS3DH_PowerDown();
    SensorBus_Disable();

    Keypad_EnableWakeInterrupt();

    Cy_GPIO_Write(DEBUG_PIN_PORT, DEBUG_PIN_NUM, 0);
    Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
    Cy_GPIO_Write(DEBUG_PIN_PORT, DEBUG_PIN_NUM, 1);

    Keypad_DisableWakeInterrupt();

    SensorBus_Enable();
    register_setting.slaveAddress = 0x19;
    LIS3DH_PowerUp();

    Servo_SetAngle(SERVO_LOCK_DEG);
    CyDelay(200);

    UART_PutString("System active.\r\n");
}

/*===========================================================================
 * Application logic
 *===========================================================================*/

/* ---------- PIN entry ----------
 *
 * Returns true  → PIN matched, proceed to knock verification.
 * Returns false → wrong PIN, timeout, or '*' cancel; caller returns to sleep.
 *
 * Timeout: if no key is pressed for PIN_TIMEOUT_MS (20 s) the function
 * aborts and returns false so the system goes back to deep sleep.
 * The inactivity timer resets on every confirmed keypress, so a user who
 * is simply typing slowly will not be timed out mid-entry.
 *---------------------------------------------------------------------------*/
static bool PIN_Verify(void)
{
    char     buf[16];
    uint8_t  idx       = 0u;
    uint32_t elapsedMs = 0u;   /* inactivity counter — resets on each keypress */

    RedLED_On();
    UART_PutString("Enter PIN followed by '#':\r\n");

    while (true)
    {
        /* ---- Inactivity timeout check ---- */
        if (elapsedMs >= PIN_TIMEOUT_MS)
        {
            UART_PutString("\r\nPIN entry timeout. Returning to sleep.\r\n");
            RedLED_Blink(3u, 100u, 100u);
            RedLED_Off();
            return false;
        }

        char key = Keypad_Debounce();   /* non-blocking poll */

        if (key == '\0')
        {
            /* No key yet — advance the inactivity timer and try again */
            CyDelay(KEY_POLL_INTERVAL);
            elapsedMs += KEY_POLL_INTERVAL;
            continue;
        }

        /* A key was confirmed — reset the inactivity timer */
        elapsedMs = 0u;

        if (key == '#')
        {
            buf[idx] = '\0';
            break;
        }
        else if (key == '*')
        {
            /* '*' during PIN entry acts as backspace */
            if (idx > 0u)
            {
                idx--;
                UART_PutString("\b \b");
            }
        }
        else if (idx < 15u)
        {
            buf[idx++] = key;
            UART_Put('*');
        }
    }

    UART_PutString("\r\n");

    if (strcmp(buf, SECRET_PIN) == 0)
    {
        RedLED_Off();
        UART_PutString("PIN correct.\r\n");
        return true;
    }

    RedLED_Blink(2u, 200u, 200u);
    RedLED_Off();
    UART_PutString("PIN incorrect.\r\n");
    return false;
}

/* ---------- Knock verification ---------- */
static bool Knock_Verify(void)
{
    float    prevMag      = 0.0f;
    uint32   loopTimeMs   = 0;
    uint32   lastKnockTime = 0;
    uint8    knockState   = STATE_LOCKED_WAIT_K1;

    GreenLED_On();
    UART_PutString("Listening for knocks...\r\n");

    for (;;)
    {
        int16 rawX = (int16)((ReadRegister(0x29) << 8) | ReadRegister(0x28));
        int16 rawY = (int16)((ReadRegister(0x2B) << 8) | ReadRegister(0x2A));
        int16 rawZ = (int16)((ReadRegister(0x2D) << 8) | ReadRegister(0x2C));

        float ax = (rawX >> 4) * 0.001f * 9.81f;
        float ay = (rawY >> 4) * 0.001f * 9.81f;
        float az = (rawZ >> 4) * 0.001f * 9.81f;
        float mag = sqrtf(ax*ax + ay*ay + az*az);

        if (mag > KNOCK_THRESHOLD &&
            prevMag <= KNOCK_THRESHOLD &&
            (loopTimeMs - lastKnockTime) > KNOCK_DEBOUNCE_MS)
        {
            uint32 dt = loopTimeMs - lastKnockTime;
            lastKnockTime = loopTimeMs;

            if (knockState == STATE_LOCKED_WAIT_K1)
            {
                printf("[Knock 1] Detected. Waiting for Knock 2 (~1 sec)...\r\n");
                knockState = STATE_LOCKED_WAIT_K2;
            }
            else if (knockState == STATE_LOCKED_WAIT_K2)
            {
                if (dt >= (TARGET_DELAY_MS - TOLERANCE_MS) &&
                    dt <= (TARGET_DELAY_MS + TOLERANCE_MS))
                {
                    printf("[Knock 2] Timing: %lu ms -> PATTERN MATCH!\r\n", dt);
                    GreenLED_Off();
                    return true;
                }
                else
                {
                    printf("[Knock 2] Timing: %lu ms -> FAILED. Resetting...\r\n", dt);
                    GreenLED_Blink(2u, 200u, 200u);
                    GreenLED_Off();
                    return false;
                }
            }
        }

        if (knockState == STATE_LOCKED_WAIT_K2 &&
            (loopTimeMs - lastKnockTime) > (TARGET_DELAY_MS + TOLERANCE_MS))
        {
            UART_PutString("Timeout waiting for Knock 2. Resetting...\r\n");
            GreenLED_Blink(2u, 200u, 200u);
            GreenLED_Off();
            return false;
        }

        if (knockState == STATE_LOCKED_WAIT_K1 && loopTimeMs > 10000)
        {
            UART_PutString("No knock detected. Resetting...\r\n");
            GreenLED_Blink(2u, 200u, 200u);
            GreenLED_Off();
            return false;
        }

        prevMag = mag;
        loopTimeMs += 100;
        CyDelay(100);
    }
}

/*===========================================================================
 * main
 *===========================================================================*/
int main(void)
{
    Cy_GPIO_Write(DEBUG_PIN_PORT, DEBUG_PIN_NUM, 0);
    Cy_GPIO_Write(EXT_GREEN_LED_PORT, EXT_GREEN_LED_NUM, 0);
    Cy_GPIO_Write(EXT_RED_LED_PORT,   EXT_RED_LED_NUM,   0);
    

    __enable_irq();

    UART_Start();
    setvbuf(stdin, NULL, _IONBF, 0);

    SensorBus_Start();
    register_setting.slaveAddress = 0x19;

    WriteRegister(0x20, 0x57);
    WriteRegister(0x23, 0x88);

    if (ReadRegister(0x0F) != 0x33)
    {
        UART_PutString("ERROR: LIS3DH not found! Check wiring.\r\n");
        for (;;) {}
    }
    UART_PutString("LIS3DH Detected!\r\n");

    PWM_Servo_Start();
    Servo_SetAngle(SERVO_LOCK_DEG);
    isUnlocked = false;

    Cy_SysInt_Init(&wakeIntCfg, Keypad_WakeISR);
    Button_Init();

    UART_PutString("=== KnockLock Ready ===\r\n");

    for (;;)
    {
        /*------------------------------------------------------------------
         * Button toggle — works in both locked and unlocked states.
         *------------------------------------------------------------------*/
        if (buttonToggleFlag)
        {
            buttonToggleFlag = false;
            if (isUnlocked)
            {
                UART_PutString("Button pressed — LOCKING.\r\n");
                Lock_Engage();
            }
            else
            {
                UART_PutString("Button pressed — UNLOCKING.\r\n");
                Lock_Disengage();
            }
            continue;
        }

        /*------------------------------------------------------------------
         * UNLOCKED idle loop
         * Stay awake, polling keypad for '*' (lock) and button flag.
         * The accelerometer is not needed here; skip I2C polling.
         *------------------------------------------------------------------*/
        if (isUnlocked)
        {
            char k = Keypad_Poll();
            if (k == '*')
            {
                UART_PutString("'*' pressed — locking.\r\n");
                Lock_Engage();
            }
            CyDelay(KEY_POLL_INTERVAL);
            continue;   /* stay in loop — no sleep while unlocked */
        }

        /*------------------------------------------------------------------
         * LOCKED — sleep until keypad or button wakes us.
         *------------------------------------------------------------------*/
        System_EnterDeepSleep();

        /* Re-check button first (may have been the wake source) */
        if (buttonToggleFlag)
        {
            buttonToggleFlag = false;
            UART_PutString("Button pressed — UNLOCKING.\r\n");
            Lock_Disengage();
            continue;
        }

        /*------------------------------------------------------------------
         * Stage 1: PIN
         * PIN_Verify() now returns false on a 20-second inactivity timeout,
         * causing the loop to continue and re-enter deep sleep.
         *------------------------------------------------------------------*/
        if (!PIN_Verify())
        {
            CyDelay(1000);
            continue;
        }

        /*------------------------------------------------------------------
         * Stage 2: Knock
         *------------------------------------------------------------------*/
        UART_PutString("PIN accepted. Enter knock pattern now.\r\n");
        if (!Knock_Verify())
        {
            UART_PutString("Knock pattern incorrect. Access denied.\r\n");
            CyDelay(2000);
            continue;
        }

        /*------------------------------------------------------------------
         * Stage 3: Unlock — stays open until button or '*'
         *------------------------------------------------------------------*/
        UART_PutString("Access granted! Unlocking...\r\n");
        Lock_Disengage();

        /* Execution falls through to the top of the loop where the
           unlocked idle branch will handle '*' and button events.         */
    }
}