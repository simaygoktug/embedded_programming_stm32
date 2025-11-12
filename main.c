/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Reaction Timer (FSM, TIM6 1ms, no delays)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* ===== BSP LCD (Lab 5) ===== */
#include "stm32412g_discovery.h"
#include "stm32412g_discovery_ts.h"
#include "stm32412g_discovery_lcd.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  ST_IDLE = 0,
  ST_CD,          /* countdown running */
  ST_CD_PAUSE,    /* countdown paused while any button pressed */
  ST_R_READY,     /* random LED k turns GREEN */
  ST_TIMING,      /* timing until correct button or timeout */
  ST_RESULT       /* freeze time; wait BTN1 for restart */
} fsm_state_t;

typedef enum { OFF=0, RED, GREEN, ORANGE } LEDcolour;
typedef enum { NOLED=-1, SLED1=0, SLED2, SLED3, SLED4, SLED5, SLED6, SLED7, SLED8 } LEDnumber;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Shield shift register & buttons */
#define LED_CLK_PORT   GPIOG
#define LED_CLK_PIN    GPIO_PIN_9
#define DATA_SEL_PORT  GPIOG
#define DATA_SEL_PIN   GPIO_PIN_14

/* Buttons (shared 4 lines via mux; PG14 selects group) */
#define BTN4_8_PORT    GPIOG
#define BTN4_8_PIN     GPIO_PIN_13
#define BTN2_6_PORT    GPIOG
#define BTN2_6_PIN     GPIO_PIN_12
#define BTN3_7_PORT    GPIOF
#define BTN3_7_PIN     GPIO_PIN_4
#define BTN1_5_PORT    GPIOF
#define BTN1_5_PIN     GPIO_PIN_10

/* TIM6 → 1ms tick (PSC=72-1, ARR=1000-1) */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;
SRAM_HandleTypeDef hsram1; /* LCD uses FSMC; left as generated */

/* USER CODE BEGIN PV */
/* Shift register image: 16-bit, LED i uses bits [2*i+1:2*i] (MSB first shift) */
static uint16_t led_sr = 0x0000;

/* FSM and timing */
static volatile uint32_t ms = 0;       /* global millisecond timebase */
static volatile uint8_t  tick1ms = 0;  /* main loop wake hint */

static fsm_state_t state = ST_IDLE;

static uint16_t blink1s_ctr = 0;
static uint16_t cd_500ms_ctr = 0;
static int8_t   cd_index = 7;          /* SLED8..SLED1 -> 7..0 */
static int8_t   target_led = -1;

static uint16_t debounce_cnt[8] = {0};
static uint8_t  btn_stable[8]   = {0};
static uint8_t  btn_prev[8]     = {0};

static uint32_t t_ms = 0;              /* reaction time (ms) */
static uint8_t  lcd_dirty = 1;         /* force initial paint */

/* LCG rng */
static uint32_t rng = 0x12345678u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* ---- low-level LED / Button helpers ---- */
static inline void LED_ShiftOut16(uint16_t pattern);
static void SetLEDcolor(LEDnumber led, LEDcolour c);
static void SetAll(LEDcolour c);
static void SetNone(void);
static uint8_t Buttons_AnyPressed(void);
static uint8_t Button_ReadNumber(void); /* 1..8 if one pressed, 0 otherwise */

/* ---- LCD helpers ---- */
static void LCD_InitAndClear(void);
static void LCD_ShowTime_ms(uint32_t ms_val);

/* ---- FSM helpers ---- */
static void RNG_Scramble(void);
static uint8_t RNG_Rand1to8(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Shift 16 bits MSB-first into SN74LV164 via PG14 (DATA_SEL) & PG9 (CLK) */
static inline void LED_ShiftOut16(uint16_t pattern)
{
  for (int i = 15; i >= 0; --i) {
    /* DATA on PG14: 0 -> RESET, 1 -> SET (biti terslemeden yazıyoruz) */
    HAL_GPIO_WritePin(DATA_SEL_PORT, DATA_SEL_PIN,
                      (pattern & (1u << i)) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    /* CLK pulse */
    HAL_GPIO_WritePin(LED_CLK_PORT, LED_CLK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_CLK_PORT, LED_CLK_PIN, GPIO_PIN_SET);
  }
}

/* Update single LED colour in shadow reg, then push */
static void SetLEDcolor(LEDnumber led, LEDcolour c)
{
  if (led < SLED1 || led > SLED8) return;
  uint16_t mask = (uint16_t)(0x3u << (led*2));
  led_sr &= ~mask;
  led_sr |= (uint16_t)((uint16_t)c << (led*2));
  LED_ShiftOut16(led_sr);
}

static void SetAll(LEDcolour c)
{
  led_sr = 0;
  for (int i=0;i<8;i++) led_sr |= (uint16_t)((uint16_t)c << (i*2));
  LED_ShiftOut16(led_sr);
}
static void SetNone(void)
{
  led_sr = 0;
  LED_ShiftOut16(led_sr);
}

/* Read buttons (debounced in 1ms ISR)
   Returns mask of currently stable-pressed buttons (bit0=BTN1 ... bit7=BTN8) */
static inline uint8_t Buttons_StableMask(void)
{
  uint8_t m=0;
  for(int i=0;i<8;i++) if (btn_stable[i]) m |= (1u<<i);
  return m;
}

/* If exactly one stable button is pressed, return its number (1..8); else 0 */
static uint8_t Button_ReadNumber(void)
{
  uint8_t m = Buttons_StableMask();
  if (m && ((m & (m-1))==0)) { /* power of two -> single press */
    for (uint8_t i=0;i<8;i++) if (m & (1u<<i)) return (uint8_t)(i+1);
  }
  return 0;
}

/* Non-debounced raw sample for the two groups (PG14=1 : BTN1..4, PG14=0 : BTN5..8) */
static inline uint8_t Buttons_RawSample(void)
{
  uint8_t m = 0;

  /* Group 1: BTN1..4 */
  HAL_GPIO_WritePin(DATA_SEL_PORT, DATA_SEL_PIN, GPIO_PIN_SET);
  if (HAL_GPIO_ReadPin(BTN1_5_PORT, BTN1_5_PIN)==GPIO_PIN_RESET) m |= (1u<<(1-1));
  if (HAL_GPIO_ReadPin(BTN2_6_PORT, BTN2_6_PIN)==GPIO_PIN_RESET) m |= (1u<<(2-1));
  if (HAL_GPIO_ReadPin(BTN3_7_PORT, BTN3_7_PIN)==GPIO_PIN_RESET) m |= (1u<<(3-1));
  if (HAL_GPIO_ReadPin(BTN4_8_PORT, BTN4_8_PIN)==GPIO_PIN_RESET) m |= (1u<<(4-1));

  /* Group 2: BTN5..8 */
  HAL_GPIO_WritePin(DATA_SEL_PORT, DATA_SEL_PIN, GPIO_PIN_RESET);
  if (HAL_GPIO_ReadPin(BTN1_5_PORT, BTN1_5_PIN)==GPIO_PIN_RESET) m |= (1u<<(5-1));
  if (HAL_GPIO_ReadPin(BTN2_6_PORT, BTN2_6_PIN)==GPIO_PIN_RESET) m |= (1u<<(6-1));
  if (HAL_GPIO_ReadPin(BTN3_7_PORT, BTN3_7_PIN)==GPIO_PIN_RESET) m |= (1u<<(7-1));
  if (HAL_GPIO_ReadPin(BTN4_8_PORT, BTN4_8_PIN)==GPIO_PIN_RESET) m |= (1u<<(8-1));

  return m;
}

static uint8_t Buttons_AnyPressed(void)
{
  return Buttons_StableMask()!=0;
}

/* LCD */
static void LCD_InitAndClear(void)
{
  BSP_LCD_Init();
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t*)"CE865 Reaction Timer", CENTER_MODE);
  BSP_LCD_DrawHLine(0, 36, 240);
  BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
  BSP_LCD_DisplayStringAt(0, 50, (uint8_t*)"Press BTN1 to start", CENTER_MODE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_DisplayStringAt(0, 100, (uint8_t*)"0.000", CENTER_MODE);
}

static void LCD_ShowTime_ms(uint32_t ms_val)
{
  char buf[16];
  if (ms_val > 9999) ms_val = 9999;
  /* xxx.yyy format (0.000 - 9.999) */
  uint32_t s = ms_val / 1000U;
  uint32_t ms3 = ms_val % 1000U;
  snprintf(buf, sizeof(buf), "%lu.%03lu", (unsigned long)s, (unsigned long)ms3);
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_FillRect(60, 90, 120, 40); /* clear area (white) */
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_DisplayStringAt(0, 100, (uint8_t*)buf, CENTER_MODE);
}

/* RNG (LCG) */
static void RNG_Scramble(void)
{
  rng ^= (uint32_t)ms;
  rng = rng * 1664525u + 1013904223u + (uint32_t)HAL_GetTick();
}
static uint8_t RNG_Rand1to8(void)
{
  rng = rng * 1103515245u + 12345u;
  return (uint8_t)(1u + ((rng >> 16) % 8u));
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_FSMC_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */
  LCD_InitAndClear();

  /* LEDs initial: all off, but IDLE blink uses SLED1 at 1Hz */
  SetNone();

  /* Start 1ms base timer */
  HAL_TIM_Base_Start_IT(&htim6);

  /* Seed RNG a bit */
  RNG_Scramble();

  state = ST_IDLE;
  lcd_dirty = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  for (;;)
  {
    /* Cooperative main loop: wake every 1ms tick */
    if (!tick1ms) continue;
    tick1ms = 0;

    /* Refresh LCD when marked dirty */
    if (lcd_dirty) {
      LCD_ShowTime_ms(t_ms);
      lcd_dirty = 0;
    }

    switch (state)
    {
      case ST_IDLE:
        /* 1 Hz blink on SLED1 */
        if (++blink1s_ctr >= 1000) { /* 1000 ms */
          blink1s_ctr = 0;
          /* toggle SLED1 only */
          LEDcolour c = ( (led_sr & (0x3u<<(SLED1*2))) ? OFF : RED );
          /* show idle blink in RED */
          SetLEDcolor(SLED1, c);
        }
        /* Start when BTN1 pressed */
        if (Button_ReadNumber() == 1) {
          /* prep countdown */
          SetAll(RED);
          cd_index = 7;            /* start from SLED8 */
          cd_500ms_ctr = 0;
          blink1s_ctr = 0;
          t_ms = 0;
          lcd_dirty = 1;
          state = ST_CD;
        }
        break;

      case ST_CD:
        /* suspend countdown if any button pressed */
        if (Buttons_AnyPressed()) { state = ST_CD_PAUSE; break; }

        if (++cd_500ms_ctr >= 500) {
          cd_500ms_ctr = 0;
          /* turn off current LED (right->left) */
          if (cd_index >= 0) {
            SetLEDcolor((LEDnumber)cd_index, OFF);
            cd_index--;
          }
          if (cd_index < 0) {
            /* last LED off -> choose random k and go R_READY */
            target_led = (int8_t)(RNG_Rand1to8() - 1);
            SetNone();
            SetLEDcolor((LEDnumber)target_led, GREEN);
            /* start timing next state */
            t_ms = 0;
            lcd_dirty = 1;
            state = ST_R_READY;
          }
        }
        break;

      case ST_CD_PAUSE:
        /* stay paused until all buttons released */
        if (!Buttons_AnyPressed()) state = ST_CD;
        break;

      case ST_R_READY:
        /* Immediately transition to TIMING; LED k already green */
        state = ST_TIMING;
        break;

      case ST_TIMING:
        /* Time runs in 1ms ISR; we just watch for events */
        lcd_dirty = 1;

        /* Correct button? */
        if (Button_ReadNumber() == (uint8_t)(target_led+1)) {
          state = ST_RESULT;
          /* keep green, freeze time */
          blink1s_ctr = 0;
        } else if (t_ms >= 9999) {
          /* timeout */
          state = ST_RESULT;
          blink1s_ctr = 0;
        }
        /* wrong buttons are ignored */
        break;

      case ST_RESULT:
        /* blink SLED1 each 1s as “ready” sign */
        if (++blink1s_ctr >= 1000) {
          blink1s_ctr = 0;
          LEDcolour c = ( (led_sr & (0x3u<<(SLED1*2))) ? OFF : RED );
          SetLEDcolor(SLED1, c);
        }
        /* BTN1 to restart from countdown */
        if (Button_ReadNumber() == 1) {
          SetAll(RED);
          cd_index = 7;
          cd_500ms_ctr = 0;
          t_ms = 0;
          lcd_dirty = 1;
          state = ST_CD;
        }
        break;

      default: state = ST_IDLE; break;
    }
  }
  /* USER CODE END WHILE */
}

/* ===================== IRQ & Timebase ===================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    ms++;
    tick1ms = 1;

    /* ---- Debounce buttons (20 ms threshold) ---- */
    uint8_t raw = Buttons_RawSample(); /* active-low mapped to ‘1’ bits */
    for (uint8_t i=0;i<8;i++) {
      uint8_t bit = (raw >> i) & 1u;
      if (bit != btn_prev[i]) {
        btn_prev[i] = bit;
        debounce_cnt[i] = 0;
      } else if (debounce_cnt[i] < 20) {
        debounce_cnt[i]++;
        if (debounce_cnt[i] == 20) {
          btn_stable[i] = bit; /* 0:not pressed, 1:pressed */
        }
      }
    }

    /* ---- Time counting in TIMING state ---- */
    if (state == ST_TIMING) {
      if (t_ms < 10000) t_ms++;
    }
  }
}

/* ===================== Boilerplate ===================== */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/* TIM6 1ms timebase */
static void MX_TIM6_Init(void)
{
  __HAL_RCC_TIM6_CLK_ENABLE();

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 72-1;   /* 72MHz/72 = 1MHz */
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;    /* 1MHz/1000 = 1kHz -> 1ms */
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) { Error_Handler(); }

  HAL_NVIC_SetPriority(TIM6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_IRQn);
}

//void TIM6_IRQHandler(void)
//{
//  HAL_TIM_IRQHandler(&htim6);
//}

/* GPIO: PG9, PG14 outputs; PF10, PG12, PF4, PG13 inputs */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  GPIO_InitTypeDef gi = {0};

  /* Outputs: PG9 (CLK), PG14 (DATA/SEL) */
  gi.Pin = GPIO_PIN_9 | GPIO_PIN_14;
  gi.Mode = GPIO_MODE_OUTPUT_PP;
  gi.Pull = GPIO_NOPULL;
  gi.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &gi);

  /* Inputs: PF10, PG12, PF4, PG13 */
  gi.Pin = GPIO_PIN_10;
  gi.Mode = GPIO_MODE_INPUT;
  gi.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &gi);

  gi.Pin = GPIO_PIN_12 | GPIO_PIN_13;
  gi.Mode = GPIO_MODE_INPUT;
  gi.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &gi);

  gi.Pin = GPIO_PIN_4;
  gi.Mode = GPIO_MODE_INPUT;
  gi.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &gi);
}

/* FSMC for LCD (as generated by board project) */
static void MX_FSMC_Init(void)
{
  /* keep your auto-generated FSMC init here if Cube added it
     (left empty if already present in project templates) */
}

/* Error handler */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
