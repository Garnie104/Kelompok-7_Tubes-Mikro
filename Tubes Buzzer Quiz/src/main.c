#include "stm32f4xx.h"

// KONFIGURASI ALAMAT LCD I2C
#define PCF8574_ADDRESS (0x27 << 1)

// VARIABEL GLOBAL
volatile uint8_t game_locked = 0;

// DEKLARASI FUNGSI
void System_Init(void);
void GPIO_Init(void);
void TIM2_PWM_Init(void);
void EXTI_Init(void);
void I2C1_Init(void);

// Helper
void Simple_Delay(volatile int count);

// Fungsi Driver LCD
void I2C_Write(uint8_t data);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_Init(void);
void LCD_String(char *str);
void LCD_Clear(void);

// Fungsi Buzzer
void Buzzer_Tone(uint16_t period);
void Buzzer_Stop(void);


int main(void) {
    System_Init(); 

    
    LCD_Clear();
    LCD_String("   BUZZER QUIZ  ");
    LCD_SendCommand(0xC0); 
    LCD_String("   SIAP MAIN!   ");

    while (1) {
        __WFI(); 
    }
}

// INTERRUPT HANDLERS

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) { // Cek Flag Line 0
        EXTI->PR = EXTI_PR_PR0;   // Clear Flag
        
        if (game_locked == 0) {
            Simple_Delay(20000); // Debounce
            game_locked = 1;
            
            GPIOB->ODR |= (1U << 0); // LED A ON
            
            LCD_Clear();
            LCD_String("PENEKAN:");
            LCD_SendCommand(0xC0);
            LCD_String("PLAYER A !!!");
            
            Buzzer_Tone(1000);       
            Simple_Delay(500000);
            Buzzer_Stop();
        }
    }
}

void EXTI1_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR = EXTI_PR_PR1;
        
        if (game_locked == 0) {
            Simple_Delay(20000);
            game_locked = 1;
            GPIOB->ODR |= (1U << 1); // LED B ON
            
            LCD_Clear();
            LCD_String("PENEKAN:");
            LCD_SendCommand(0xC0);
            LCD_String("PLAYER B !!!");
            
            Buzzer_Tone(2000);       
            Simple_Delay(500000);
            Buzzer_Stop();
        }
    }
}

void EXTI2_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR2) {
        EXTI->PR = EXTI_PR_PR2;
        
        Buzzer_Stop();
        GPIOB->ODR &= ~((1U << 0) | (1U << 1));
        Simple_Delay(50000);
        game_locked = 0;
        
        
        EXTI->PR = EXTI_PR_PR0 | EXTI_PR_PR1; 
        
        LCD_Clear();
        LCD_String("   BUZZER QUIZ  ");
        LCD_SendCommand(0xC0);
        LCD_String("   SIAP MAIN!   ");
    }
}



void System_Init(void) {
    GPIO_Init();
    TIM2_PWM_Init();
    I2C1_Init();
    EXTI_Init();
    
    Simple_Delay(500000); 
    LCD_Init(); 
    
    EXTI->PR = 0xFF;
}


void GPIO_Init(void) {
    
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    GPIOB->MODER &= ~((3U << (0 * 2)) | (3U << (1 * 2))); 
    
    GPIOB->MODER |= ((1U << (0 * 2)) | (1U << (1 * 2)));
    
    
    GPIOA->MODER &= ~((3U << (0 * 2)) | (3U << (1 * 2)) | (3U << (2 * 2)));
    
    GPIOA->PUPDR &= ~((3U << (0 * 2)) | (3U << (1 * 2)) | (3U << (2 * 2)));
    GPIOA->PUPDR |= ((1U << (0 * 2)) | (1U << (1 * 2)) | (1U << (2 * 2))); 
}


void TIM2_PWM_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Konfigurasi PB3 ke Alternate Function (AF)
    GPIOB->MODER &= ~(3U << (3 * 2)); // Clear
    GPIOB->MODER |= (2U << (3 * 2));  // Set AF (10)

    // Set AF Mapping (AF01 untuk TIM2) pada Pin 3
    // AFR[0] untuk pin 0-7, AFR[1] untuk pin 8-15
    GPIOB->AFR[0] &= ~(0xF << (3 * 4)); // Clear 4 bit posisi pin 3
    GPIOB->AFR[0] |= (1U << (3 * 4));   // Set AF01

    TIM2->PSC = 16 - 1; 
    TIM2->ARR = 1000 - 1; 
    
    // PWM Mode 1 (110) pada Channel 2
    TIM2->CCMR1 &= ~TIM_CCMR1_OC2M; // Clear bits
    TIM2->CCMR1 |= (6U << 12);      // Set PWM Mode 1
    TIM2->CCMR1 |= TIM_CCMR1_OC2PE; // Preload Enable

    TIM2->CCER |= TIM_CCER_CC2E;    // Enable Output
    TIM2->CR1 |= TIM_CR1_CEN;       // Enable Counter
}

void Buzzer_Tone(uint16_t period) {
    TIM2->ARR = period - 1;
    TIM2->CCR2 = period / 2; 
}

void Buzzer_Stop(void) {
    TIM2->CCR2 = 0;
}

// 3. I2C INIT
void I2C1_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // PB6, PB7 sebagai Alternate Function
    GPIOB->MODER &= ~((3U << 12) | (3U << 14)); // Clear
    GPIOB->MODER |= (2U << 12) | (2U << 14);    // Set AF

    GPIOB->OTYPER |= (1U << 6) | (1U << 7);     // Open Drain
    GPIOB->OSPEEDR |= (3U << 12) | (3U << 14);  // High Speed
    GPIOB->PUPDR |= (1U << 12) | (1U << 14);    // Pull Up

    // Set AF04 (I2C1) untuk PB6 & PB7
    GPIOB->AFR[0] &= ~((0xF << 24) | (0xF << 28)); // Clear
    GPIOB->AFR[0] |= (4U << 24) | (4U << 28);      // Set AF4

    I2C1->CR1 |= I2C_CR1_SWRST; // Reset I2C
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    I2C1->CR2 = 16; // 16 MHz
    I2C1->CCR = 80; // 100kHz Standard Mode
    I2C1->TRISE = 17; 
    I2C1->CR1 |= I2C_CR1_PE; 
}

void I2C_Write(uint8_t data) {
    volatile int timeout = 50000;
    
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) { if(--timeout<=0) return; }

    I2C1->DR = PCF8574_ADDRESS;
    timeout = 50000;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) { if(--timeout<=0) return; }
    (void)I2C1->SR2; 

    I2C1->DR = data;
    timeout = 50000;
    while (!(I2C1->SR1 & I2C_SR1_BTF)) { if(--timeout<=0) return; }

    I2C1->CR1 |= I2C_CR1_STOP;
}

void Simple_Delay(volatile int count) {
    while(count--);
}

void LCD_Send_I2C(uint8_t data, uint8_t flags) {
    uint8_t up = (data & 0xF0) | flags | 0x08; 
    uint8_t lo = ((data << 4) & 0xF0) | flags | 0x08;

    I2C_Write(up); Simple_Delay(100);
    I2C_Write(up | 0x04); Simple_Delay(500);      
    I2C_Write(up & ~0x04); Simple_Delay(500);     

    I2C_Write(lo); Simple_Delay(100);
    I2C_Write(lo | 0x04); Simple_Delay(500);     
    I2C_Write(lo & ~0x04); Simple_Delay(500);
}

void LCD_SendCommand(uint8_t cmd) { LCD_Send_I2C(cmd, 0); }
void LCD_SendData(uint8_t data) { LCD_Send_I2C(data, 1); }

void LCD_Init(void) {
    Simple_Delay(500000); 
    LCD_SendCommand(0x33); Simple_Delay(50000); 
    LCD_SendCommand(0x32); Simple_Delay(50000);
    LCD_SendCommand(0x28); 
    LCD_SendCommand(0x0C); 
    LCD_SendCommand(0x06); 
    LCD_SendCommand(0x01); Simple_Delay(100000);  
}

void LCD_Clear(void) {
    LCD_SendCommand(0x01);
    Simple_Delay(50000); 
}

void LCD_String(char *str) {
    while (*str) LCD_SendData(*str++);
}

// 4. EXTI INIT
void EXTI_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Clear mapping dulu
    SYSCFG->EXTICR[0] &= ~(0xFFF); 
    // Default 0000 sudah mapping ke PAx, jadi tidak perlu Set lagi jika pakai PA
    
    EXTI->IMR |= (EXTI_IMR_MR0 | EXTI_IMR_MR1 | EXTI_IMR_MR2);     // Unmask
    EXTI->FTSR |= (EXTI_FTSR_TR0 | EXTI_FTSR_TR1 | EXTI_FTSR_TR2); // Falling Edge

    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
}