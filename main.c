#include <stm32f10x.h>
#include <stdint.h>

void gpio_init(void);
void SystemCLK(void);
void spi_init(void);
void spi_transmit(uint8_t data);
uint16_t spi_receiveData(void);

void SystemCLK(void){
	RCC->CR |= 1<<16;
	while(!(RCC->CR & RCC_CR_HSERDY));
	
	//configure flash memory
	FLASH->ACR |= FLASH_ACR_PRFTBE;
	while(!(FLASH->ACR & FLASH_ACR_PRFTBS));
	
	FLASH->ACR |= FLASH_ACR_LATENCY_2; //latency for HCLK 64mHZ
	//Disabled PLL for configure
//	RCC->CR &= ~ RCC_CR_PLLON;
//	while(RCC->CR & RCC_CR_PLLRDY);
	// configure PLL
	RCC->CFGR |= RCC_CFGR_PLLXTPRE_HSE_Div2;
	//MULL
	RCC->CFGR |=RCC_CFGR_PLLMULL16; //4x16=64 mHZ
	
	RCC->CR |= 1<<24; // PLL ON
	while(!(RCC->CR &RCC_CR_PLLRDY));
	
	// Set PLL for HCLK
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL));
	
}

void gpio_init(void){
	RCC->APB2ENR |=RCC_APB2ENR_IOPAEN;
	GPIOA->CRL &= ~((GPIO_CRL_MODE4 | GPIO_CRL_CNF4) | 
                (GPIO_CRL_MODE5 | GPIO_CRL_CNF5) |
                (GPIO_CRL_MODE6 | GPIO_CRL_CNF6) |
                (GPIO_CRL_MODE7 | GPIO_CRL_CNF7));
	GPIOA->CRL |= (11<<16) | ( 11<<20) | (11<<28); //PA4->PA7 mode AF push-pull 
	GPIOA->CRL |= 4<<24 ; //port PA6 mode input floating
}

void spi_init(void){
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 &= ~(1<<6); // disable SPI1
	//Cau hinh che do du lieu
	SPI1->CR1 &= ~SPI_CR1_CPOL;  // Polarity: Low
  SPI1->CR1 &= ~SPI_CR1_CPHA;  // Phase: 1st edge
	
	SPI1->CR1 |=SPI_CR1_MSTR; //STM32 là master
	SPI1->CR1 |= 0x0028; // f/64= 1MHZ
	SPI1->CR1 &= ~SPI_CR1_DFF;   // 8-bit data frame format
  SPI1->CR1 &= ~SPI_CR1_LSBFIRST; // MSB first
	SPI1->CR1 |= 1<<6; //Enable SPI
}
void spi_transmit(uint8_t data){
	while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR =(uint16_t)data;
	while(SPI1->SR & SPI_SR_BSY);
}
uint16_t spi_receiveData(void){
	while(!(SPI1->SR & SPI_SR_RXNE));
	return SPI1->DR;
}
int main(void){
	SystemCLK();
	gpio_init();
	spi_init();
	uint8_t data = 0x2A;
	spi_transmit(data);
}
