// display_init.cpp
// Hardware initialisation for the STM32F429I-DISCO onboard 2.4" ILI9341 display.
#define PW_LOG_MODULE_NAME "DISP"
#include "pw_log/log.h"
//
// The ILI9341 IM[3:0] pins are hardwired for RGB-DE interface mode.  Pixel data
// streams from the LTDC peripheral; configuration registers are accessed via SPI5
// (shared with the L3GD20, different CS: PC2 for the LCD vs PC1 for the gyro).
//
// Sequence:
//   1. FMC / SDRAM   – provides the 150 KB framebuffer at 0xD000_0000
//   2. PLLSAI        – generates the ~6 MHz LTDC pixel clock
//   3. LTDC GPIOs    – 18 colour pins + sync/clock pins on AF14
//   4. LTDC          – timing, layer, framebuffer address
//   5. ILI9341       – wake from sleep, set RGB565, display on

#include "display_init.hpp"
#include <modm/board.hpp>          // Board::SystemClock, gpio typedefs, modm::delay
#include <modm/platform.hpp>       // SpiMaster5, Gpio*

using namespace modm::platform;
using namespace Board;

static constexpr uint32_t LCD_W  = 240;
static constexpr uint32_t LCD_H  = 320;
// FB_STRIDE (512 bytes/row) is declared in display_init.hpp (included above).

// ---------------------------------------------------------------------------
// Framebuffer – 240 × 320 × 2 bytes = 153 600 bytes
// TEST MODE: placed in internal SRAM to rule out SDRAM timing as comb cause.
// 150 KB out of 192 KB AHB SRAM (SRAM1+2+3); LVGL heap may need reducing if
// the linker overflows.  Revert to the SDRAM version once root cause is found.
// ---------------------------------------------------------------------------
static uint16_t FramebufferStorage[(FB_STRIDE / 2) * LCD_H] __attribute__((aligned(32)));
uint16_t* const Framebuffer = FramebufferStorage;

// ---------------------------------------------------------------------------
// 1.  SDRAM (IS42S16400J-6BLI, 64 Mbit, 16-bit bus, bank 2 of FMC)
// ---------------------------------------------------------------------------
// All FMC GPIO pins are on AF12.
// The board.hpp fmc:: namespace names every pin.

static void initSdram()
{
    // Enable FMC clock
    RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;
    __DSB();

    // --- GPIO setup: AF12 on every FMC pin ---
    // Helper lambda: configure one GPIO pin for AF12 (FMC), very-high speed, no pull
    auto fmcPin = [](GPIO_TypeDef* port, uint32_t pin) {
        uint32_t pos = pin * 2;
        // Mode = alternate function (10)
        port->MODER  = (port->MODER  & ~(3u << pos)) | (2u << pos);
        // Speed = very high (11)
        port->OSPEEDR = (port->OSPEEDR & ~(3u << pos)) | (3u << pos);
        // No pull
        port->PUPDR  &= ~(3u << pos);
        // AF12
        if (pin < 8) {
            port->AFR[0] = (port->AFR[0] & ~(0xFu << (pin * 4))) | (12u << (pin * 4));
        } else {
            port->AFR[1] = (port->AFR[1] & ~(0xFu << ((pin - 8) * 4))) | (12u << ((pin - 8) * 4));
        }
    };

    // Enable GPIO clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
                    RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN |
                    RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN;
    __DSB();

    // PB5=SDCKE1, PB6=SDNE1
    fmcPin(GPIOB, 5); fmcPin(GPIOB, 6);
    // PC0=SDNWE
    fmcPin(GPIOC, 0);
    // PD0=D2, PD1=D3, PD8=D13, PD9=D14, PD10=D15, PD14=D0, PD15=D1
    fmcPin(GPIOD, 0); fmcPin(GPIOD, 1); fmcPin(GPIOD, 8);
    fmcPin(GPIOD, 9); fmcPin(GPIOD, 10); fmcPin(GPIOD, 14); fmcPin(GPIOD, 15);
    // PE0=NBL0, PE1=NBL1, PE7–PE15=D4–D12
    fmcPin(GPIOE, 0); fmcPin(GPIOE, 1);
    for (uint32_t p = 7; p <= 15; ++p) fmcPin(GPIOE, p);
    // PF0–PF5=A0–A5, PF11=NRAS, PF12–PF15=A6–A9
    for (uint32_t p = 0; p <= 5; ++p) fmcPin(GPIOF, p);
    fmcPin(GPIOF, 11);
    for (uint32_t p = 12; p <= 15; ++p) fmcPin(GPIOF, p);
    // PG0=A10, PG1=A11, PG4=BA0, PG5=BA1, PG8=SDCLK, PG15=NCAS
    fmcPin(GPIOG, 0); fmcPin(GPIOG, 1); fmcPin(GPIOG, 4);
    fmcPin(GPIOG, 5); fmcPin(GPIOG, 8); fmcPin(GPIOG, 15);

    // --- FMC SDRAM Control Registers ---
    // RM0090 §37.7.3: SDCLK, RBURST and RPIPE must always be written to SDCR[0]
    // even when targeting bank 2; the hardware ignores those bits in SDCR[1].
    FMC_Bank5_6->SDCR[0] =
        FMC_SDCR1_SDCLK_1       |   // SDCLK = HCLK/2 = 84 MHz
        FMC_SDCR1_RBURST        |   // burst read enable
        FMC_SDCR1_RPIPE_0;          // read pipe = 2 cycle

    // Bank 2 specific geometry / CAS (SDCLK/RBURST/RPIPE ignored here per RM0090)
    FMC_Bank5_6->SDCR[1] =
        FMC_SDCR1_NC_0          |   // 8-bit column (01)
        FMC_SDCR1_NR_0          |   // 12-bit row (01)
        FMC_SDCR1_MWID_0        |   // 16-bit bus (01)
        FMC_SDCR1_NB            |   // 4 banks
        FMC_SDCR1_CAS_0         |   // CAS latency = 3 (11) – bit 0 of 2-bit field
        FMC_SDCR1_CAS_1;            // CAS latency = 3 (11) – bit 1

    // --- FMC SDRAM Timing Registers ---
    // RM0090 §37.7.3: TRC and TRP must always be written to SDTR[0] even for bank 2.
    // IS42S16400J at HCLK/2 = 84 MHz:
    // TMRD=2, TXSR=7, TRAS=4, TRC=7, TWR=2, TRP=2, TRCD=2  (in clock cycles-1)
    FMC_Bank5_6->SDTR[0] =
        (7u - 1) << FMC_SDTR1_TRC_Pos   |   // row cycle: 7 clk (ignored in SDTR[1])
        (2u - 1) << FMC_SDTR1_TRP_Pos;      // row precharge: 2 clk (ignored in SDTR[1])

    FMC_Bank5_6->SDTR[1] =
        (2u - 1) << FMC_SDTR1_TMRD_Pos  |   // mode register to active: 2 clk
        (7u - 1) << FMC_SDTR1_TXSR_Pos  |   // exit self-refresh: 7 clk
        (5u - 1) << FMC_SDTR1_TRAS_Pos  |   // self-refresh: 5 clk
        (2u - 1) << FMC_SDTR1_TWR_Pos   |   // write recovery: 2 clk
        (2u - 1) << FMC_SDTR1_TRCD_Pos;     // row to column: 2 clk

    // Helper: wait for FMC not busy
    auto waitFmc = [] {
        while (FMC_Bank5_6->SDSR & FMC_SDSR_BUSY) {}
    };

    // SDRAM command: clock enable
    FMC_Bank5_6->SDCMR =  (1u << FMC_SDCMR_CTB2_Pos) |   // target bank 2
                          (1u << FMC_SDCMR_MODE_Pos);     // CLK enable command
    waitFmc();
    modm::delay(1ms);

    // SDRAM command: precharge all
    FMC_Bank5_6->SDCMR =  (1u << FMC_SDCMR_CTB2_Pos) |
                          (2u << FMC_SDCMR_MODE_Pos);
    waitFmc();

    // SDRAM command: 8× auto-refresh
    FMC_Bank5_6->SDCMR = (1u << FMC_SDCMR_CTB2_Pos)  |
                          (3u << FMC_SDCMR_MODE_Pos)   |
                          ((8u - 1) << FMC_SDCMR_NRFS_Pos);
    waitFmc();

    // SDRAM command: load mode register
    // CAS=3, burst length=4, burst type=sequential, write burst=single.
    // MRD = 0x0232: bit[9]=1 (single write burst), bits[6:4]=011 (CAS=3), bits[2:0]=010 (BL=4).
    // BL=4 aligns with the LTDC DMA AHB burst size (INCR4, 4×32-bit = 8×16-bit words) so the
    // FMC needs fewer SDRAM READ commands and CAS overheads per AHB burst (was BL=1 = 1 READ
    // per word = 8× the overhead).
    FMC_Bank5_6->SDCMR =  (1u << FMC_SDCMR_CTB2_Pos)  |
                          (4u << FMC_SDCMR_MODE_Pos)   |
                          (0x0232u << FMC_SDCMR_MRD_Pos);
    waitFmc();
    modm::delay(1ms);
    // Refresh rate: (64 ms / 4096 rows) * 90 MHz - 20 = 1386  (conservative: 1292)
    FMC_Bank5_6->SDRTR = 1295u << FMC_SDRTR_COUNT_Pos;

    // Clear framebuffer to black
    for (uint32_t i = 0; i < (FB_STRIDE / 2) * LCD_H; ++i) Framebuffer[i] = 0x0000u;
}

// ---------------------------------------------------------------------------
// 2.  PLLSAI – pixel clock for LTDC ≈ 6 MHz
//     PLLM = 4 (from SystemClock), PLLSAIN = 192, PLLSAIR = 4,
//     DCKCFGR.PLLSAIDIVR = /16  →  (2 MHz × 192 / 4) / 16 = 6 MHz
// ---------------------------------------------------------------------------
static void initPllSai()
{
    // Disable PLLSAI before configuring
    RCC->CR &= ~RCC_CR_PLLSAION;
    while (RCC->CR & RCC_CR_PLLSAIRDY) {}

    // PLLSAIN=192, PLLSAIQ=7 (keep SAI clock), PLLSAIR=4
    RCC->PLLSAICFGR =
        (192u << RCC_PLLSAICFGR_PLLSAIN_Pos) |
        (7u   << RCC_PLLSAICFGR_PLLSAIQ_Pos) |
        (4u   << RCC_PLLSAICFGR_PLLSAIR_Pos);

    // PLLSAIDIVR = /16 (bits [17:16] = 11)
    RCC->DCKCFGR = (RCC->DCKCFGR & ~RCC_DCKCFGR_PLLSAIDIVR) |
                   RCC_DCKCFGR_PLLSAIDIVR;   // /16

    // Enable PLLSAI and wait for lock
    RCC->CR |= RCC_CR_PLLSAION;
    while (!(RCC->CR & RCC_CR_PLLSAIRDY)) {}
}

// ---------------------------------------------------------------------------
// 3.  LTDC GPIO pins – AF14, very-high speed
//     Sync/clock: PC6=HSYNC, PA4=VSYNC, PF10=DE, PG7=CLK
//     Red:   PC10=R2, PB0=R3, PA11=R4, PA12=R5, PB1=R6, PG6=R7
//     Green: PA6=G2,  PG10=G3, PB10=G4, PB11=G5, PC7=G6, PD3=G7
//     Blue:  PD6=B2,  PG11=B3, PG12=B4, PA3=B5,  PB8=B6, PB9=B7
// ---------------------------------------------------------------------------
static void initLtdcGpio()
{
    // Ensure all required GPIO clocks are on (most already on from SDRAM init)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN |
                    RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN;
    __DSB();

    auto ltdcPin = [](GPIO_TypeDef* port, uint32_t pin) {
        uint32_t pos = pin * 2;
        port->MODER   = (port->MODER   & ~(3u << pos)) | (2u << pos);   // AF
        port->OSPEEDR = (port->OSPEEDR & ~(3u << pos)) | (3u << pos);   // very high
        port->PUPDR  &= ~(3u << pos);                                    // no pull
        if (pin < 8)
            port->AFR[0] = (port->AFR[0] & ~(0xFu << (pin       * 4))) | (14u << (pin       * 4));
        else
            port->AFR[1] = (port->AFR[1] & ~(0xFu << ((pin - 8) * 4))) | (14u << ((pin - 8) * 4));
    };

    // Sync and clock
    ltdcPin(GPIOC, 6);   // HSYNC
    ltdcPin(GPIOA, 4);   // VSYNC
    ltdcPin(GPIOF, 10);  // DE
    ltdcPin(GPIOG, 7);   // CLK

    // Red channel
    ltdcPin(GPIOC, 10);  // R2
    ltdcPin(GPIOB, 0);   // R3
    ltdcPin(GPIOA, 11);  // R4
    ltdcPin(GPIOA, 12);  // R5
    ltdcPin(GPIOB, 1);   // R6
    ltdcPin(GPIOG, 6);   // R7

    // Green channel
    ltdcPin(GPIOA, 6);   // G2
    ltdcPin(GPIOG, 10);  // G3
    ltdcPin(GPIOB, 10);  // G4
    ltdcPin(GPIOB, 11);  // G5
    ltdcPin(GPIOC, 7);   // G6
    ltdcPin(GPIOD, 3);   // G7

    // Blue channel
    ltdcPin(GPIOD, 6);   // B2
    ltdcPin(GPIOG, 11);  // B3
    ltdcPin(GPIOG, 12);  // B4
    ltdcPin(GPIOA, 3);   // B5
    ltdcPin(GPIOB, 8);   // B6
    ltdcPin(GPIOB, 9);   // B7
}

// ---------------------------------------------------------------------------
// 4.  LTDC peripheral
//     ILI9341 RGB timing at ~6 MHz dot clock (from datasheet):
//       HSYNC=10, HBP=20, active=240, HFP=10
//       VSYNC=2,  VBP=2,  active=320, VFP=4
// ---------------------------------------------------------------------------
static void initLtdc()
{
    RCC->APB2ENR |= RCC_APB2ENR_LTDCEN;
    __DSB();

    // Synchronisation / porch registers (values are end-of-region, 0-based)
    constexpr uint32_t HSync = 10u, HBP = 20u, HActive = 240u, HFP = 10u;
    constexpr uint32_t VSync =  2u, VBP =  2u, VActive = 320u, VFP =  4u;

    LTDC->SSCR = ((HSync - 1u) << LTDC_SSCR_HSW_Pos) |
                 ((VSync - 1u) << LTDC_SSCR_VSH_Pos);

    LTDC->BPCR = ((HSync + HBP - 1u) << LTDC_BPCR_AHBP_Pos) |
                 ((VSync + VBP - 1u) << LTDC_BPCR_AVBP_Pos);

    LTDC->AWCR = ((HSync + HBP + HActive - 1u) << LTDC_AWCR_AAW_Pos) |
                 ((VSync + VBP + VActive - 1u) << LTDC_AWCR_AAH_Pos);

    LTDC->TWCR = ((HSync + HBP + HActive + HFP - 1u) << LTDC_TWCR_TOTALW_Pos) |
                 ((VSync + VBP + VActive + VFP - 1u) << LTDC_TWCR_TOTALH_Pos);

    // Background colour: black
    LTDC->BCCR = 0x00000000u;

    // --- Layer 1 ---
    LTDC_Layer1->WHPCR = ((HSync + HBP)                  << LTDC_LxWHPCR_WHSTPOS_Pos) |
                         ((HSync + HBP + HActive - 1u)   << LTDC_LxWHPCR_WHSPPOS_Pos);

    LTDC_Layer1->WVPCR = ((VSync + VBP)                  << LTDC_LxWVPCR_WVSTPOS_Pos) |
                         ((VSync + VBP + VActive - 1u)   << LTDC_LxWVPCR_WVSPPOS_Pos);

    // Pixel format: RGB565 (2)
    LTDC_Layer1->PFCR = 0x2u;

    // Default colour (ARGB8888, used outside active layer area)
    LTDC_Layer1->DCCR = 0xFF000000u;

    // Constant alpha = 255 (fully opaque), blending: pixel alpha × constant alpha / 255
    LTDC_Layer1->CACR = 0xFFu;
    LTDC_Layer1->BFCR = (4u << LTDC_LxBFCR_BF1_Pos) | (5u << LTDC_LxBFCR_BF2_Pos);

    // Framebuffer address and pitch.
    // CFBLL = active line length in bytes + 3 (LTDC requirement).
    // CFBP  = total row stride in bytes (must equal FB_STRIDE so the LTDC
    //         DMA address advances by exactly one padded row per scanline).
    LTDC_Layer1->CFBAR = reinterpret_cast<uint32_t>(Framebuffer);
    LTDC_Layer1->CFBLR = ((LCD_W * 2u + 3u) << LTDC_LxCFBLR_CFBLL_Pos) |
                         (FB_STRIDE          << LTDC_LxCFBLR_CFBP_Pos);

    LTDC_Layer1->CFBLNR = LCD_H;

    // Enable layer
    LTDC_Layer1->CR |= LTDC_LxCR_LEN;

    // Reload shadow registers immediately and enable LTDC
    LTDC->SRCR = LTDC_SRCR_IMR;
    // HSPOL/VSPOL/DEPOL all active-low (0).
    // PCPOL=0: LTDC drives data on rising edge; ILI9341 0xB0 DPL=1 samples on
    // falling edge → data is stable well before the sample point.  Correct.
    LTDC->GCR &= ~(LTDC_GCR_HSPOL | LTDC_GCR_VSPOL | LTDC_GCR_DEPOL | LTDC_GCR_PCPOL);
    LTDC->GCR  |= LTDC_GCR_LTDCEN;
}

// ---------------------------------------------------------------------------
// 5.  ILI9341 configuration via SPI5
//     The IM pins are hardwired for RGB interface; SPI is for register writes only.
//     CS=PC2, DC/WRX=PD13  (lcd::Csx, lcd::WrxDcx in board.hpp)
// ---------------------------------------------------------------------------

// Simple blocking SPI5 transfer (no DMA needed for a one-time init sequence)
static void spi5Transfer(uint8_t byte)
{
    while (!(SPI5->SR & SPI_SR_TXE)) {}
    *reinterpret_cast<volatile uint8_t*>(&SPI5->DR) = byte;
    while (!(SPI5->SR & SPI_SR_RXNE)) {}
    (void)*reinterpret_cast<volatile uint8_t*>(&SPI5->DR);
    while (SPI5->SR & SPI_SR_BSY) {}
}

static void ili9341Command(uint8_t cmd)
{
    lcd::WrxDcx::reset();   // DC low = command
    lcd::Csx::reset();      // CS low = select
    spi5Transfer(cmd);
    lcd::Csx::set();        // CS high = deselect
}

static void ili9341Data(uint8_t data)
{
    lcd::WrxDcx::set();     // DC high = data
    lcd::Csx::reset();
    spi5Transfer(data);
    lcd::Csx::set();
}

static void initIli9341()
{
    // Configure CS and DC pins as GPIO outputs
    lcd::Csx::setOutput(modm::Gpio::High);
    lcd::WrxDcx::setOutput(modm::Gpio::High);

    // SPI5: 8-bit, Mode 0, ~5 MHz  (APB2=84 MHz, BR=010 → ~10.5 MHz is fine)
    SpiMaster5::connect<GpioF7::Sck, GpioF8::Miso, GpioF9::Mosi>();
    SpiMaster5::initialize<SystemClock, 5_MHz>();

    modm::delay(5ms);

    // --- Power and timing initialisation (matches ST DISCO BSP ili9341.c) ---
    ili9341Command(0xCA);                       // undocumented power-up sequence
    ili9341Data(0xC3); ili9341Data(0x08); ili9341Data(0x50);

    ili9341Command(0xCF);                       // Power control B
    ili9341Data(0x00); ili9341Data(0xC1); ili9341Data(0x30);

    ili9341Command(0xED);                       // Power on sequence control
    ili9341Data(0x64); ili9341Data(0x03); ili9341Data(0x12); ili9341Data(0x81);

    ili9341Command(0xE8);                       // Driver timing control A
    ili9341Data(0x85); ili9341Data(0x00); ili9341Data(0x78);

    ili9341Command(0xCB);                       // Power control A
    ili9341Data(0x39); ili9341Data(0x2C); ili9341Data(0x00);
    ili9341Data(0x34); ili9341Data(0x02);

    ili9341Command(0xF7);                       // Pump ratio control
    ili9341Data(0x20);

    ili9341Command(0xEA);                       // Driver timing control B
    ili9341Data(0x00); ili9341Data(0x00);

    // --- Core display configuration ---
    ili9341Command(0xB1);                       // Frame rate (79 Hz)
    ili9341Data(0x00); ili9341Data(0x1B);

    ili9341Command(0xB6);                       // Display function control
    // P1: SM is bit 3 per ILI9341 datasheet (0x08 = SM=1 = alternating gate scan = combs!)
    // Use 0x00: SM=0 (normal sequential), SS=0, PTG=00, ISC=00
    ili9341Data(0x00);   // SM=0 (bit 3 clear): normal sequential gate scan
    ili9341Data(0xA7);   // GS=1 (bit 7), NL=39 → (39+1)×8 = 320 gate lines
    ili9341Data(0x27);
    ili9341Data(0x04);

    // Second 0xB6 write: gate/line config for RGB DE-mode (overrides first write)
    ili9341Command(0xB6);
    ili9341Data(0x00);   // SM=0 (bit 3 clear)
    ili9341Data(0xA7);   // GS=1, NL=39 (320 lines)

    ili9341Command(0xC0);                       // Power control 1
    ili9341Data(0x10);
    ili9341Command(0xC1);                       // Power control 2
    ili9341Data(0x10);
    ili9341Command(0xC5);                       // VCOM control 1
    ili9341Data(0x45); ili9341Data(0x15);
    ili9341Command(0xC7);                       // VCOM control 2
    ili9341Data(0x90);

    ili9341Command(0x36);                       // Memory access control
    ili9341Data(0xC8);

    // --- RGB interface ---
    ili9341Command(0xB0);                       // RGB interface signal control
    ili9341Data(0xC2);                          // DE mode, DOTCLK falling, HSYNC/VSYNC active-low

    ili9341Command(0x3A);                       // Pixel format: RGB565
    ili9341Data(0x55);

    ili9341Command(0xF2);                       // 3-gamma disable
    ili9341Data(0x00);

    ili9341Command(0xF6);                       // Interface control
    ili9341Data(0x01); 
    ili9341Data(0x00); 
    ili9341Data(0x06);

    ili9341Command(0x26);                       // Gamma curve 1
    ili9341Data(0x01);

    // --- Wake up ---
    ili9341Command(0x11);   // SLPOUT
    modm::delay(120ms);
    ili9341Command(0x29);   // Display ON
}

// ---------------------------------------------------------------------------
// Public entry point
// ---------------------------------------------------------------------------
void initializeDisplay()
{
    // initSdram() skipped – framebuffer is now in internal SRAM (test mode).
    PW_LOG_INFO("initPllSai...");
    initPllSai();
    PW_LOG_INFO("initLtdcGpio...");
    initLtdcGpio();
    PW_LOG_INFO("initLtdc...");
    initLtdc();
    PW_LOG_INFO("initIli9341...");
    initIli9341();
}
