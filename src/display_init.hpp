#pragma once
#include <cstdint>

// Framebuffer (TEST: internal SRAM; production: SDRAM at 0xD000_0000).
// Valid only after initializeDisplay() returns.
extern uint16_t* const Framebuffer;

// Row stride in bytes.  Padded to 512 to align each row with an SDRAM page
// boundary, eliminating mid-scanline page-close/open latency spikes.
// Use this constant (not 240*2) everywhere a row offset is computed.
static constexpr uint32_t FB_STRIDE = 512;

// Initialise the onboard 2.4" ILI9341 display on the STM32F429I-DISCO.
// Sequence: SDRAM → PLLSAI/LTDC clock → LTDC peripheral → ILI9341 via SPI5.
// Call once, after Board::initialize().
void initializeDisplay();
