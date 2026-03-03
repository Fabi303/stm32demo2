# STM32F429I-DISCO — LTDC + SDRAM + LVGL Demo

Minimal bare-metal C++ example for the STM32F429I-DISCO board demonstrating:

- **SDRAM framebuffer** (IS42S16400J, 8 MB, FMC bank 2, `0xD000_0000`)
- **LTDC** driving an ILI9341 2.4" TFT in **RGB-DE bypass mode** (no MCU pixel writes; pixel stream comes entirely from LTDC)
- **ILI9341** configured via SPI5 (register writes only; CS=PC2, DC=PD13)
- **LVGL v9** rendering directly into the LTDC framebuffer (no copy/DMA transfer needed)
- **Pigweed** (`pw_log`) for UART logging, **modm** as the HAL

---

## Hardware

| Item | Detail |
|---|---|
| MCU | STM32F429ZIT6 (168 MHz, 256 KB SRAM + 64 KB CCM) |
| Display | On-board 2.4" ILI9341 TFT, 240 × 320, RGB565 |
| Interface | LTDC RGB-DE (18-bit parallel) + SPI5 for ILI9341 config |
| Framebuffer | IS42S16400J SDRAM, bank 2 at `0xD000_0000` |

---

## Critical Issues with External SDRAM + LTDC

Getting LTDC to read cleanly from SDRAM requires attention to several non-obvious constraints. Each of the items below caused real, hard-to-diagnose display artifacts during development.

### 1. Framebuffer stride must align to the SDRAM page size

**Problem:** The SDRAM row (page) is 256 columns × 2 bytes = **512 bytes**. A naively sized framebuffer row is 240 × 2 = **480 bytes**. Because 480 and 512 are not multiples of each other, every display row after row 0 crosses a SDRAM page boundary at a *different horizontal pixel position* (advancing 16 pixels per row, repeating every 16 rows). The page-close (tRP) + page-open (tRCD) penalty arrives at a different column on each scan line, shifting that pixel by one position and creating a **comb artifact on every vertical edge**.

**Fix:** Pad the framebuffer stride to **512 bytes** (256 pixels per row, 240 active). Each display row then starts exactly on a SDRAM page boundary — no mid-scanline page crossing, no latency spike.

```cpp
// display_init.hpp
static constexpr uint32_t FB_STRIDE = 512;  // bytes, one full SDRAM page

// display_init.cpp — LTDC layer pitch
LTDC_Layer1->CFBLR = ((LCD_W * 2u + 3u) << LTDC_LxCFBLR_CFBLL_Pos) |
                     (FB_STRIDE          << LTDC_LxCFBLR_CFBP_Pos);

// main.cpp — LVGL must use the same stride
lv_display_set_buffers_with_stride(disp, Framebuffer, nullptr,
    (FB_STRIDE / 2u) * 320u * sizeof(uint16_t),
    FB_STRIDE, LV_DISPLAY_RENDER_MODE_DIRECT);

// Pixel addressing everywhere
Framebuffer[y * (FB_STRIDE / 2u) + x] = colour;
```

### 2. SDRAM burst length must match the LTDC DMA burst size

**Problem:** The SDRAM mode register was initialised with burst length = 1 (`MRD bits[2:0] = 000`). The LTDC DMA issues AHB INCR4 bursts (4 × 32-bit = 8 × 16-bit words per burst). With BL=1 the FMC must issue a separate SDRAM READ command for every word, each paying the full CAS latency. This creates excessive per-word overhead and LTDC FIFO stress, compounding the page-crossing problem above.

**Fix:** Set burst length = 4 (`MRD bits[2:0] = 010`) so one SDRAM burst delivers 4 × 16-bit words — two SDRAM bursts satisfy one INCR4 AHB burst with far less command overhead.

```cpp
// MRD = 0x0232: bit[9]=1 (single write burst), bits[6:4]=011 (CAS=3), bits[2:0]=010 (BL=4)
FMC_Bank5_6->SDCMR = (1u << FMC_SDCMR_CTB2_Pos) |
                     (4u << FMC_SDCMR_MODE_Pos)  |
                     (0x0232u << FMC_SDCMR_MRD_Pos);
```

> **Diagnostic:** Add `PW_LOG_INFO("LTDC ISR FUIF=%lu", (LTDC->ISR >> 1) & 1u);` after your test patterns. FUIF=1 means the LTDC FIFO ran dry — confirm SDRAM bandwidth/latency is the culprit before hunting elsewhere.

### 3. ILI9341 `0xB6` DFUNCTR — SM bit must be 0

**Problem:** Register `0xB6` (Display Function Control) parameter 1 contains the **SM bit at bit 3** (not bit 1 as some third-party examples show). SM=1 enables alternating gate scan, which maps even rows to one half of the panel and odd rows to the other. All rows appear the same colour — the display looks uniformly blank or shows a solid fill regardless of what is written to the framebuffer.

**Fix:** Write `0x00` for P1 in both `0xB6` writes (the RGB-DE mode requires two writes):

```cpp
ili9341Command(0xB6);
ili9341Data(0x00);   // P1: SM=0 (bit 3 clear) = normal sequential gate scan
ili9341Data(0xA7);   // P2: GS=1 (bit 7, reverse gate dir), NL=39 → 320 gate lines
ili9341Data(0x27);
ili9341Data(0x04);

ili9341Command(0xB6);  // second write required for RGB-DE mode
ili9341Data(0x00);
ili9341Data(0xA7);
```

> **Diagnostic:** Fill alternating rows with white/black. SM=1 collapses the zebra pattern to a solid colour; SM=0 shows clean alternating stripes.

### 4. LTDC pixel clock polarity (PCPOL) must be 0

**Problem:** The ILI9341 `0xB0` (RGB Interface Signal Control) is set with `DPL=1`, meaning it **samples pixel data on the falling edge** of DOTCLK. LTDC with `PCPOL=0` drives data on the rising edge, which is then stable at the falling-edge sample point — correct. Setting `PCPOL=1` inverts the LTDC clock, causing data to be driven on the falling edge and sampled during the transition — producing random pixel corruption.

**Fix:** Leave `PCPOL=0` (default after reset):
```cpp
LTDC->GCR &= ~(LTDC_GCR_HSPOL | LTDC_GCR_VSPOL | LTDC_GCR_DEPOL | LTDC_GCR_PCPOL);
```

### 5. PLLSAI pixel clock

The target pixel clock is ~6 MHz. With `PLLM=4` (set by the system clock), `PLLSAIN=192`, `PLLSAIR=4`, and `DCKCFGR.PLLSAIDIVR=/16`:

```
2 MHz × 192 / 4 / 16 = 6 MHz  ✓
```

### 6. LTDC CFBLR line-length field

`CFBLL` (line length in bytes) must include a +3 byte offset required by the LTDC hardware:

```cpp
CFBLL = LCD_W * 2 + 3   // = 483 for 240-pixel RGB565 rows
```

Omitting the +3 causes the last pixel(s) of each line to read from the wrong address.

### 7. CCM RAM cannot be used for the framebuffer

The STM32F429's 64 KB CCM RAM (`0x1000_0000`) is **CPU-only** — it is not accessible by the DMA bus matrix. The LTDC DMA master cannot read from CCM. The framebuffer must reside in the AHB-accessible SRAM (`0x2000_0000`, SRAM1+2+3 = 192 KB) or in external SDRAM.

---

## LVGL v9 Notes

- `LV_HOR_RES` / `LV_VER_RES` are **runtime function macros** in v9 — never define them as compile-time constants in `lv_conf.h`. Doing so corrupts internal stride and rendering calculations.
- Use `LV_DISPLAY_RENDER_MODE_DIRECT` with the SDRAM framebuffer as the single buffer — LVGL renders straight into the LTDC framebuffer and the flush callback just calls `lv_display_flush_ready()` immediately.
- Pass the explicit stride to `lv_display_set_buffers_with_stride()` so LVGL never recomputes it from `LV_DRAW_BUF_STRIDE_ALIGN`.

---

## Build

```bash
# Generate modm HAL (lbuild) and build with CMake
lbuild build
cmake -B build -G Ninja
cmake --build build
# Flash with OpenOCD or ST-LINK
```
