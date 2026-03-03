#define PW_LOG_MODULE_NAME "MAIN"
#include "pw_log/log.h"

#include <modm/board.hpp>
#include "display_init.hpp"
#include <lvgl/lvgl.h>

using namespace Board;

// LVGL display flush callback.
// The display driver is set up in LV_DISPLAY_RENDER_MODE_DIRECT, which means
// LVGL renders straight into the LTDC framebuffer – nothing to transfer here.
static void disp_flush(lv_display_t* disp, const lv_area_t* /*area*/, uint8_t* /*px_map*/)
{
    lv_display_flush_ready(disp);
}

// LVGL timer callback: update a counter label every second
static lv_obj_t* counterLabel;
static void timer_cb(lv_timer_t* /*t*/)
{
    static uint32_t count = 0;
    lv_label_set_text_fmt(counterLabel, "Count: %lu", count++);
}

int main()
{
    Board::initialize();

    PW_LOG_INFO("STM32F429I-DISCO LVGL demo starting");
    PW_LOG_INFO("System clock: %lu Hz", SystemClock::Frequency);

    // Bring up SDRAM → LTDC → ILI9341
    initializeDisplay();
    PW_LOG_INFO("Display initialised");
    // LTDC register readback – CFBLR should be 0x01E001E3 (CFBP=480, CFBLL=483)
    // CFBLNR should be 320
    // CFBLL = 240*2+3 = 483 = 0x1E3; CFBP = 512 = 0x200 → CFBLR = 0x02000 1E3
    PW_LOG_INFO("LTDC CFBLR=0x%08lX (expect 0x020001E3)  CFBLNR=%lu (expect 320)",
                (unsigned long)LTDC_Layer1->CFBLR,
                (unsigned long)LTDC_Layer1->CFBLNR);

    // Stride/layout smoke-test.
    // Phase 1 (vertical stripes): 10-px wide R/G/B columns.  Any diagonal tilt
    //   means LTDC CFBP ≠ framebuffer pitch → LTDC/SDRAM issue.
    // Phase 2 (horizontal bands): 10-row tall R/G/B bands.  Any diagonal tilt
    //   means LVGL stride ≠ LTDC pitch → LVGL stride issue.
    // Both phases should show perfectly axis-aligned bands if all is correct.
    // Phase 0 – SM gate-scan diagnostic (2 s)
    // If SM=1 (alternating gate scan): even rows map to the first half of the panel and
    // odd rows to the second half → all rows appear the same colour → solid screen.
    // If SM=0 (correct): 1-px alternating white/black horizontal lines = clean zebra stripes.
    // Row stride in pixels (FB_STRIDE is in bytes; each pixel is 2 bytes).
    constexpr uint32_t STRIDE_PX = FB_STRIDE / 2u;   // 256

    for (uint32_t y = 0; y < 320u; ++y)
        for (uint32_t x = 0; x < 240u; ++x)
            Framebuffer[y * STRIDE_PX + x] = (y & 1u) ? 0xFFFFu : 0x0000u;
    modm::delay(2000ms);

    {
        // Phase 1 – vertical stripes
        static const uint16_t col[3] = { 0xF800u, 0x07E0u, 0x001Fu };
        for (uint32_t y = 0; y < 320u; ++y)
            for (uint32_t x = 0; x < 240u; ++x)
                Framebuffer[y * STRIDE_PX + x] = col[(x / 10u) % 3u];
    }
    modm::delay(2000ms);
    {
        // Phase 2 – horizontal bands
        static const uint16_t row[3] = { 0xF800u, 0x07E0u, 0x001Fu };
        for (uint32_t y = 0; y < 320u; ++y)
            for (uint32_t x = 0; x < 240u; ++x)
                Framebuffer[y * STRIDE_PX + x] = row[(y / 10u) % 3u];
    }
    modm::delay(2000ms);

    //3rd test
    for (uint32_t y = 0; y < 320; y++) {
        for (uint32_t x = 0; x < 240; x++) {
            Framebuffer[y * STRIDE_PX + x] = (x % 20 < 10) ? 0xF800 : 0x001F;
        }
    }
    modm::delay(2000ms);

    // Check for LTDC FIFO underrun (FUIF) or transfer error (TERRIF).
    // Either flag means LTDC ran out of data from memory → pixel corruption.
    uint32_t isr = LTDC->ISR;
    PW_LOG_INFO("LTDC ISR after patterns: 0x%02lX  FUIF=%lu TERRIF=%lu",
                (unsigned long)isr,
                (unsigned long)((isr >> 1) & 1u),   // bit 1 = FUIF
                (unsigned long)((isr >> 2) & 1u));  // bit 2 = TERRIF
    LTDC->ICR = 0x0Fu;   // clear all flags

    // Register the LVGL display.
    // modm:lvgl auto-called lv_init() and set the tick callback before main().
    lv_display_t* disp = lv_display_create(240, 320);
    lv_display_set_flush_cb(disp, disp_flush);
    // DIRECT mode: LVGL renders straight into the LTDC framebuffer.
    // Stride = FB_STRIDE (512 bytes) matches LTDC CFBP; pads each row to one
    // SDRAM page so LTDC DMA never crosses a page boundary mid-scanline.
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers_with_stride(disp,
        Framebuffer, nullptr,
        (FB_STRIDE / 2u) * 320u * sizeof(uint16_t),
        FB_STRIDE,                       // stride = 512 bytes = LTDC CFBP
        LV_DISPLAY_RENDER_MODE_DIRECT);

    // --- Build a simple UI ---
    lv_obj_t* screen = lv_screen_active();
    // bg_opa must be set explicitly — LVGL v9 defaults to transparent.
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x003A57), LV_PART_MAIN);

    // Title label
    lv_obj_t* title = lv_label_create(screen);
    lv_label_set_text(title, "Hello LVGL!");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    // Counter label (updated by timer)
    counterLabel = lv_label_create(screen);
    lv_label_set_text(counterLabel, "Count: 0");
    lv_obj_set_style_text_color(counterLabel, lv_color_hex(0xFFD700), LV_PART_MAIN);
    lv_obj_align(counterLabel, LV_ALIGN_CENTER, 0, 0);

    // Simple button
    lv_obj_t* btn = lv_button_create(screen);
    lv_obj_set_size(btn, 100, 40);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_style_shadow_width(btn, 0, 0);  // disable shadow – avoids comb artifact on TN panel
    lv_obj_t* btnLabel = lv_label_create(btn);
    lv_label_set_text(btnLabel, "Press me");
    lv_obj_center(btnLabel);

    // 1-second periodic timer to update the counter
    lv_timer_create(timer_cb, 1000, nullptr);

    // Main loop
    while (true)
    {
        lv_timer_handler();      // drives LVGL rendering + timers
        LedGreen::toggle();
        modm::delay(5ms);        // ~200 Hz loop; adjust as needed
    }
}
