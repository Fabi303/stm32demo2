#pragma once

// LVGL local configuration for STM32F429I-DISCO
// Picked up automatically by the generated lv_conf.h via __has_include.

#define LV_COLOR_DEPTH  16   // RGB565 – matches the LTDC layer and ILI9341 config
// LV_HOR_RES / LV_VER_RES must NOT be defined here.
// In LVGL v9 these are runtime function macros; overriding them as compile-time
// constants corrupts internal stride/rendering calculations.
