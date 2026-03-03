// fault_handler.cpp
//
// HardFault / MemManage / BusFault / UsageFault handler for STM32F429I-DISCO.
//
// Writes a human-readable register + fault-status dump directly to USART1
// (the ST-LINK virtual COM port), bypassing modm IOStream entirely so that
// the dump works even when the normal logging stack is broken.
//
// After printing, breaks into GDB if a debugger is attached (checks
// CoreDebug->DHCSR.C_DEBUGEN), then spins forever.
//
// Inspired by pw_cpu_exception_cortex_m but self-contained — no extra Pigweed
// CMake targets required.

#include <cstdint>

// ---------------------------------------------------------------------------
// Cortex-M4 exception frame pushed automatically on exception entry
// ---------------------------------------------------------------------------
struct ExcFrame {
    uint32_t r0, r1, r2, r3, r12, lr, pc, xpsr;
};

// ---------------------------------------------------------------------------
// Direct USART1 writes — safe from any fault context
// STM32F4 USART1: base 0x40011000
//   SR  (+0x00): bit 7 = TXE (transmit data register empty)
//   DR  (+0x04): write byte here
// ---------------------------------------------------------------------------
namespace {

constexpr uintptr_t kUsart1Sr   = 0x40011000u;
constexpr uintptr_t kUsart1Dr   = 0x40011004u;
constexpr uintptr_t kUsart1Cr1  = 0x4001100Cu;
constexpr uintptr_t kCfsrAddr   = 0xE000ED28u;  // Configurable Fault Status
constexpr uintptr_t kHfsrAddr   = 0xE000ED2Cu;  // Hard Fault Status
constexpr uintptr_t kMmfarAddr  = 0xE000ED34u;  // MemManage Fault Address
constexpr uintptr_t kBfarAddr   = 0xE000ED38u;  // Bus Fault Address
constexpr uintptr_t kDhcsrAddr  = 0xE000EDF0u;  // Debug Halting Control

static inline volatile uint32_t& Reg(uintptr_t addr) {
    return *reinterpret_cast<volatile uint32_t*>(addr);
}

static void PutChar(char c) {
    // Wait for TXE with a timeout — a plain spin would hang forever if
    // modm's interrupt-driven TX ring buffer is stuck (the TXEIE ISR cannot
    // run at fault-handler priority, so the hardware drains the shift register
    // but the buffer pointer never advances until we write DR directly).
    for (uint32_t i = 0u; i < 1000000u; ++i) {
        if (Reg(kUsart1Sr) & (1u << 7)) break;  // TXE set
    }
    Reg(kUsart1Dr) = static_cast<uint8_t>(c);
}

static void PutStr(const char* s) {
    while (*s) PutChar(*s++);
}

static void PutHex32(uint32_t v) {
    static constexpr char kHex[] = "0123456789ABCDEF";
    PutStr("0x");
    for (int i = 7; i >= 0; --i)
        PutChar(kHex[(v >> (i * 4)) & 0xFu]);
}

// Print one line if a specific CFSR bit is set: "  <reg>.<name>\r\n"
static void DecodeFlag(uint32_t cfsr, uint8_t bit, const char* label) {
    if (cfsr & (1u << bit)) {
        PutStr("  ");
        PutStr(label);
        PutStr("\r\n");
    }
}

static void DumpFaultState(ExcFrame* f) {
    // Drain any in-progress modm UART transmission before writing directly.
    // In fault-handler context the TXEIE ISR cannot run, so we:
    //   1. Wait for TC (shift register fully empty, not just DR empty).
    //   2. Clear TXEIE so modm's ISR doesn't fight us when priority drops.
    for (uint32_t i = 0u; i < 2000000u; ++i) {
        if (Reg(kUsart1Sr) & (1u << 6)) break;  // TC set
    }
    Reg(kUsart1Cr1) &= ~(1u << 7);  // clear TXEIE

    PutStr("\r\n\r\n*** HARD FAULT ***\r\n");

    // Stacked exception frame
    PutStr("PC   = "); PutHex32(f->pc);   PutStr("\r\n");
    PutStr("LR   = "); PutHex32(f->lr);   PutStr("\r\n");
    PutStr("xPSR = "); PutHex32(f->xpsr); PutStr("\r\n");
    PutStr("R0   = "); PutHex32(f->r0);   PutStr("\r\n");
    PutStr("R1   = "); PutHex32(f->r1);   PutStr("\r\n");
    PutStr("R2   = "); PutHex32(f->r2);   PutStr("\r\n");
    PutStr("R3   = "); PutHex32(f->r3);   PutStr("\r\n");
    PutStr("R12  = "); PutHex32(f->r12);  PutStr("\r\n");

    // Fault status registers
    const uint32_t cfsr  = Reg(kCfsrAddr);
    const uint32_t hfsr  = Reg(kHfsrAddr);
    const uint32_t mmfar = Reg(kMmfarAddr);
    const uint32_t bfar  = Reg(kBfarAddr);

    PutStr("CFSR = "); PutHex32(cfsr);  PutStr("\r\n");
    PutStr("HFSR = "); PutHex32(hfsr);  PutStr("\r\n");
    if (cfsr & (1u << 7))  { PutStr("MMFAR= "); PutHex32(mmfar); PutStr("\r\n"); }
    if (cfsr & (1u << 15)) { PutStr("BFAR = "); PutHex32(bfar);  PutStr("\r\n"); }

    // Human-readable CFSR decode
    // MemManage faults (MMFSR, bits 7:0)
    DecodeFlag(cfsr,  0, "MMFSR.IACCVIOL  - instruction access violation");
    DecodeFlag(cfsr,  1, "MMFSR.DACCVIOL  - data access violation");
    DecodeFlag(cfsr,  3, "MMFSR.MUNSTKERR - unstacking exception return fault");
    DecodeFlag(cfsr,  4, "MMFSR.MSTKERR   - stacking exception entry fault");
    DecodeFlag(cfsr,  5, "MMFSR.MLSPERR   - FP lazy-save fault");
    // BusFault (BFSR, bits 15:8)
    DecodeFlag(cfsr,  8, "BFSR.IBUSERR    - instruction bus error");
    DecodeFlag(cfsr,  9, "BFSR.PRECISERR  - precise data bus error");
    DecodeFlag(cfsr, 10, "BFSR.IMPRECISERR- imprecise data bus error");
    DecodeFlag(cfsr, 11, "BFSR.UNSTKERR   - unstacking bus error");
    DecodeFlag(cfsr, 12, "BFSR.STKERR     - stacking bus error");
    DecodeFlag(cfsr, 13, "BFSR.LSPERR     - FP lazy-save bus error");
    // UsageFault (UFSR, bits 31:16)
    DecodeFlag(cfsr, 16, "UFSR.UNDEFINSTR - undefined instruction");
    DecodeFlag(cfsr, 17, "UFSR.INVSTATE   - invalid EPSR.T or EPSR.IT value");
    DecodeFlag(cfsr, 18, "UFSR.INVPC      - bad EXC_RETURN integrity check");
    DecodeFlag(cfsr, 19, "UFSR.NOCP       - coprocessor not present/disabled");
    DecodeFlag(cfsr, 24, "UFSR.UNALIGNED  - unaligned memory access");
    DecodeFlag(cfsr, 25, "UFSR.DIVBYZERO  - integer divide-by-zero");
    // HardFault escalation reason
    DecodeFlag(hfsr,  1, "HFSR.VECTTBL   - vector table read fault");
    DecodeFlag(hfsr, 30, "HFSR.FORCED    - escalated configurable fault");
    DecodeFlag(hfsr, 31, "HFSR.DEBUGEVT  - debug event (should not appear here)");

    PutStr("*** halting ***\r\n");

    // Trigger a GDB breakpoint if a debugger is connected; otherwise spin.
    // Checking C_DEBUGEN avoids a Lockup caused by BKPT with no debugger attached.
    if (Reg(kDhcsrAddr) & 0x1u) {
        __asm volatile("BKPT #0");
    }
    while (true) {}
}

} // namespace

// ---------------------------------------------------------------------------
// Dedicated fault handler stack
//
// Switching to a private stack immediately on exception entry means the dump
// succeeds even when the main stack has overflowed into the 0xcacacaca fill
// region.  1 KB is enough for DumpFaultState's call tree (PutStr/PutHex32).
// ---------------------------------------------------------------------------
static uint32_t fault_stack_buf[256];  // 1 KB

// C-linkage pointer to the top of the fault stack so the naked stub can load
// it with a single-indirection LDR from the literal pool.
extern "C" uint32_t* const fault_stack_top = fault_stack_buf + 256;

// ---------------------------------------------------------------------------
// C entry point — called by the naked entry stubs below
// ---------------------------------------------------------------------------
extern "C" void fault_handler_c(ExcFrame* frame) {
    DumpFaultState(frame);
}

// ---------------------------------------------------------------------------
// Naked entry stub:
//   1. Capture the exception frame pointer (MSP or PSP) into R0.
//   2. Switch SP to the dedicated fault stack so DumpFaultState has clean
//      stack space regardless of what happened to the main stack.
//   3. Tail-call fault_handler_c(frame).
//
// Overrides the weak modm alias for every configurable fault vector.
// ---------------------------------------------------------------------------

#define FAULT_ENTRY()                           \
    __asm volatile(                             \
        "tst    lr, #4              \n"         \
        "ite    eq                  \n"         \
        "mrseq  r0, msp             \n"         \
        "mrsne  r0, psp             \n"         \
        "ldr    r1, =fault_stack_top\n"         \
        "ldr    sp, [r1]            \n"         \
        "b      fault_handler_c     \n"         \
    )

extern "C" __attribute__((naked)) void HardFault_Handler(void)  { FAULT_ENTRY(); }
extern "C" __attribute__((naked)) void MemManage_Handler(void)  { FAULT_ENTRY(); }
extern "C" __attribute__((naked)) void BusFault_Handler(void)   { FAULT_ENTRY(); }
extern "C" __attribute__((naked)) void UsageFault_Handler(void) { FAULT_ENTRY(); }
