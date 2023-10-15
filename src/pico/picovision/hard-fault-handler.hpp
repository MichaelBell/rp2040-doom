#include <string.h>
#include "hardware/gpio.h"
#include "hardware/structs/systick.h"

namespace ramshim {

  volatile uint32_t faults = 0;
  constexpr uint32_t _ram_start         = 0x2f000000;
  constexpr uint32_t _ram_size          =   0x800000;
  constexpr uint32_t _ram_end           = _ram_start + _ram_size;
}

#include "qspi-psram-cache.hpp"

namespace ramshim {

  ramshim::cache_t _cache;

  enum stack_offsets : int8_t {
    // values we added to the stack
    R4  = -5,
    R5  = -4,
    R6  = -3,
    R7  = -2,
   _LR  = -1,  // link register for the error handler

    // default stack state on exception
    R0  =  0,
    R1  =  1,
    R2  =  2,
    R3  =  3,
    R12 =  4,
    LR  =  5,
    PC  =  6,
    PSR =  7
  };

  // the registers are on the stack in the order 5, 6, 7, 8, 0, 1, 2, 3
  // this function takes the "natural" index of a register (e.g. 0 for r0,
  // or 2 for r2) and returns the index of it on the stack.
  __always_inline int8_t stack_index(uint8_t i) {
    //int8_t map[8] = {R0, R1, R2, R3, R4, R5, R6, R7};
    //return map[i];
    if (i >= 4) return (int8_t)i - 9;
    else return (int8_t)i;
  }

  // calculate address for register instruction
  __always_inline uint32_t raddr(uint16_t ins, uint32_t* stack) {
    uint8_t n = (ins >> 3) & 0b111;
    uint8_t m = (ins >> 6) & 0b111;
    return (stack[stack_index(n)] + stack[stack_index(m)]);
  }

  // calculate address for immediate instruction
  __always_inline uint32_t iaddr(uint16_t ins, uint8_t mul, uint32_t* stack) {
    uint8_t n = (ins >> 3) & 0b111;
    uint8_t o = (ins >> 6) & 0b11111;
    return (stack[stack_index(n)] + o * mul);
  }

  // return the source/target register for the instruction
  __always_inline uint32_t *srctar(uint16_t ins, uint32_t* stack) {
    uint8_t t = ins & 0b111;
    return &stack[stack_index(t)];
  }

}

extern "C"
{

  // stack will be populated with:
  // - r0, r1, r2, r3, r12
  // - lr (r14)
  // - pc (r15)     - address of faulting instruction (return address)
  // - xPSR

#define HF_PERF_COUNTER 0
#define ENABLE_HF_PROFILE 0

#if 1
  void __not_in_flash_func(isr_hardfault)()
  {
    asm(
#if HF_PERF_COUNTER
      "ldr r0, =0xe000e000\n"
      "movs r1, #0\n"
      "str r1, [r0, #0x18]\n"  // Start perf counter
#endif

      // put the stack pointer into r0, this points at the standard fault stack
      "mrs r0, msp\n"

      "ldr r3, [r0, #24]\n"  // Get PC
      "ldrh r1, [r3]\n"      // Get instruction and stash in r1 for handler
      "lsr r3, r1, #11\n"    // Opcode
      "lsl r3, r3, #2\n"     // 4 byte wide pointers
      "ldr  r2, =(handler_funcs - 40)\n"  // Handler table, adjusted as starts from 10.
      "ldr  r3, [r3, r2]\n"  // Address of handler

      // jump into our fault handler, which will immediately push {r4, r5, r6, r7, lr} below the fault stack.
      "bx r3\n"
    );
  }
#endif

  // list of opcodes we need to emulate

  // ldr   lit   01001 tttiiiiiiii
  // ldrsb reg   01010 11mmmnnnttt
  // ldr   reg   01011 00mmmnnnttt
  // ldrb  reg   01011 10mmmnnnttt
  // ldrh  reg   01011 01mmmnnnttt
  // ldrsh reg   01011 11mmmnnnttt
  // ldr   imm   01101 iiiiinnnttt
  // ldrb  imm   01111 iiiiinnnttt
  // ldrh  imm   10001 iiiiinnnttt
  // ldr   imm   10011 tttiiiiiiii

  // ldm         11001 nnnrrrrrrrr

  // str   imm   01100 iiiiinnnttt
  // str   reg   01010 00mmmnnnttt
  // strh  reg   01010 01mmmnnnttt
  // strb  reg   01010 10mmmnnnttt
  // strb  imm   01110 iiiiinnnttt
  // strh  imm   10000 iiiiinnnttt

  // stm         11000 nnnrrrrrrrr

#if HF_PERF_COUNTER
  uint32_t ticks_in_hf = 0;
#endif

  #define NUM_PROFILE_ENTRIES 400
  struct profile_entry {
    uint32_t pc;
    uint32_t count;
  } profile[NUM_PROFILE_ENTRIES] = {0};

  static void __not_in_flash_func(add_profile_entry)(uint32_t pc)
  {
    static uint8_t foo = 0;
    if (gpio_get(9) == 0 && (foo++ & 7) == 0) {
      for (int i = 0; i < NUM_PROFILE_ENTRIES; ++i) {
        if (profile[i].pc == pc) {
          profile[i].count++;
          return;
        }
        else if (profile[i].pc == 0) {
          profile[i].pc = pc;
          profile[i].count = 1;
          return;
        }
      }
    }
  }

  static void clear_profile() {
    memset(&profile, 0, sizeof(profile));
  }

  void picovision_print_profile() {
#if HF_PERF_COUNTER
    printf("Time in hf: %lu\n", ticks_in_hf / 266);
    ticks_in_hf = 0;
#endif
#if ENABLE_HF_PROFILE
    gpio_init(9);
    gpio_set_pulls(9, true, false);
    if (gpio_get(9) == 0) {
      printf("\nProfile:\n");
      for (int i = 0; i < NUM_PROFILE_ENTRIES; ++i) {
        if (profile[i].count > 50) {
          printf("%08x: %d\n", (uint)profile[i].pc, (uint)profile[i].count);
        }
      }
      clear_profile();
    }
#endif
  }

  void __not_in_flash_func(hard_fault_ldrsb)(uint32_t *stack, const uint16_t ins)
  {
    // Force the compiler to push r4-r7 onto the stack
    //asm("" : : : "r4", "r5", "r6", "r7" );
#if ENABLE_HF_PROFILE
    add_profile_entry(stack[ramshim::PC]);
#endif

    const uint8_t variant = (ins >>  9) & 0b11;    // 2 bit op code variant
    if(variant == 0b11) { // ldrsb (reg)
      uint32_t a =  ramshim::raddr(ins, stack);
      if ((a & 0xff800000) != 0x2f000000) __breakpoint();

      stack[ramshim::PC] += 2; 
      
      int32_t *t = (int32_t *)ramshim::srctar(ins, stack);
      // printf("ldrsb (reg): ");
      *t = ramshim::_cache.s8(a);

#if HF_PERF_COUNTER
      ticks_in_hf += 0x1000000 - systick_hw->cvr;
#endif
      return;
    }

    __breakpoint();
  }

  void __not_in_flash_func(hard_fault_ldr_reg)(uint32_t *stack, const uint16_t ins)
  {
    // Force the compiler to push r4-r7 onto the stack
    //asm("" : : : "r4", "r5", "r6", "r7" );

#if ENABLE_HF_PROFILE
    add_profile_entry(stack[ramshim::PC]);
#endif

    const uint8_t variant = (ins >>  9) & 0b11;    // 2 bit op code variant

    uint32_t  a =  ramshim::raddr(ins, stack);
    if ((a & 0xff800000) != 0x2f000000) __breakpoint();

    stack[ramshim::PC] += 2;

    uint32_t *t = ramshim::srctar(ins, stack);
    if(variant == 0b00) {                             // ldr
      //printf("ldr (reg): ");
      *t = ramshim::_cache.u32(a);
    } else if(variant == 0b01) {                      // ldrh
      // printf("ldrh (reg): ");
      *t = ramshim::_cache.u16(a);
    } else if(variant == 0b10) {                      // ldrb
      // printf("ldrb (reg): ");
      *t = ramshim::_cache.u8(a);
    } else if(variant == 0b11) {                      // ldrsh
      int32_t *t = (int32_t *)ramshim::srctar(ins, stack);
      // printf("ldrsh (reg): ");
      *t = ramshim::_cache.s16(a);
    }

#if HF_PERF_COUNTER
    ticks_in_hf += 0x1000000 - systick_hw->cvr;
#endif
  }

  void __not_in_flash_func(hard_fault_ldr_imm)(uint32_t *stack, const uint16_t ins)
  {
    // Force the compiler to push r4-r7 onto the stack
    //asm("" : : : "r4", "r5", "r6", "r7" );

#if ENABLE_HF_PROFILE
    add_profile_entry(stack[ramshim::PC]);
#endif

    uint32_t  a =  ramshim::iaddr(ins, 4, stack);
    if ((a & 0xff800000) != 0x2f000000) __breakpoint();

    stack[ramshim::PC] += 2;

    uint32_t *t = ramshim::srctar(ins, stack);
    //printf("ldr (imm) 0x%08x\n", a);
    *t = ramshim::_cache.u32(a);

#if HF_PERF_COUNTER
    ticks_in_hf += 0x1000000 - systick_hw->cvr;
#endif
  }

  void __not_in_flash_func(hard_fault_ldrh_imm)(uint32_t *stack, const uint16_t ins)
  {
    // Force the compiler to push r4-r7 onto the stack
    //asm("" : : : "r4", "r5", "r6", "r7" );

#if ENABLE_HF_PROFILE
    add_profile_entry(stack[ramshim::PC]);
#endif

    uint32_t  a =  ramshim::iaddr(ins, 2, stack);
    if ((a & 0xff800000) != 0x2f000000) __breakpoint();

    stack[ramshim::PC] += 2;

    uint32_t *t = ramshim::srctar(ins, stack);
    //printf("ldr (imm) 0x%08x\n", a);
    *t = ramshim::_cache.u16(a);

#if HF_PERF_COUNTER
    ticks_in_hf += 0x1000000 - systick_hw->cvr;
#endif
  }

  void __not_in_flash_func(hard_fault_ldrb_imm)(uint32_t *stack, const uint16_t ins)
  {
    // Force the compiler to push r4-r7 onto the stack
    //asm("" : : : "r4", "r5", "r6", "r7" );

#if ENABLE_HF_PROFILE
    add_profile_entry(stack[ramshim::PC]);
#endif

    uint32_t  a =  ramshim::iaddr(ins, 1, stack);
    if ((a & 0xff800000) != 0x2f000000) __breakpoint();

    stack[ramshim::PC] += 2;

    uint32_t *t = ramshim::srctar(ins, stack);
    //printf("ldr (imm) 0x%08x\n", a);
    *t = ramshim::_cache.u8(a);

#if HF_PERF_COUNTER
    ticks_in_hf += 0x1000000 - systick_hw->cvr;
#endif
  }

  void __not_in_flash_func(hard_fault_ldm)(uint32_t *stack, const uint16_t ins)
  {
    // Force the compiler to push r4-r7 onto the stack
    //asm("" : : : "r4", "r5", "r6", "r7" );

#if ENABLE_HF_PROFILE
    add_profile_entry(stack[ramshim::PC]);
#endif

    uint32_t addr_reg = (ins >> 8) & 0b111;
    uint32_t  a = stack[ramshim::stack_index(addr_reg)];
    if ((a & 0xff800000) != 0x2f000000) __breakpoint();

    uint32_t regs = ins & 0xFF;
    bool wback = !(regs & (1 << addr_reg));
    for (uint8_t i = 0; regs; ++i, regs >>= 1) {
      if (regs & 1) {
        uint32_t *t = &stack[ramshim::stack_index(i)];
        *t = ramshim::_cache.u32(a);
        a += 4;
      }
    }

    if (wback) {
      uint32_t *t = &stack[ramshim::stack_index(addr_reg)];
      *t = a;
    }

    stack[ramshim::PC] += 2;

#if HF_PERF_COUNTER
    ticks_in_hf += 0x1000000 - systick_hw->cvr;
#endif
  }

  typedef void (*handler_func)(uint32_t *stack, const uint16_t ins);
  handler_func handler_funcs[] = {
    hard_fault_ldrsb,    // 10 = 01010
    hard_fault_ldr_reg,  // 11 = 01011
    nullptr,
    hard_fault_ldr_imm,  // 13 = 01101
    nullptr,
    hard_fault_ldrb_imm, // 15 - 01111
    nullptr,
    hard_fault_ldrh_imm, // 17 = 10001
    nullptr,             // 18
    nullptr,             // 19
    nullptr,             // 20
    nullptr,             // 21
    nullptr,             // 22
    nullptr,             // 23
    nullptr,             // 24
    hard_fault_ldm,      // 25 = 11001
  };

#if 0
  void __not_in_flash_func(hard_fault_handler_c)(uint32_t *stack)
  {
    const uint16_t *&pc = reinterpret_cast<const uint16_t*&>(stack[ramshim::PC]);
    const uint16_t ins = *pc;
    const uint8_t opcode  = (ins >> 11) & 0b11111; // 5 bit op code

    handler_funcs[opcode - 10](stack, ins);
  }
#endif
}