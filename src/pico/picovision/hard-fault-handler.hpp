#include "qspi-psram-cache.hpp"



namespace ramshim {

  volatile uint32_t faults = 0;
  constexpr uint32_t _ram_start         = 0x2f000000;
  constexpr uint32_t _ram_size          =   0x800000;
  constexpr uint32_t _ram_end           = _ram_start + _ram_size;

  ramshim::cache_t _cache(_ram_start);

  enum stack_offsets : uint8_t {
    // values we added to the stack
    R4  =  0,
    R5  =  1,
    R6  =  2,
    R7  =  3,
   _LR  =  4,  // link register for the error handler

    // default stack state on exception
    R0  =  5,
    R1  =  6,
    R2  =  7,
    R3  =  8,
    R12 =  9,
    LR  = 10,
    PC  = 11,
    PSR = 12
  };

  // the registers are on the stack in the order 5, 6, 7, 8, 0, 1, 2, 3
  // this function takes the "natural" index of a register (e.g. 0 for r0,
  // or 2 for r2) and returns the index of it on the stack.
  uint8_t stack_index(uint8_t i) {
    uint8_t map[8] = {R0, R1, R2, R3, R4, R5, R6, R7};
    return map[i];
  }

  // calculate address for register instruction
  uint32_t raddr(uint16_t ins, uint32_t* stack) {
    uint8_t n = (ins & 0b000111000) >> 3;
    uint8_t m = (ins & 0b111000000) >> 6;
    return (stack[stack_index(n)] + stack[stack_index(m)]);
  }

  // calculate address for immediate instruction
  uint32_t iaddr(uint16_t ins, uint8_t mul, uint32_t* stack) {
    uint8_t n = (ins & 0b000111000) >> 3;
    uint8_t o = (ins >> 6) & 0b11111;
    return (stack[stack_index(n)] + o * mul);
  }

  // return the source/target register for the instruction
  uint32_t *srctar(uint16_t ins, uint32_t* stack) {
    uint8_t t = (ins & 0b000000111) >> 0;
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

#if 1
  void isr_hardfault()
  {
    asm(
      // push link register onto the stack
      "push {lr}\n"

      // push r4-r7 onto the stack
      "push {r4-r7}\n"

      // put the stack pointer into r0
      "mrs r0, msp\n"

      // call into our fault handler (r0-r3 are automatically pushed??)
      "bl hard_fault_handler_c\n"

      // restore r4-r7
      "pop {r4-r7}\n"

      // put link register value into program counter
      "pop {pc}\n"
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

  void hard_fault_handler_c(uint32_t *stack)
  {
    const uint16_t *&pc = reinterpret_cast<const uint16_t*&>(stack[ramshim::PC]);
    const uint16_t ins = *pc;
    const uint8_t opcode  = (ins >> 11) & 0b11111; // 5 bit op code
    const uint8_t variant = (ins >>  9) & 0b11;    // 2 bit op code variant

#if 0
    // storing instructions
    if(opcode == 0b01100) { // str (imm)
      uint32_t a =  ramshim::iaddr(ins, 4, stack);
      uint32_t *t = ramshim::srctar(ins, stack);
      //printf("str (imm) 0x%08x\n", a);
      // printf("str (imm): ");
      ramshim::_cache.u32(a, *t);

      pc++; return;
    }

    if(opcode == 0b01010 && variant != 0b11) { // str/strh/strb (reg)
      uint32_t  a =  ramshim::raddr(ins, stack);
      if(variant == 0b00) {                             // str
        uint32_t *t = ramshim::srctar(ins, stack);
        // printf("str (reg): ");
        ramshim::_cache.u32(a, *t);
      } else if(variant == 0b01) {                      // strh
        uint16_t *t = (uint16_t *)ramshim::srctar(ins, stack);
        // printf("strh (reg): ");
        ramshim::_cache.u16(a, *t);
      } else if(variant == 0b10) {                      // strb
        uint8_t  *t = (uint8_t  *)ramshim::srctar(ins, stack);
        // printf("strb (reg): ");
        ramshim::_cache.u8(a, *t);
      }
      pc++; return;
    }

    if(opcode == 0b01110) { // strb (imm)
      uint32_t a =  ramshim::iaddr(ins, 4, stack);
      uint8_t *t = (uint8_t *)ramshim::srctar(ins, stack);
      // printf("strb (imm): 0x%08x", a);
      ramshim::_cache.u8(a, *t);
      pc++; return;
    }

    if(opcode == 0b10000) { // strh (imm)
      uint32_t  a =  ramshim::iaddr(ins, 4, stack);
      uint16_t *t = (uint16_t *)ramshim::srctar(ins, stack);
      // printf("strh (imm): ");
      ramshim::_cache.u16(a, *t);
      pc++; return;
    }

    if(opcode == 0b11000) { // stm
      // used by memset
      uint32_t  a = ramshim::iaddr(ins, 4, stack);
      uint32_t *t = ramshim::srctar(ins, stack);

      ramshim::_cache.u32(a, *t);

      pc++; return;
    }
#endif
    // loading instructions

    if(opcode == 0b01001) { // ldr (lit)
      uint32_t  a = ramshim::iaddr(ins, 4, stack);
      uint32_t *t = ramshim::srctar(ins, stack);

      // printf("LDR (imm) %#010x > %#010x\n", a, *t);
      *t = ramshim::_cache.u32(a);

      pc++; return;
    }


    if(opcode == 0b01010 && variant == 0b11) { // ldrsb (reg)
      uint32_t a =  ramshim::raddr(ins, stack);
      int8_t *t = (int8_t *)ramshim::srctar(ins, stack);
      // printf("ldrsb (reg): ");
      *t = ramshim::_cache.s8(a);
      pc++; return;
    }

    if(opcode == 0b01011) { // ldr/ldrb/ldrh/ldrsh (reg)
      uint32_t  a =  ramshim::raddr(ins, stack);
      uint32_t *t = ramshim::srctar(ins, stack);
      if(variant == 0b00) {                             // ldr
        //uint32_t *t = (uint32_t *)ramshim::srctar(ins, stack);
        //printf("ldr (reg): ");
        *t = ramshim::_cache.u32(a);
      } else if(variant == 0b01) {                      // ldrh
        //uint16_t *t = (uint16_t *)ramshim::srctar(ins, stack);
        // printf("ldrh (reg): ");
        *t = ramshim::_cache.u16(a);
      } else if(variant == 0b10) {                      // ldrb
        //uint8_t *t = (uint8_t *)ramshim::srctar(ins, stack);
        // printf("ldrb (reg): ");
        *t = ramshim::_cache.u8(a);
      } else if(variant == 0b11) {                      // ldrsh
        //int16_t *t = (int16_t *)ramshim::srctar(ins, stack);
        // printf("ldrsh (reg): ");
        *t = ramshim::_cache.s16(a);
      }
      pc++; return;
    }

    if(opcode == 0b01101) { // ldr (imm)
      uint32_t  a =  ramshim::iaddr(ins, 4, stack);
      uint32_t *t = ramshim::srctar(ins, stack);
      //printf("ldr (imm) 0x%08x\n", a);
      *t = ramshim::_cache.u32(a);
      pc++; return;
    }

    if(opcode == 0b10001) { // ldrh (imm)
      uint32_t a = ramshim::iaddr(ins, 2, stack);
      uint32_t *t = ramshim::srctar(ins, stack);
      // printf("ldrh (imm): ");
      *t = ramshim::_cache.u16(a);
      pc++; return;
    }

    if(opcode == 0b01111) { // ldrb (imm)
      uint32_t  a =  ramshim::iaddr(ins, 1, stack);
      uint32_t *t = ramshim::srctar(ins, stack);
      // printf("ldrb (imm): 0x%08x", a);
      *t = ramshim::_cache.u8(a);
      pc++; return;
    }

    if(opcode == 0b11001) { // ldm
      // TODO: Could do a longer read from cache.
      uint32_t addr_reg = (ins >> 8) & 0b111;
      uint32_t  a = stack[ramshim::stack_index(addr_reg)];
      uint32_t regs = ins & 0xFF;
      bool wback = !(regs & (1 << addr_reg));
      for (int i = 0; regs; ++i, regs >>= 1) {
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

      pc++; return;
    }
  }

}