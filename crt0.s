    .arm
__start:
    b .Linit

    @ ROM header
    .byte 0x24,0xff,0xae,0x51,0x69,0x9a,0xa2,0x21,0x3d,0x84,0x82,0x0a,0x84,0xe4,0x09,0xad
    .byte 0x11,0x24,0x8b,0x98,0xc0,0x81,0x7f,0x21,0xa3,0x52,0xbe,0x19,0x93,0x09,0xce,0x20
    .byte 0x10,0x46,0x4a,0x4a,0xf8,0x27,0x31,0xec,0x58,0xc7,0xe8,0x33,0x82,0xe3,0xce,0xbf
    .byte 0x85,0xf4,0xdf,0x94,0xce,0x4b,0x09,0xc1,0x94,0x56,0x8a,0xc0,0x13,0x72,0xa7,0xfc
    .byte 0x9f,0x84,0x4d,0x73,0xa3,0xca,0x9a,0x61,0x58,0x97,0xa3,0x27,0xfc,0x03,0x98,0x76
    .byte 0x23,0x1d,0xc7,0x61,0x03,0x04,0xae,0x56,0xbf,0x38,0x84,0x00,0x40,0xa7,0x0e,0xfd
    .byte 0xff,0x52,0xfe,0x03,0x6f,0x95,0x30,0xf1,0x97,0xfb,0xc0,0x85,0x60,0xd6,0x80,0x25
    .byte 0xa9,0x63,0xbe,0x03,0x01,0x4e,0x38,0xe2,0xf9,0xa2,0x34,0xff,0xbb,0x3e,0x03,0x44
    .byte 0x78,0x00,0x90,0xcb,0x88,0x11,0x3a,0x94,0x65,0xc0,0x7c,0x63,0x87,0xf0,0x3c,0xaf
    .byte 0xd6,0x25,0xe4,0x8b,0x38,0x0a,0xac,0x72,0x21,0xd4,0xf8,0x07

    @ game title
    .byte 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00

    @ game code
    .byte 0x00,0x00,0x00,0x00

    @ maker code
    .byte 0x00,0x00

    .byte 0x96

    @ main unit code
    .byte 0x00

    @ device type (0x00 retail, 0x80 debug)
    .byte 0x00

    @ reserved
    .byte 0x00,0x00,0x00,0x00,0x00,0x00,0x00

    @ software version
    .byte 0x00

    @ complement check
    .byte 0x51

    @ reserved area
    .space 2

.Linit:
    @ Set address of user IRQ handler
    ldr r0, =MainIrqHandler
    ldr r1, =0x03FFFFFC
    str r0, [r1]

    @ set IRQ stack pointer
    mov r0, #0x12
    msr CPSR_c, r0
    ldr sp, =0x3007fa0

    @ set user stack pointer
    mov r0, #0x1f
    msr CPSR_c, r0
    ldr sp, =0x3007f00

	@ @ copy .data.slow section to EWRAM
    @ ldr r0, =__data_slow_lma     @ source address
    @ ldr r1, =__data_slow_start   @ destination address
    @ ldr r2, =__data_slow_end
    @ subs r2, r1             @ length
    @ @ these instructions are only executed if r2 is nonzero
    @ @ (i.e. don't bother copying an empty .data.slow section)
    @ addne r2, #3
    @ asrne r2, #2
    @ addne r2, #0x04000000
    @ swine 0xb0000

    @ copy .data section to IWRAM
    ldr r0, =__data_lma     @ source address
    ldr r1, =__data_start   @ destination address
    ldr r2, =__data_end
    subs r2, r1             @ length
    @ these instructions are only executed if r2 is nonzero
    @ (i.e. don't bother copying an empty .data section)
    addne r2, #3
    asrne r2, #2
    addne r2, #0x04000000
    swine 0xb0000

    @ jump to user code
    ldr r0, =main
    bx r0

    .arm
    .global MainIrqHandler
    .align 4, 0
MainIrqHandler:
    @ Load base I/O register address
    mov r2, #0x04000000
    add r2, r2, #0x200

    @ Save IRQ stack pointer and IME
    mrs r0, spsr
    ldrh r1, [r2, #8]
    stmdb sp!, {r0-r2,lr}

    @ Disable all interrupts by writing to IME
    @ r2 (0x4000200) can be used as we only care about bit 0 being unset
    strh r2, [r2, #8]

    @ Acknowledge all received interrupts that were enabled in IE
    ldr r3, [r2, #0]
    and r0, r3, r3, lsr #16
    strh r0, [r2, #2]

    @ Switch from IRQ mode to system mode
    @ cpsr_c = 0b000_10010u8 | 0b000_01101u8
    mrs r2, cpsr
    orr r2, r2, #0xD
    msr cpsr_c, r2

    @ Jump to user specified IRQ handler
    ldr r2, =__IRQ_HANDLER
    ldr r1, [r2]
    stmdb sp!, {lr}
    adr lr, .Lreturn
    bx r1
.Lreturn:
    ldmia sp!, {lr}

    @ Switch from ??? mode to IRQ mode, disable IRQ
    @ cpsr_c = ( !0b000_01101u8 & cpsr_c ) | 0b100_10010u8
    mrs r2, cpsr
    bic r2, r2, #0xD
    orr r2, r2, #0x92
    msr cpsr_c, r2

    @ Restore IRQ stack pointer and IME
    ldmia sp!, {r0-r2,lr}
    strh r1, [r2, #8]
    msr spsr_cf, r0

    @ Return to BIOS IRQ handler
    bx lr
    .pool
