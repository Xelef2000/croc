// Copyright (c) 2024 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0/
//
// Authors:
// - Philippe Sauter <phsauter@iis.ee.ethz.ch>

#include "uart.h"
#include "print.h"
#include "timer.h"
#include "gpio.h"
#include "util.h"

// ROM address space
#define ROM_BASE 0x20000000
#define ROM_SIZE 36

// PRNG address space (moved to separate range)
#define PRNG_BASE 0x20001000
#define PRNG_0    (PRNG_BASE + 0x0)  // First PRNG
#define PRNG_1    (PRNG_BASE + 0x4)  // Second PRNG

/// @brief Example integer square root
/// @return integer square root of n
uint32_t isqrt(uint32_t n) {
    uint32_t res = 0;
    uint32_t bit = (uint32_t)1 << 30;

    while (bit > n) bit >>= 2;

    while (bit) {
        if (n >= res + bit) {
            n -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}


int main() {
    uart_init(); // setup the uart peripheral

    // simple printf support (only prints text and hex numbers)
    printf("Hello World!\n");
    // wait until uart has finished sending
    uart_write_flush();

    // toggling some GPIOs
    // gpio_set_direction(0xFFFF, 0x000F); // lowest four as outputs
    // gpio_write(0x0A);  // ready output pattern
    // gpio_enable(0xFF); // enable lowest eight
    // // wait a few cycles to give GPIO signal time to propagate
    // asm volatile ("nop; nop; nop; nop; nop;");
    // printf("GPIO (expect 0xA0): 0x%x\n", gpio_read());

    // gpio_toggle(0x0F); // toggle lower 8 GPIOs
    // asm volatile ("nop; nop; nop; nop; nop;");
    // printf("GPIO (expect 0x50): 0x%x\n", gpio_read());
    // uart_write_flush();

    // // doing some compute
    // uint32_t start = get_mcycle();
    // uint32_t res   = isqrt(1234567890UL);
    // uint32_t end   = get_mcycle();
    // printf("Result: 0x%x, Cycles: 0x%x\n", res, end - start);
    // uart_write_flush();

    // // using the timer
    // printf("Tick\n");
    // sleep_ms(10);
    // printf("Tock\n");
    // uart_write_flush();

    // Read and display ROM string

    volatile uint32_t* rom_ptr = (volatile uint32_t*)ROM_BASE;
    for (int i = 0; i < 9; i++) {
        volatile uint32_t val = *rom_ptr;
        char* chars = (char*)&val;
        printf("ROM chars: %c%c%c%c\n", chars[0], chars[1], chars[2], chars[3]);
        rom_ptr++;
    }


    uart_write_flush();

    // Test PRNG functionality with updated addresses
    printf("Testing PRNG at new address 0x%x\n", PRNG_BASE);
    
    volatile uint32_t* prng_ptr_0 = (volatile uint32_t*)PRNG_0;
    volatile uint32_t* prng_ptr_1 = (volatile uint32_t*)PRNG_1;
    
    printf("PRNG pointers created\n");
    uart_write_flush();
    
    // Add a delay to ensure UART output completes
    for(volatile int i=0; i<1000; i++) { asm("nop"); }

    uint32_t seed = 0x600;
    // Write the seed to the first PRNG
    *prng_ptr_0 = seed;
    uint32_t random_val_0 = *prng_ptr_0;
    printf("Random value from PRNG_0 (0x%x): 0x%x\n", PRNG_0, random_val_0);
    uart_write_flush();
    
    // Add a delay
    for(volatile int i=0; i<1000; i++) { asm("nop"); }

    // Test second PRNG
    uint32_t random_val_1 = *prng_ptr_1;
    printf("Random value from PRNG_1 (0x%x): 0x%x\n", PRNG_1, random_val_1);
    uart_write_flush();

    // Generate a few more random numbers to demonstrate functionality
    printf("Generating more random numbers:\n");
    for(int i = 0; i < 5; i++) {
        uint32_t rnd = *prng_ptr_0;
        printf("PRNG_0: 0x%x\n", rnd);
        
        // Small delay between reads
        for(volatile int j=0; j<100; j++) { asm("nop"); }
    }
    uart_write_flush();

    return 1;
}