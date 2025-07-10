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


#define ROM_BASE 0x20000000
#define ROM_SIZE 36

#define PRNG_BASE (ROM_BASE + 0x1000) // 4KB offset for PRNGs
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

int read_rom_string(char* buffer, int max_len) {
    volatile uint32_t* rom_ptr = (volatile uint32_t*)ROM_BASE;
    int char_count = 0;
    int word_idx = 0;
    
    // Read words from ROM and extract characters
    while (char_count < max_len - 1) {  // Leave space for null terminator
        uint32_t word = rom_ptr[word_idx];
        
        // Extract 4 bytes from the word (little-endian)
        for (int byte_idx = 0; byte_idx < 4 && char_count < max_len - 1; byte_idx++) {
            char ch = (word >> (byte_idx * 8)) & 0xFF;
            
            if (ch == '\0') {
                // Found null terminator
                buffer[char_count] = '\0';
                return char_count;
            }
            
            buffer[char_count++] = ch;
        }
        
        word_idx++;
        
        // Safety check to prevent reading beyond ROM
        if (word_idx * 4 >= ROM_SIZE) {
            break;
        }
    }
    
    buffer[char_count] = '\0';
    return char_count;
}

int main() {
    uart_init(); // setup the uart peripheral

    // simple printf support (only prints text and hex numbers)
    printf("Hello World!\n");
    // wait until uart has finished sending
    uart_write_flush();

    // toggling some GPIOs
    gpio_set_direction(0xFFFF, 0x000F); // lowest four as outputs
    gpio_write(0x0A);  // ready output pattern
    gpio_enable(0xFF); // enable lowest eight
    // wait a few cycles to give GPIO signal time to propagate
    asm volatile ("nop; nop; nop; nop; nop;");
    printf("GPIO (expect 0xA0): 0x%x\n", gpio_read());

    gpio_toggle(0x0F); // toggle lower 8 GPIOs
    asm volatile ("nop; nop; nop; nop; nop;");
    printf("GPIO (expect 0x50): 0x%x\n", gpio_read());
    uart_write_flush();

    // doing some compute
    uint32_t start = get_mcycle();
    uint32_t res   = isqrt(1234567890UL);
    uint32_t end   = get_mcycle();
    printf("Result: 0x%x, Cycles: 0x%x\n", res, end - start);
    uart_write_flush();

    // using the timer
    printf("Tick\n");
    sleep_ms(10);
    printf("Tock\n");
    uart_write_flush();



    char rom_string[64];  // Buffer for the string
    int length = read_rom_string(rom_string, sizeof(rom_string));
    
    printf("ROM String: %s\n", rom_string);
    printf("String length: %d\n", length);
    
    // Also show the raw hex data from ROM
    printf("Raw ROM data (first 9 words):\n");
    volatile uint32_t* rom_ptr = (volatile uint32_t*)ROM_BASE;
    for (int i = 0; i < 9; i++) {
        printf("Word %d: 0x%x\n", i, rom_ptr[i]);
    }
    
    // wait until uart has finished sending

    // // 32'h2000_0000;
    // volatile uint32_t* random_ptr = (volatile uint32_t*)0x20000000; // Pointer to RAM address
    // printf("pointer created\n");
    // uart_write_flush();
    
    // // Add a delay to ensure UART output completes
    // for(volatile int i=0; i<1000; i++) { asm("nop"); }

    // uint32_t seed = 0x600;
    // // Write the seed to the random number generator
    // *random_ptr = seed; // Write the seed to the address 0x200000
    // uint32_t random_val = *random_ptr;
    // printf("Random value read from 0x20000000: 0x%x\n", random_val);

    // random_ptr = (volatile uint32_t*)0x20000008; // Pointer to RAM address
    // printf("pointer created\n");
    // uart_write_flush();
    
    // // Add a delay to ensure UART output completes
    // for(volatile int i=0; i<1000; i++) { asm("nop"); }

    // seed = 0x600;
    // // Write the seed to the random number generator
    // // *random_ptr = seed; // Write the seed to the address 0x200000
    // random_val = *random_ptr;
    // printf("Random value read from 0x20000004: 0x%x\n", random_val);
    


    uart_write_flush();




    return 1;
}
