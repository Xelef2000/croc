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
    uart_init();
    printf("Starting ROM test...\n");
    uart_write_flush();
    
    // Print address we're about to access to help with debugging
    printf("About to access address 0x10001000\n");
    uart_write_flush();
    
    volatile uint32_t* ram_ptr = (volatile uint32_t*)0x10001016;
    printf("pointer created\n");
    uart_write_flush();
    
    // Add a delay to ensure UART output completes
    for(volatile int i=0; i<1000; i++) { asm("nop"); }

    // Write 
    *ram_ptr = 0xDEADBEEF; 

    // Try the actual read
    uint32_t ram_val = *ram_ptr;
    
    // If we get here, print the value
    printf("read successful! Value: 0x%x\n", ram_val);
    uart_write_flush();
    
    // Rest of your program...
    return 1;
}