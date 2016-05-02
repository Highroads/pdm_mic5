/*
 * mbed library for RingBuffer
 * Copyright (c) 2010 Hiroshi Suga
 * Released under the MIT License: http://mbed.org/license/mit
 */

/** @file RingBuffer.cpp
 * @brief Ring Buffer
 */
 
#include "RingBuffer.h"

RingBuffer::RingBuffer (int p_size) {
    size = p_size + 1;
    buf = new uint16_t[size];
    addr_w = 0;
    addr_r = 0;
}

RingBuffer::~RingBuffer () {
    delete [] buf;
}

int RingBuffer::put (uint16_t dat) {
    int next;

    next = (addr_w + 1) % size;
    if (next == addr_r) {
        return -1;
    }
    buf[addr_w] = dat;
    addr_w = next;
    return -1;
}

int RingBuffer::put (uint16_t *dat, int len) {
    int next, i;


    for (i = 0; i < len; i ++) {
        next = (addr_w + 1) % size;
        if (next == addr_r) {
            break;
        }
        buf[addr_w] = dat[i];
        addr_w = next;
    }
    return i;
}

int RingBuffer::get (uint16_t *dat) {
    if (addr_r == addr_w) {
        return -1;
    }
    *dat = buf[addr_r];
    addr_r = (addr_r + 1) % size;
    return 0;
}

int RingBuffer::get (uint16_t *dat, int len) {
    int i;

    for (i = 0; i < len; i ++) {
        if (addr_r == addr_w) {
            break;
        }
        dat[i] = buf[addr_r];
        addr_r = (addr_r + 1) % size;
    }
    return i;
}

int RingBuffer::available () {
    if (addr_w < addr_r) {
        return addr_r - addr_w - 1;
    } else {
        return (size - addr_w) + addr_r - 1;
    }
}

int RingBuffer::use () {
    return size - available() - 1;
}

void RingBuffer::clear () {
    addr_w = 0;
    addr_r = 0;
}
