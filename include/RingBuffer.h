/*
 * Based on mbed library for RingBuffer
 * Copyright (c) 2010 Hiroshi Suga
 * Released under the MIT License: http://mbed.org/license/mit
 */

/** @file RingBuffer.h
 * @brief Ring Buffer
 */
#include "stm32f4xx_hal.h"

#ifndef RingBuffer_H
#define RingBuffer_H


class RingBuffer {
public:
    /** init Stack class
     * @param p_size size of ring buffer
     */
    RingBuffer (int p_size);
    ~RingBuffer ();

    /** put to ring buffer
     * @param dat data
     * @return data / -1:error
     */
    int put(uint16_t dat);

    /** put to ring buffer
     * @param dat data
     * @param len length
     * @return put length
     */
    int put(uint16_t *dat, int len);

    /** get from ring buffer
     * @param dat data
     * @retval 0:ok / -1:error
     */
    int get(uint16_t *dat);

    /** get from ring buffer
     * @param dat data
     * @param len length
     * @return get length
     */
    int get(uint16_t *dat, int len);

    void clear ();
    int available ();
    int use ();

private:
    uint16_t *buf;
    int size;
    int addr_w, addr_r;
};

#endif
