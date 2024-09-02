/*
 *  modbus_crc.h
 *  Cross platform MODBUS library
 *  Optimized algorithm to compute MODBUS RTU Cyclic Redundancy Check
 *
 *  Created by Benoit BOUCHEZ (BEB)
 *
 * Copyright (c) 2024 Benoit BOUCHEZ
 * License : MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef __MODBUS_CRC_H__
#define __MODBUS_CRC_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Return the CRC for the pointed PDU. On a received frame, substract 2 from received PDULen (do not count received CRC)
uint16_t ComputeCRC (uint8_t* PDU, unsigned int PDULen);

#ifdef __cplusplus
}
#endif

#endif
