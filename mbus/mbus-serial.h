//------------------------------------------------------------------------------
// Copyright (C) 2011, Robert Johansson, Raditex AB
// All rights reserved.
//
// rSCADA
// http://www.rSCADA.se
// info@rscada.se
//
//------------------------------------------------------------------------------

/**
 * @file   mbus-serial.h
 *
 * @brief  Functions and data structures for sending M-Bus data via RS232.
 *
 */

#ifndef MBUS_SERIAL_H
#define MBUS_SERIAL_H

#ifndef RPI_PICO
#include <termios.h>
#else
#include <hardware/uart.h>
#endif
#include "mbus-protocol-aux.h"
#include "mbus-protocol.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct _mbus_serial_data
{
    char *device;
#ifndef RPI_PICO
    struct termios t;
#else
    uart_inst_t *uart;
    char *rx_buf;
    uint16_t first;
    uint16_t next;
    uint16_t data_len;
    int block_time_ms;
#endif
} mbus_serial_data;

int  mbus_serial_connect(mbus_handle *handle);
int  mbus_serial_disconnect(mbus_handle *handle);
int  mbus_serial_send_frame(mbus_handle *handle, mbus_frame *frame);
int  mbus_serial_recv_frame(mbus_handle *handle, mbus_frame *frame);
int  mbus_serial_set_baudrate(mbus_handle *handle, long baudrate);
void mbus_serial_data_free(mbus_handle *handle);
#ifdef RPI_PICO
void mbus_serial_pico_buf_init(mbus_serial_data *serial_data);
void mbus_serial_pico_buf_check(mbus_serial_data *serial_data);
void mbus_serial_pico_read_into_rx_buf(mbus_serial_data *serial_data);
ssize_t mbus_serial_pico_read(mbus_handle *handle, char *buf, ssize_t count);
void mbus_serial_pico_rx_irq_handler();
uart_inst_t *mbus_serial_pico_get_uart_from_fd(int);
#endif

#ifdef __cplusplus
}
#endif

#endif /* MBUS_SERIAL_H */

