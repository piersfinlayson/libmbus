//------------------------------------------------------------------------------
// Copyright (C) 2011, Robert Johansson, Raditex AB
// All rights reserved.
//
// rSCADA
// http://www.rSCADA.se
// info@rscada.se
//
//------------------------------------------------------------------------------

#ifndef RPI_PICO
#include <unistd.h>
#include <limits.h>
#include <fcntl.h>
#include <sys/types.h>

#include <stdio.h>
#include <strings.h>

#include <termios.h>
#include <errno.h>
#include <string.h>
#else
#include <stdio.h>
#include <string.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <hardware/uart.h>
#define fprintf(fd, ...) printf(__VA_ARGS__)
#define SSIZE_MAX (sizeof(ssize_t) * 256)
#endif

#include "mbus-serial.h"
#include "mbus-protocol-aux.h"
#include "mbus-protocol.h"

#define PACKET_BUFF_SIZE 2048
#ifdef RPI_PICO
volatile void *irq_uart_handle;

// Initialize the rx buffer
void mbus_serial_pico_buf_init(mbus_serial_data *serial_data)
{
    serial_data->first = 0;
    serial_data->next = 0;
    serial_data->data_len = 0;
    irq_uart_handle = (void *)serial_data;
}

// Check for consistency
void mbus_serial_pico_buf_check(mbus_serial_data *serial_data)
{
    int check_data_len;
    // Check for buffer errors - if errors reinitialize the buffer
    if ((serial_data->first >= PICO_MBUS_RX_BUF_LEN) ||
        (serial_data->next >= PICO_MBUS_RX_BUF_LEN) ||
        (serial_data->data_len > PICO_MBUS_RX_BUF_LEN) ||
        ((serial_data->data_len == PICO_MBUS_RX_BUF_LEN) && 
            (serial_data->next != serial_data->first)))
    {
#ifdef MBUS_SERIAL_DEBUG
        printf("%s: Invalid buffer value: data_len: %d first: %d next: %d\n",
               __PRETTY_FUNCTION__,
               serial_data->data_len,
               serial_data->first,
               serial_data->next);
#endif               
        mbus_serial_pico_buf_init(serial_data);
    }
    check_data_len = (serial_data->next - serial_data->first) % PICO_MBUS_RX_BUF_LEN;
    if (check_data_len < 0)
    {
        check_data_len = PICO_MBUS_RX_BUF_LEN + check_data_len;
    }
    if (check_data_len != (serial_data->data_len % PICO_MBUS_RX_BUF_LEN))
    {
#ifdef MBUS_SERIAL_DEBUG
        printf("%s: Invalid data_len: %d first: %d next: %d\n",
               __PRETTY_FUNCTION__,
               serial_data->data_len,
               serial_data->first,
               serial_data->next);
#endif
        mbus_serial_pico_buf_init(serial_data);
    }
}

// Read all outstanding RX data into ring buffer
void mbus_serial_pico_read_into_rx_buf(mbus_serial_data *serial_data)
{
    bool discard = false;
    char byte_read;

    // Do all of our checks within this while loop, as we the know that we're about to read in at least one byte
    while (uart_is_readable(serial_data->uart))
    {
        mbus_serial_pico_buf_check(serial_data);

        // Check if the buffer is already full
        if (serial_data->data_len == PICO_MBUS_RX_BUF_LEN)
        {
#ifdef PICO_MBUS_RX_BUF_FULL_OVERWRITE
            if (PICO_MBUS_RX_BUF_FULL_OVERWRITE)
#else
            if (false)
#endif                
            {
                serial_data->first++;
                serial_data->data_len--;
                if (serial_data->first >= PICO_MBUS_RX_BUF_LEN)
                {
                    serial_data->first = 0;
                }
            }
            else
            {
                discard = true;
            }
        }

        // Get the byte and store it if the buffer isn't full (and we've not be told to overwrite)
        byte_read = uart_getc(serial_data->uart);
        if (!discard)
        {
            serial_data->rx_buf[serial_data->next] = byte_read;
            serial_data->next++;
            serial_data->data_len++;
            if (serial_data->next >= PICO_MBUS_RX_BUF_LEN)
            {
                serial_data->next = 0;
            }
        }
    }
}

// Get a character from the ring buffer, into buf.  Get up to len chars if available.
// Don't block.  Use similar prototype to read, but use mbus_handle instead of int fd for first param
ssize_t mbus_serial_pico_read(mbus_handle *handle, char *buf, ssize_t count)
{
    ssize_t read = 0;
    bool waited;
    mbus_serial_data *serial_data;


    if ((handle == NULL) || (handle->auxdata == NULL))
    {
        return -1;
    }
    serial_data = (mbus_serial_data *)handle->auxdata;
    if (serial_data->rx_buf == NULL)
    {
        read = -1;
        return read;
    }
    mbus_serial_pico_buf_check(serial_data);

    waited = false;
    while ((read < count) && 
           ((serial_data->data_len > 0) || !waited))
    {
        if (serial_data->data_len == 0)
        {
            if (!waited)
            {
                // Naive implementation - just wait the entire allowed time
#ifdef MBUS_SERIAL_DEBUG            
                printf("%s: Pausing for %dms for data\n", __PRETTY_FUNCTION__, PICO_MBUS_RX_WAIT_TIME_MS);
#endif
                sleep_ms(PICO_MBUS_RX_WAIT_TIME_MS);
                waited = true;
            }
        }
        else
        {
#ifdef MBUS_SERIAL_DEBUG            
            printf("%s: There's %d bytes in the rx_buffer, read one out\n", __PRETTY_FUNCTION__, serial_data->data_len);
#endif
            *buf = serial_data->rx_buf[serial_data->first];
            buf++;
            read++;
            serial_data->first++;
            serial_data->data_len--;
            if (serial_data->first >= PICO_MBUS_RX_BUF_LEN)
            {
                serial_data->first = 0;
            }
        }
    }

    return read;
}

// Reads RX data into a ring buffer
void mbus_serial_pico_rx_irq_handler()
{
    if (irq_uart_handle != NULL)
    {
        mbus_serial_data *serial_data = (mbus_serial_data *)irq_uart_handle;
        mbus_serial_pico_read_into_rx_buf(serial_data);
    }
}
#endif

//------------------------------------------------------------------------------
/// Set up a serial connection handle.
//------------------------------------------------------------------------------
int
mbus_serial_connect(mbus_handle *handle)
{
    mbus_serial_data *serial_data;
    const char *device;
#ifndef RPI_PICO
    struct termios *term;
#endif

    if (handle == NULL)
        return -1;

    serial_data = (mbus_serial_data *) handle->auxdata;
#ifndef RPI_PICO
    if (serial_data == NULL || serial_data->device == NULL)
#else
    if (serial_data == NULL || serial_data->device == NULL || serial_data->rx_buf == NULL)
#endif
        return -1;

    device = serial_data->device;
#ifndef RPI_PICO
    term = &(serial_data->t);
#else
    if (!strcmp(device, "UART0"))
    {
        printf("M-Bus: Using %s\n", device);
        serial_data->uart = uart0;
    }
    else if (!strcmp(device, "UART1"))
    {
        printf("M-Bus: Using %s\n", device);
        serial_data->uart = uart1;
    }
    else
    {
        fprintf(stderr, "%s: Invalid Pico UART: %s\n", __PRETTY_FUNCTION__, device);
        return -1;
    }
    mbus_serial_pico_buf_init(serial_data);
    serial_data->block_time_ms = PICO_MBUS_RX_WAIT_TIME_MS;

#endif
    //
    // create the SERIAL connection
    //

#ifndef RPI_PICO
    // Use blocking read and handle it by serial port VMIN/VTIME setting
    if ((handle->fd = open(device, O_RDWR | O_NOCTTY)) < 0)
    {
        fprintf(stderr, "%s: failed to open tty.", __PRETTY_FUNCTION__);
        return -1;
    }

    memset(term, 0, sizeof(*term));
    term->c_cflag |= (CS8|CREAD|CLOCAL);
    term->c_cflag |= PARENB;

    // No received data still OK
    term->c_cc[VMIN] = (cc_t) 0;

    // Wait at most 0.2 sec.Note that it starts after first received byte!!
    // I.e. if CMIN>0 and there are no data we would still wait forever...
    //
    // The specification mentions link layer response timeout this way:
    // The time structure of various link layer communication types is described in EN60870-5-1. The answer time
    // between the end of a master send telegram and the beginning of the response telegram of the slave shall be
    // between 11 bit times and (330 bit times + 50ms).
    //
    // Nowadays the usage of USB to serial adapter is very common, which could
    // result in additional delay of 100 ms in worst case.
    //
    // For 2400Bd this means (330 + 11) / 2400 + 0.15 = 292 ms (added 11 bit periods to receive first byte).
    // I.e. timeout of 0.3s seems appropriate for 2400Bd.

    term->c_cc[VTIME] = (cc_t) 3; // Timeout in 1/10 sec

    cfsetispeed(term, B2400);
    cfsetospeed(term, B2400);

#ifdef MBUS_SERIAL_DEBUG
    printf("%s: t.c_cflag = %x\n", __PRETTY_FUNCTION__, term->c_cflag);
    printf("%s: t.c_oflag = %x\n", __PRETTY_FUNCTION__, term->c_oflag);
    printf("%s: t.c_iflag = %x\n", __PRETTY_FUNCTION__, term->c_iflag);
    printf("%s: t.c_lflag = %x\n", __PRETTY_FUNCTION__, term->c_lflag);
#endif

    tcsetattr(handle->fd, TCSANOW, term);
#else
    uint speed;
    printf("M-Bus: Set baudrate to %d\n", PICO_MBUS_BAUDRATE);
    speed = uart_init(serial_data->uart, PICO_MBUS_BAUDRATE);
    if (speed != PICO_MBUS_BAUDRATE)
    {
        fprintf(stderr, "%s: Failed to open UART: %s %d\n", __PRETTY_FUNCTION__, device, speed);
        uart_deinit(serial_data->uart);
        return -1;
    }
    if (serial_data->uart == uart0)
    {
        handle->fd = 0;
    }
    else
    {
        handle->fd = 1;
    }
    printf("M-Bus: TX Pin: %d\n", PICO_MBUS_UART_TX_PIN);
    printf("M-Bus: RX Pin: %d\n", PICO_MBUS_UART_RX_PIN);
    gpio_set_function(PICO_MBUS_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(PICO_MBUS_UART_RX_PIN, GPIO_FUNC_UART);
    printf("M-Bus: Set flow control: CTS off RTS off\n");
    uart_set_hw_flow(serial_data->uart, false, false);
    printf("M-Bus: UART settings: Data %d Stop %d Parity %d\n",
          PICO_MBUS_UART_DATA_BITS,
          PICO_MBUS_UART_STOP_BITS,
          PICO_MBUS_UART_PARITY);
    uart_set_format(serial_data->uart,
            PICO_MBUS_UART_DATA_BITS,
            PICO_MBUS_UART_STOP_BITS,
            PICO_MBUS_UART_PARITY);

    // As we're using interrupt handling, we disable the FIFO handler
    uart_set_fifo_enabled(serial_data->uart, false);
    int uart_irq = serial_data->uart == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(uart_irq, mbus_serial_pico_rx_irq_handler);
    irq_set_enabled(uart_irq, true);
    uart_set_irq_enables(serial_data->uart, true, false);
    printf("M-Bus: IRQ Handler registered\n");

#endif    

    return 0;
}

//------------------------------------------------------------------------------
// Set baud rate for serial connection
//------------------------------------------------------------------------------
int
mbus_serial_set_baudrate(mbus_handle *handle, long baudrate)
{
#ifndef RPI_PICO    
    speed_t speed;
#else
    uint speed;
#endif
    mbus_serial_data *serial_data;

    if (handle == NULL)
        return -1;

    serial_data = (mbus_serial_data *) handle->auxdata;

    if (serial_data == NULL)
        return -1;

#ifndef RPI_PICO    
    switch (baudrate)
    {
        case 300:
            speed = B300;
            serial_data->t.c_cc[VTIME] = (cc_t) 20; // Timeout in 1/10 sec
            break;

        case 600:
            speed = B600;
            serial_data->t.c_cc[VTIME] = (cc_t) 13;  // Timeout in 1/10 sec
            break;

        case 1200:
            speed = B1200;
            serial_data->t.c_cc[VTIME] = (cc_t) 8;  // Timeout in 1/10 sec
            break;

        case 2400:
            speed = B2400;
            serial_data->t.c_cc[VTIME] = (cc_t) 5;  // Timeout in 1/10 sec
            break;

        case 4800:
            speed = B4800;
            serial_data->t.c_cc[VTIME] = (cc_t) 4;  // Timeout in 1/10 sec
            break;

        case 9600:
            speed = B9600;
            serial_data->t.c_cc[VTIME] = (cc_t) 3;  // Timeout in 1/10 sec
            break;

        case 19200:
            speed = B19200;
            serial_data->t.c_cc[VTIME] = (cc_t) 3;  // Timeout in 1/10 sec
            break;

        case 38400:
            speed = B38400;
            serial_data->t.c_cc[VTIME] = (cc_t) 3;  // Timeout in 1/10 sec
            break;

       default:
            return -1; // unsupported baudrate
    }

    // Set input baud rate
    if (cfsetispeed(&(serial_data->t), speed) != 0)
    {
        return -1;
    }

    // Set output baud rate
    if (cfsetospeed(&(serial_data->t), speed) != 0)
    {
        return -1;
    }

    // Change baud rate immediately
    if (tcsetattr(handle->fd, TCSANOW, &(serial_data->t)) != 0)
    {
        return -1;
    }
#else
    uart_inst_t *uart;
    uart = mbus_serial_pico_get_uart_from_fd(handle->fd);
    if (uart == NULL)
    {
        fprintf(stderr, "%s: Failed to get uart from handle %d\n", __PRETTY_FUNCTION__, handle->fd);
        return -1;
    }
    speed = uart_set_baudrate(uart, (uint)baudrate);
    if (speed != baudrate)
    {
        fprintf(stderr, "%s: Pico UART failed to set baudrate: %d %d %d\n", __PRETTY_FUNCTION__, handle->fd, baudrate, speed);
        return -1;
    }
#endif

    return 0;
}


//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int
mbus_serial_disconnect(mbus_handle *handle)
{
    if (handle == NULL)
    {
        return -1;
    }

#ifndef RPI_PICO    
    if (handle->fd < 0)
    {
       return -1;
    }

    close(handle->fd);
    handle->fd = -1;
#else
    uart_inst_t *uart;
    uart = mbus_serial_pico_get_uart_from_fd(handle->fd);
    if (uart == NULL)
    {
        return -1;
    }
    uart_deinit(uart);
#endif

    return 0;
}

void
mbus_serial_data_free(mbus_handle *handle)
{
    mbus_serial_data *serial_data;

    if (handle)
    {
        serial_data = (mbus_serial_data *) handle->auxdata;

        if (serial_data == NULL)
        {
            return;
        }

        free(serial_data->device);
        free(serial_data);
        handle->auxdata = NULL;
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int
mbus_serial_send_frame(mbus_handle *handle, mbus_frame *frame)
{
    unsigned char buff[PACKET_BUFF_SIZE];
    int len, ret;

    if (handle == NULL || frame == NULL)
    {
        return -1;
    }

#ifndef RPI_PICO
    // Make sure serial connection is open
    if (isatty(handle->fd) == 0)
    {
        return -1;
    }
#else
    uart_inst_t *uart;
    uart = mbus_serial_pico_get_uart_from_fd(handle->fd);
    if ((uart == NULL) || (!uart_is_enabled(uart)))
    {
        fprintf(stderr, "%s: Invalid UART or UART not enabled: %d\n", __PRETTY_FUNCTION__, handle->fd);
        return -1;
    }
#endif

    if ((len = mbus_frame_pack(frame, buff, sizeof(buff))) == -1)
    {
        fprintf(stderr, "%s: mbus_frame_pack failed\n", __PRETTY_FUNCTION__);
        return -1;
    }

#ifdef MBUS_SERIAL_DEBUG
    // if debug, dump in HEX form to stdout what we write to the serial port
    printf("%s: Dumping M-Bus frame [%d bytes]: ", __PRETTY_FUNCTION__, len);
    int i;
    for (i = 0; i < len; i++)
    {
       printf("%.2X ", buff[i]);
    }
    printf("\n");
#endif

#ifndef RPI_PICO
    if ((ret = write(handle->fd, buff, len)) == len)
    {
        //
        // call the send event function, if the callback function is registered
        //
        if (handle->send_event)
                handle->send_event(MBUS_HANDLE_TYPE_SERIAL, buff, len);
    }
    else
    {
        fprintf(stderr, "%s: Failed to write frame to socket (ret = %d: %s)\n", __PRETTY_FUNCTION__, ret, strerror(errno));
        return -1;
    }

    //
    // wait until complete frame has been transmitted
    //
    tcdrain(handle->fd);
#else
#ifdef MBUS_SERIAL_DEBUG
    printf("M-Bus: Transmitting %d bytes\n", len);
#endif
    uart_write_blocking(uart, buff, len);
#endif    

    return 0;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int
mbus_serial_recv_frame(mbus_handle *handle, mbus_frame *frame)
{
    char buff[PACKET_BUFF_SIZE];
    int remaining, timeouts;
    ssize_t len, nread;

    if (handle == NULL || frame == NULL)
    {
        fprintf(stderr, "%s: Invalid parameter.\n", __PRETTY_FUNCTION__);
        return MBUS_RECV_RESULT_ERROR;
    }

#ifndef RPI_PICO
    // Make sure serial connection is open
    if (isatty(handle->fd) == 0)
    {
        fprintf(stderr, "%s: Serial connection is not available.\n", __PRETTY_FUNCTION__);
        return MBUS_RECV_RESULT_ERROR;
    }
#else
    uart_inst_t *uart;
    uart = mbus_serial_pico_get_uart_from_fd(handle->fd);
    if ((uart == NULL) || (!uart_is_enabled(uart)))
    {
        fprintf(stderr, "%s: Invalid UART or UART not enabled: %d", __PRETTY_FUNCTION__, handle->fd);
        return MBUS_RECV_RESULT_ERROR;
    }
#endif

    memset((void *)buff, 0, sizeof(buff));

    //
    // read data until a packet is received
    //
    remaining = 1; // start by reading 1 byte
    len = 0;
    timeouts = 0;

    do {
        if (len + remaining > PACKET_BUFF_SIZE)
        {
            // avoid out of bounds access
            return MBUS_RECV_RESULT_ERROR;
        }

#ifdef MBUS_SERIAL_DEBUG
        printf("%s: Attempt to read %d bytes [len = %d]\n", __PRETTY_FUNCTION__, remaining, len);
#endif

#ifndef RPI_PICO
        if ((nread = read(handle->fd, &buff[len], remaining)) == -1)
#else
        if ((nread = mbus_serial_pico_read(handle, &buff[len], remaining)) == -1)
#endif
        {
#ifdef MBUS_SERIAL_DEBUG
            fprintf(stderr, "%s: aborting recv frame (remaining = %d, len = %d, nread = %d)\n",
                   __PRETTY_FUNCTION__, remaining, len, nread);
#endif                   
            return MBUS_RECV_RESULT_ERROR;
        }

#ifdef MBUS_SERIAL_DEBUG
        printf("%s: Got %d byte [remaining %d, len %d]\n", __PRETTY_FUNCTION__, nread, remaining, len);
#endif

        if (nread == 0)
        {
            timeouts++;

            if (timeouts >= 3)
            {
                // abort to avoid endless loop
                fprintf(stderr, "%s: Timeout\n", __PRETTY_FUNCTION__);
                break;
            }
        }

        if (len > (SSIZE_MAX-nread))
        {
            // avoid overflow
            return MBUS_RECV_RESULT_ERROR;
        }

        len += nread;

    } while ((remaining = mbus_parse(frame, buff, len)) > 0);

    if (len == 0)
    {
        // No data received
        return MBUS_RECV_RESULT_TIMEOUT;
    }

    //
    // call the receive event function, if the callback function is registered
    //
    if (handle->recv_event)
        handle->recv_event(MBUS_HANDLE_TYPE_SERIAL, buff, len);

    if (remaining != 0)
    {
        // Would be OK when e.g. scanning the bus, otherwise it is a failure.
#ifdef MBUS_SERIAL_DEBUG
        printf("%s: M-Bus layer failed to receive complete data (OK when scanning bus)\n", __PRETTY_FUNCTION__);
#endif
        return MBUS_RECV_RESULT_INVALID;
    }

    if (len == -1)
    {
        fprintf(stderr, "%s: M-Bus layer failed to parse data.\n", __PRETTY_FUNCTION__);
        return MBUS_RECV_RESULT_ERROR;
    }

    return MBUS_RECV_RESULT_OK;
}

#ifdef RPI_PICO
uart_inst_t *mbus_serial_pico_get_uart_from_fd(int fd)
{
    uart_inst_t *uart;
    if (fd == 0)
    {
#ifdef MBUS_SERIAL_DEBUG
        printf("M-Bus: Retreived UART0\n");
#endif
        uart = uart0;
    }
    else if (fd == 1)
    {
#ifdef MBUS_SERIAL_DEBUG
        printf("M-Bus: Retreived UART1\n");
#endif
        uart = uart1;
    }
    else
    {
        //fprintf(stderr, "%s: Pico UART fd is invalid: %d\n", __PRETTY_FUNCTION__, fd);
        uart = NULL;
    }
    return uart;
}
#endif
