//------------------------------------------------------------------------------
// Copyright (C) 2011, Robert Johansson, Raditex AB
// All rights reserved.
//
// rSCADA
// http://www.rSCADA.se
// info@rscada.se
//
//------------------------------------------------------------------------------

//#include <unistd.h>
#include <limits.h>
#include <fcntl.h>

#include <sys/types.h>

#include <stdio.h>
#include <strings.h>

//#include <termios.h>
#include <errno.h>
#include <string.h>

#include "mbus.h"
#include "mbus-serial.h"
#include "mbus-protocol-aux.h"
#include "mbus-protocol.h"

//------------------------------------------------------------------------------
/// Set up a serial connection handle.
//------------------------------------------------------------------------------
int ICACHE_FLASH_ATTR 
mbus_serial_connect(mbus_handle *handle)
{
    mbus_serial_data *serial_data;
    Softuart *softuart;
    
    if (handle == NULL)
        return -1;

    serial_data = (mbus_serial_data *) handle->auxdata;
    if (serial_data == NULL)
        return -1;

    softuart = &(serial_data->softuart);
    serial_data->rx_pin = 4;
    serial_data->tx_pin = 5;
    Softuart_SetPinRx(softuart, serial_data->rx_pin);
    Softuart_SetPinTx(softuart, serial_data->tx_pin);
    Softuart_Init(softuart, serial_data->baudrate);
    serial_data->inited = 1;

#if 0
    mbus_serial_data *serial_data;
    const char *device;
    struct termios *term;

    if (handle == NULL)
        return -1;

    serial_data = (mbus_serial_data *) handle->auxdata;
    if (serial_data == NULL || serial_data->device == NULL)
        return -1;

    device = serial_data->device;
    term = &(serial_data->t);
    //
    // create the SERIAL connection
    //
    
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
    // For 2400Bd this means (330 + 11) / 2400 + 0.05 = 188.75 ms (added 11 bit periods to receive first byte).
    // I.e. timeout of 0.2s seems appropriate for 2400Bd.

    term->c_cc[VTIME] = (cc_t) 2; // Timeout in 1/10 sec

    cfsetispeed(term, B2400);
    cfsetospeed(term, B2400);

#ifdef MBUS_SERIAL_DEBUG
    printf("%s: t.c_cflag = %x\n", __PRETTY_FUNCTION__, term->c_cflag);
    printf("%s: t.c_oflag = %x\n", __PRETTY_FUNCTION__, term->c_oflag);
    printf("%s: t.c_iflag = %x\n", __PRETTY_FUNCTION__, term->c_iflag);
    printf("%s: t.c_lflag = %x\n", __PRETTY_FUNCTION__, term->c_lflag);
#endif

    tcsetattr(handle->fd, TCSANOW, term);
#endif

    return 0;
}

//------------------------------------------------------------------------------
// Set baud rate for serial connection
//------------------------------------------------------------------------------
int ICACHE_FLASH_ATTR 
mbus_serial_set_baudrate(mbus_handle *handle, long baudrate)
{
    mbus_serial_data *serial_data;

    if (handle == NULL)
        return -1;

    serial_data = (mbus_serial_data *) handle->auxdata;

    if (serial_data == NULL)
        return -1;
        
    if (serial_data->inited)
    {
      return -1;
    }

    serial_data->baudrate = baudrate;

    return 0;
}


//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int ICACHE_FLASH_ATTR 
mbus_serial_disconnect(mbus_handle *handle)
{
    mbus_serial_data *serial_data;

    if (handle == NULL)
    {
        return -1;
    }

    serial_data = (mbus_serial_data *) handle->auxdata;
    if (serial_data == NULL)
        return -1;

    serial_data->inited = 0;

    return 0;
}

void ICACHE_FLASH_ATTR 
mbus_serial_data_free(mbus_handle *handle)
{
  // No op?
#if 0
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
#endif
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int ICACHE_FLASH_ATTR 
mbus_serial_send_frame(mbus_handle *handle, mbus_frame *frame)
{
    unsigned char buff[PACKET_BUFF_SIZE];
    int len, ret;
    mbus_serial_data *serial_data;
    Softuart *softuart;
    int ii;

    if (handle == NULL || frame == NULL)
    {
        return -1;
    }

    serial_data = (mbus_serial_data *) handle->auxdata;
    if (serial_data == NULL)
        return -1;

    if (!serial_data->inited)
    {
      return -1;
    }

    softuart = &(serial_data->softuart);

    if ((len = mbus_frame_pack(frame, buff, sizeof(buff))) == -1)
    {
        WARN("%s: mbus_frame_pack failed", __PRETTY_FUNCTION__);
        return -1;
    }
    
    // XXX Do bit banging of frame at hardcoded speed for now
    for (ii = 0; ii < len; ii++)
    {
      Softuart_Putchar(softuart, buff[ii]);
    }
    
    DEBUG("%s: Sent %d bytes", __PRETTY_FUNCTION__, ii);

#if 0
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
#endif

    return 0;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int ICACHE_FLASH_ATTR 
mbus_serial_recv_frame(mbus_handle *handle, mbus_frame *frame)
{
    char buff[PACKET_BUFF_SIZE];
    int remaining, timeouts;
    ssize_t len, nread;
    mbus_serial_data *serial_data;
    Softuart *softuart;
    
    if (handle == NULL || frame == NULL)
    {
        return -1;
    }

    serial_data = (mbus_serial_data *) handle->auxdata;
    if (serial_data == NULL)
        return -1;

    if (!serial_data->inited)
    {
      return -1;
    }

    softuart = &(serial_data->softuart);

    memset((void *)buff, 0, sizeof(buff));

    remaining = 1;
    len = 0;
    timeouts = 0;

    while (remaining > 0)
    {
        if (len > PACKET_BUFF_SIZE)
        {
            // avoid out of bounds access
            WARN("Got too many bytes");
            return MBUS_RECV_RESULT_ERROR;
        }
        
        if (Softuart_Available(softuart))
        {
            buff[len] = Softuart_Read(softuart);
            INFO("Got byte: 0x%02x", buff[len]);
            len++;
            remaining--;
        }
        else
        {
            timeouts++;
            if (timeouts >= 3)
            {
                // abort to avoid endless loop
                DEBUG("%s: Timeout", __PRETTY_FUNCTION__);
                break;
            }
        }
        if (remaining == 0)
        {
            remaining = mbus_parse(frame, buff, len);
            INFO("Still want %d bytes", remaining);
        }
    }
    
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
        // printf("%s: M-Bus layer failed to receive complete data.\n", __PRETTY_FUNCTION__);
        return MBUS_RECV_RESULT_INVALID;
    }

    return MBUS_RECV_RESULT_OK;
}

