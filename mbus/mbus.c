//------------------------------------------------------------------------------
// Copyright (C) 2010, Raditex AB
// All rights reserved.
//
// rSCADA
// http://www.rSCADA.se
// info@rscada.se
//
//------------------------------------------------------------------------------

#include "mbus.h"
#include "mbus-protocol.h"
#include "../config.h"

//
//
//

static mbus_handle *my_handle;
void ICACHE_FLASH_ATTR mbus_init()
{
  int rc;
  int ii;
  my_handle = mbus_context_serial("");
  mbus_frame reply;
  mbus_frame_data reply_data;
  char *xml;
  
  rc = mbus_serial_set_baudrate(my_handle, 2400);
  if (rc)
  {
    ERROR("mbus: hit error setting baudrate");
    goto EXIT_LABEL;
  }
  rc = mbus_serial_connect(my_handle);
  if (rc)
  {
    ERROR("mbus: hit error connecting");
    goto EXIT_LABEL;
  }
  
  INFO("mbus: send ping frame");
  rc = mbus_send_ping_frame(my_handle, MBUS_ADDRESS_NETWORK_LAYER, 1);
  if (rc)
  {
    ERROR("mbus: hit error sending ping frame 1");
    goto EXIT_LABEL;
  }

  os_delay_us(200000);

  INFO("mbus: send ping frame");
  rc = mbus_send_ping_frame(my_handle, MBUS_ADDRESS_NETWORK_LAYER, 1);
  if (rc)
  {
    ERROR("mbus: hit error sending ping frame 2");
    goto EXIT_LABEL;
  }
  
  os_delay_us(200000);
  
  INFO("mbus: send request frame");
  rc = mbus_send_request_frame(my_handle, 1);
  if (rc)
  {
    ERROR("mbus: hit error sending request frame");
    goto EXIT_LABEL;
  }
  
  INFO("mbus: wait for response");
  // Give device 1s to get it's act together!  At 2400 baud that's time for 300 bytes.
  // Am expecting ITRO 100 bytes.
  os_delay_us(1000000);
  
  rc = mbus_recv_frame(my_handle, &reply);
  
  if (rc)
  {
    ERROR("mbus: receive frame return code: %d", rc);
    goto EXIT_LABEL;
  }

  mbus_frame_print(&reply);

  rc = mbus_frame_data_parse(&reply, &reply_data);
  if (rc)
  {
    ERROR("mbus: frame data parse return code: %d", rc);
    goto EXIT_LABEL;
  }
  
  xml = mbus_frame_data_xml(&reply_data);
  if (xml == NULL)
  {
    ERROR("mbus: parsing into xml failed");
    goto EXIT_LABEL;
  }
  
  ets_printf("XML Result:\r\n%s\r\n", xml);
  os_free(xml);
  mbus_data_record_free(reply_data.data_var.record);
  mbus_disconnect(my_handle);
  mbus_context_free(my_handle);
  
  
EXIT_LABEL:

  return;
}

///
/// Return current version of the library
///
const char*
ICACHE_FLASH_ATTR mbus_get_current_version() {return VERSION;}

