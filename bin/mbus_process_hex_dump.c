//------------------------------------------------------------------------------
// Copyright (C) 2011, Robert Johansson, Raditex AB
// All rights reserved.
//
// rSCADA
// http://www.rSCADA.se
// info@rscada.se
//
//------------------------------------------------------------------------------

#include <string.h>

#include <stdio.h>
#include <mbus/mbus.h>

static int debug = 0;

//------------------------------------------------------------------------------
// Scan for devices using secondary addressing.
//------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
    mbus_frame reply;
    mbus_frame_data reply_data;
    mbus_handle *handle = NULL;

    char *input;
    char hex_byte[3];
    char buff[256];
    long val;
    int ii;
    int len;
    char *xml_result;
    int rem;

    memset((void *)&reply, 0, sizeof(mbus_frame));
    memset((void *)&reply_data, 0, sizeof(mbus_frame_data));

    if (argc == 2)
    {
      input = argv[1];
    }
    else if (argc == 3 && strcmp(argv[1], "-d") == 0)
    {
      input = argv[2];
      debug = 1;
    }
    else
    {
        fprintf(stderr, "usage: %s hex input string\n", argv[0]);
        return 0;
    }
    
    // Read input string into
    hex_byte[2] = 0;
    len = 0;
    for (ii = 0; ii < (strlen(input)/2); ii++)
    {
      hex_byte[0] = input[(ii*2)];
      hex_byte[1] = input[(ii*2)+1];
      val = strtol(hex_byte, NULL, 16);
      buff[ii] = (unsigned char)val;
      if (debug)
      {
        printf("Byte: %d, Value: 0x%02x\n", ii, buff[ii]);
      }
      len++;
    }
    
    // XXX Should process rc
    rem = mbus_parse(&reply, buff, len);
    printf("mbus_parse says %d bytes left\n", rem);

    //
    // dump hex data if debug is true
    //
    if (debug)
    {
        mbus_frame_print(&reply);
    }

    //
    // parse data
    //
    if (mbus_frame_data_parse(&reply, &reply_data) == -1)
    {
        fprintf(stderr, "M-bus data parse error: %s\n", mbus_error_str());
        return 1;
    }

    //
    // generate XML and print to standard output
    //
    if ((xml_result = mbus_frame_data_xml(&reply_data)) == NULL)
    {
        fprintf(stderr, "Failed to generate XML representation of MBUS frame: %s\n", mbus_error_str());
        return 1;
    }

    printf("%s", xml_result);
    free(xml_result);

    // manual free
    if (reply_data.data_var.record)
    {
        mbus_data_record_free(reply_data.data_var.record); // free's up the whole list
    }

    return 0;
}



