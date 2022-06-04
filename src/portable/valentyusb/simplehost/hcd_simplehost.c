/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Marcus Comstedt
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if CFG_TUH_ENABLED && (CFG_TUSB_MCU == OPT_MCU_VALENTYUSB_SIMPLEHOST)

#ifndef DEBUG
#define DEBUG 0
#endif

#ifndef LOG_USB
#define LOG_USB 0
#endif

#include "host/hcd.h"
#include "host/usbh.h"
#include "hcd_simplehost.h"
#include "csr.h"
#include "irq.h"

#define ROOT_PORT 0


static uint8_t settle_cnt, reset_cnt;
static volatile uint32_t frame_number;

static uint8_t *in_xfer_buffer;
static uint16_t in_xfer_buflen;
static uint8_t *out_xfer_buffer;
static uint16_t out_xfer_buflen;
static uint32_t xfer_cmd;
static uint32_t xfer_len;
static uint8_t xfer_timeout;
static uint16_t max_xfer = 8;

static bool need_pre(uint8_t dev_addr)
{
    return hcd_port_speed_get(0) != tuh_speed_get(dev_addr);
}

static void retransmit_xfer(void)
{
    if (out_xfer_buflen) {
        unsigned i, cnt = out_xfer_buflen > max_xfer? max_xfer : out_xfer_buflen;
	for (i=0; i<cnt; i++)
	  usb_transfer_data_out_write(out_xfer_buffer[i]);
    }

    usb_transfer_cmd_write(xfer_cmd);
}

static void complete_xfer(xfer_result_t xfer_result)
{
    hcd_event_xfer_complete(usb_transfer_cmd_addr_extract(xfer_cmd),
			    tu_edpt_addr(usb_transfer_cmd_epno_extract(xfer_cmd),
					 usb_transfer_cmd_in_extract(xfer_cmd)),
			    xfer_len, xfer_result, true);
    xfer_cmd = 0;
}


//--------------------------------------------------------------------+
// HCD API
//--------------------------------------------------------------------+
bool hcd_init(uint8_t rhport)
{
    (void)rhport;
    usb_sof_ev_enable_write(0);
    usb_transfer_ev_enable_write(0);
    usb_pullup_detect_ev_enable_write(0);
    printf("hcd_init %d\n", rhport);

    usb_ctrl_sof_enable_write(0);
    settle_cnt = 0;
    reset_cnt = 0;
    in_xfer_buffer = NULL;
    in_xfer_buflen = 0;
    out_xfer_buffer = NULL;
    out_xfer_buflen = 0;
    frame_number = usb_sof_frame_read();
    xfer_cmd = 0;

    usb_pullup_detect_ev_enable_write(1);
    usb_transfer_ev_enable_write(0x1f);
    usb_sof_ev_enable_write(1);

    return true;
}

void hcd_port_reset(uint8_t rhport)
{
    (void)rhport;
    printf("hcd_port_reset\n");
    usb_ctrl_sof_enable_write(0);
    usb_ctrl_reset_write(1);
    reset_cnt = 12;
    in_xfer_buffer = NULL;
    in_xfer_buflen = 0;
    out_xfer_buffer = NULL;
    out_xfer_buflen = 0;
    xfer_cmd = 0;
}

bool hcd_port_connect_status(uint8_t rhport)
{
    (void)rhport;
    return usb_pullup_detect_pullup_read() != 0;
}

tusb_speed_t hcd_port_speed_get(uint8_t rhport)
{
    (void)rhport;
    return (usb_ctrl_low_speed_read()? TUSB_SPEED_LOW : TUSB_SPEED_FULL);
}


// Close all opened endpoint belong to this device
void hcd_device_close(uint8_t rhport, uint8_t dev_addr)
{
  printf("hcd_device_close %d\n", dev_addr);
  (void) rhport;

  if (dev_addr == 0) return;
}

uint32_t hcd_frame_number(uint8_t rhport)
{
    (void) rhport;
    return frame_number;
}

void hcd_int_enable(uint8_t rhport)
{
    (void)rhport;
    // printf("hcd_int_enable\n");
	irq_setmask(irq_getmask() | (1 << USB_INTERRUPT));
}

void hcd_int_disable(uint8_t rhport)
{
    (void)rhport;
    // printf("hcd_int_disable\n");
  irq_setmask(irq_getmask() & ~(1 << USB_INTERRUPT));
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

bool hcd_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const * ep_desc)
{
    (void) rhport;

    max_xfer = tu_edpt_packet_size(ep_desc);

    printf("hcd_edpt_open dev_addr %d, ep_addr %d, max_xfer %u\n", dev_addr, ep_desc->bEndpointAddress, max_xfer);

    return true;
}

bool hcd_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t * buffer, uint16_t buflen)
{
    (void) rhport;
    (void) dev_addr;
    (void) ep_addr;
    (void) buffer;
    (void) buflen;

    // printf("hcd_edpt_xfer dev_addr %d, ep_addr 0x%x, len %d\n", dev_addr, ep_addr, buflen);
    
    tusb_dir_t const ep_dir = tu_edpt_dir(ep_addr);  // true -> in

    if (ep_dir) {
      in_xfer_buffer = buffer;
      in_xfer_buflen = (buffer? buflen : 0);
      out_xfer_buffer = NULL;
      out_xfer_buflen = 0;
    } else {
      out_xfer_buffer = buffer;
      out_xfer_buflen = (buffer? buflen : 0);
      in_xfer_buffer = NULL;
      in_xfer_buflen = 0;
    }

    xfer_cmd = (dev_addr << CSR_USB_TRANSFER_CMD_ADDR_OFFSET) |
      (ep_addr << CSR_USB_TRANSFER_CMD_EPNO_OFFSET) |
      (ep_dir?
       (1U << CSR_USB_TRANSFER_CMD_IN_OFFSET) :
       (1U << CSR_USB_TRANSFER_CMD_OUT_OFFSET)) |
      (1U << CSR_USB_TRANSFER_CMD_DATA1_OFFSET) |
      (need_pre(dev_addr) << CSR_USB_TRANSFER_CMD_PRE_OFFSET);
    xfer_len = 0;
    xfer_timeout = 100;

    retransmit_xfer();

    return true;
}

bool hcd_setup_send(uint8_t rhport, uint8_t dev_addr, uint8_t const setup_packet[8])
{
    unsigned i;

    (void) rhport;

    // printf("hcd_setup_send(%u, %u, ...)\n", (unsigned)rhport, (unsigned)dev_addr);

    for (i=0; i<8; i++)
        usb_transfer_data_out_write(setup_packet[i]);

    xfer_cmd = (dev_addr << CSR_USB_TRANSFER_CMD_ADDR_OFFSET) |
      (1U << CSR_USB_TRANSFER_CMD_SETUP_OFFSET) |
      (need_pre(dev_addr) << CSR_USB_TRANSFER_CMD_PRE_OFFSET);
    xfer_len = 0;
    xfer_timeout = 100;

    usb_transfer_cmd_write(xfer_cmd);
    
    return true;
}

//--------------------------------------------------------------------+
// ISR
//--------------------------------------------------------------------+

static xfer_result_t handle_data_out_ack(void)
{
  if (out_xfer_buflen > max_xfer) {
    out_xfer_buflen -= max_xfer;
    out_xfer_buffer += max_xfer;
    xfer_len += max_xfer;
    /* Send next data chunk */
    xfer_cmd ^= (1U << CSR_USB_TRANSFER_CMD_DATA1_OFFSET);
    retransmit_xfer();
    return XFER_RESULT_INVALID;
  }

  xfer_len += out_xfer_buflen;
  return XFER_RESULT_SUCCESS;
}

static xfer_result_t handle_data_in(bool dtb)
{
  (void) dtb;

  if (!usb_transfer_cmd_in_extract(xfer_cmd)
      /* || dtb != usb_transfer_cmd_data1_extract(xfer_cmd) */)
    return XFER_RESULT_INVALID;

  uint32_t cnt = 0;
  uint32_t bcnt = 0;

  while (usb_transfer_status_have_read()) {
    uint8_t byte = usb_transfer_data_in_read();
    ++cnt;
    if (in_xfer_buflen) {
      ++bcnt;
      *in_xfer_buffer++ = byte;
      --in_xfer_buflen;
    }
  }

  /* Remove CRC */
  while (bcnt+2 > cnt) {
    --in_xfer_buffer;
    in_xfer_buflen++;
    --bcnt;
  }

  xfer_len += bcnt;

  if (!in_xfer_buflen || cnt < max_xfer+2u)
    return XFER_RESULT_SUCCESS;

  /* Request next data chunk */
  xfer_cmd ^= (1U << CSR_USB_TRANSFER_CMD_DATA1_OFFSET);
  retransmit_xfer();
  return XFER_RESULT_INVALID;
}

static void handle_pullup_change(uint32_t ev)
{
  if (!usb_pullup_detect_ev_pending_pullup_change_extract(ev))
    return;
  reset_cnt = 0;
  uint32_t ctrl = usb_ctrl_read(), pullup = usb_pullup_detect_pullup_read();
  ctrl = usb_ctrl_reset_replace(ctrl, 0);
  ctrl = usb_ctrl_sof_enable_replace(ctrl, 0);
  if(pullup) {
    settle_cnt = 100;
    if (usb_pullup_detect_pullup_k_extract(pullup))
      /* A pullup to state K means we need to switch root port speed */
      ctrl = usb_ctrl_low_speed_replace(ctrl, !usb_ctrl_low_speed_extract(ctrl));
  } else {
    settle_cnt = 0;
    hcd_event_device_remove(ROOT_PORT, true);
  }
  usb_ctrl_write(ctrl);
}

static void handle_sof(uint32_t ev)
{
  if (!usb_sof_ev_pending_new_frame_extract(ev))
    return;
  uint32_t f = usb_sof_frame_read();
  if (f < (frame_number & 0x7ff)) {
    /* Wrap */
    frame_number = ((frame_number & ~0x7ff) + 0x800) | f;
  } else {
    frame_number = (frame_number & ~0x7ff) | f;
  }
  if (settle_cnt) {
    if (!--settle_cnt) {
      usb_ctrl_reset_write(1);
      reset_cnt = (usb_ctrl_low_speed_read()? 450 : 12);
      hcd_event_device_attach(ROOT_PORT, true);
    }
  } else if(reset_cnt) {
    if (!--reset_cnt) {
      usb_ctrl_reset_write(0);
      usb_ctrl_sof_enable_write(1);
    }
  } else if(xfer_cmd) {
    if (xfer_timeout == 0xff) {
      /* NAK -> resend */
      retransmit_xfer();
      xfer_timeout = 100;
    } else if(!--xfer_timeout)
      complete_xfer(XFER_RESULT_TIMEOUT);
  }
}

static void handle_transfer(uint32_t ev)
{
  xfer_result_t xfer_result = XFER_RESULT_INVALID;
  if (!xfer_cmd)
    return;
  if (usb_transfer_ev_pending_stall_extract(ev))
    xfer_result = XFER_RESULT_STALLED;
  else if (usb_transfer_ev_pending_ack_extract(ev))
    xfer_result = (usb_transfer_cmd_out_extract(xfer_cmd)?
		   handle_data_out_ack() : XFER_RESULT_SUCCESS);
  else if (usb_transfer_ev_pending_nak_extract(ev)) {
    if (!usb_transfer_cmd_setup_extract(xfer_cmd))
      xfer_timeout = 0xff; /* Retransmit next frame */
  } else if (usb_transfer_ev_pending_data0_extract(ev)) {
    xfer_result = handle_data_in(0);
  } else if (usb_transfer_ev_pending_data1_extract(ev)) {
    xfer_result = handle_data_in(1);
  }

  if (xfer_result != XFER_RESULT_INVALID)
    complete_xfer(xfer_result);
}

void hcd_int_handler(uint8_t rhport)
{
  (void)rhport;
  uint32_t r;
  if ((r = usb_pullup_detect_ev_pending_read())) {
    usb_pullup_detect_ev_pending_write(r);
    handle_pullup_change(r);
  }
  if ((r = usb_transfer_ev_pending_read())) {
    usb_transfer_ev_pending_write(r);
    handle_transfer(r);
  }
  if ((r = usb_sof_ev_pending_read())) {
    usb_sof_ev_pending_write(r);
    handle_sof(r);
  }
}
#endif
