/******************************************************************************
 * Copyright 2020-2021 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <firmament.h>

#include "drv_usbd_cdc.h"
#include "hal/usb/usbd_cdc.h"
#include "module/utils/ringbuffer.h"

#include "ch32h417_usbfs_device.h"

#define USBFS_CDC_MAX_PACKET_SIZE  DEF_USBD_FS_PACK_SIZE
#define USBFS_CDC_TX_FIFO_SIZE     USBD_CDC_TX_BUFSIZE
#define USBFS_CDC_RX_RESUME_LEVEL  (USBD_CDC_RX_BUFSIZE - USBFS_CDC_MAX_PACKET_SIZE - 1)
#define USBFS_CDC_LINE_CODING_SIZE 7

#define CDC_GET_LINE_CODING        0x21
#define CDC_SET_LINE_CODING        0x20
#define CDC_SET_CONTROL_LINE_STATE 0x22
#define CDC_SEND_BREAK             0x23

static struct usbd_cdc_dev usbd_dev = { 0 };
static ringbuffer* tx_rb = RT_NULL;
static uint8_t rx_buffer[USBFS_CDC_MAX_PACKET_SIZE] __attribute__((aligned(4)));
static uint8_t tx_buffer[USBFS_CDC_MAX_PACKET_SIZE] __attribute__((aligned(4)));
static uint8_t line_coding[USBFS_CDC_LINE_CODING_SIZE] = {
    0x00,
    0xC2,
    0x01,
    0x00,
    0x00,
    0x00,
    0x08,
};
static volatile rt_bool_t tx_active = RT_FALSE;
static volatile rt_bool_t tx_need_zlp = RT_FALSE;
static volatile rt_bool_t rx_stopped = RT_FALSE;
static volatile rt_bool_t set_address_pending = RT_FALSE;
static volatile uint16_t control_line_state = 0;

const uint8_t* pUSBFS_Descr;

volatile uint8_t USBFS_SetupReqCode;
volatile uint8_t USBFS_SetupReqType;
volatile uint16_t USBFS_SetupReqValue;
volatile uint16_t USBFS_SetupReqIndex;
volatile uint16_t USBFS_SetupReqLen;

volatile uint8_t USBFS_DevConfig;
volatile uint8_t USBFS_DevAddr;
volatile uint8_t USBFS_DevSleepStatus;
volatile uint8_t USBFS_DevEnumStatus;

__attribute__((aligned(4))) uint8_t USBFS_EP0_Buf[DEF_USBD_UEP0_SIZE];
__attribute__((aligned(4))) uint8_t USBFS_EP1_Buf[DEF_USBD_ENDP1_SIZE];
__attribute__((aligned(4))) uint8_t USBFS_EP2_Buf[DEF_USBD_ENDP2_SIZE];
__attribute__((aligned(4))) uint8_t USBFS_EP3_Buf[DEF_USBD_ENDP3_SIZE];

static void usbfs_cdc_arm_rx(void)
{
    USBFSD->UEP2_RX_CTRL = (USBFSD->UEP2_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
    rx_stopped = RT_FALSE;
}

static void usbfs_cdc_stop_rx(void)
{
    USBFSD->UEP2_RX_CTRL = (USBFSD->UEP2_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
    rx_stopped = RT_TRUE;
}

static void usbfs_cdc_maybe_resume_rx(usbd_cdc_dev_t usbd)
{
    if (rx_stopped == RT_TRUE &&
        usbd->rx_rb != RT_NULL &&
        ringbuffer_getlen(usbd->rx_rb) < USBFS_CDC_RX_RESUME_LEVEL) {
        usbfs_cdc_arm_rx();
    }
}

static uint16_t usbfs_min_u16(uint16_t a, uint16_t b)
{
    return (a < b) ? a : b;
}

static void usbfs_cdc_kick_tx_locked(void)
{
    rt_size_t send_len;

    if (tx_active == RT_TRUE) {
        return;
    }

    send_len = ringbuffer_getlen(tx_rb);
    if (send_len == 0) {
        if (tx_need_zlp == RT_TRUE) {
            tx_need_zlp = RT_FALSE;
            tx_active = RT_TRUE;
            USBFSD->UEP3_TX_LEN = 0;
            USBFSD->UEP3_TX_CTRL = (USBFSD->UEP3_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
        }
        return;
    }

    if (send_len > sizeof(tx_buffer)) {
        send_len = sizeof(tx_buffer);
    }

    (void)ringbuffer_get(tx_rb, tx_buffer, send_len);
    memcpy(USBFS_EP3_Buf, tx_buffer, send_len);
    USBFSD->UEP3_TX_LEN = (uint16_t)send_len;
    USBFSD->UEP3_TX_CTRL = (USBFSD->UEP3_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
    tx_active = RT_TRUE;

    if ((send_len == USBFS_CDC_MAX_PACKET_SIZE) && (ringbuffer_getlen(tx_rb) == 0)) {
        tx_need_zlp = RT_TRUE;
    }
}

static void usbfs_cdc_complete_tx(void)
{
    rt_base_t level = rt_hw_interrupt_disable();

    USBFSD->UEP3_TX_CTRL ^= USBFS_UEP_T_TOG;
    USBFSD->UEP3_TX_CTRL = (USBFSD->UEP3_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;

    tx_active = RT_FALSE;
    usbfs_cdc_kick_tx_locked();

    if (tx_active == RT_FALSE && tx_need_zlp == RT_FALSE) {
        hal_usbd_cdc_notify_status(&usbd_dev, USBD_STATUS_TX_COMPLETE);
    }

    rt_hw_interrupt_enable(level);
}

static rt_size_t usbd_cdc_read(usbd_cdc_dev_t usbd, rt_off_t pos, void* buf, rt_size_t size)
{
    rt_size_t rb;

    RT_ASSERT(usbd->rx_rb != RT_NULL);

    (void)pos;

    rb = ringbuffer_get(usbd->rx_rb, buf, size);
    usbfs_cdc_maybe_resume_rx(usbd);

    return rb;
}

static rt_size_t usbd_cdc_write(usbd_cdc_dev_t usbd, rt_off_t pos, const void* buf, rt_size_t size)
{
    rt_size_t wb;
    rt_base_t level;

    (void)usbd;
    (void)pos;

    if (tx_rb == RT_NULL) {
        return 0;
    }

    level = rt_hw_interrupt_disable();
    wb = ringbuffer_put(tx_rb, buf, size);
    if ((wb > 0) && ((wb % USBFS_CDC_MAX_PACKET_SIZE) == 0)) {
        tx_need_zlp = RT_TRUE;
    }
    usbfs_cdc_kick_tx_locked();
    rt_hw_interrupt_enable(level);

    return wb;
}

void drv_usbd_cdc_receive(uint8_t* buffer, uint32_t size)
{
    if (usbd_dev.rx_rb == RT_NULL) {
        return;
    }

    (void)ringbuffer_put(usbd_dev.rx_rb, buffer, size);
    hal_usbd_cdc_notify_status(&usbd_dev, USBD_STATUS_RX);
}

void drv_usbd_cdc_transmist_complete(uint8_t* buffer, uint32_t size)
{
    (void)buffer;
    (void)size;
    hal_usbd_cdc_notify_status(&usbd_dev, USBD_STATUS_TX_COMPLETE);
}

void drv_usbd_cdc_connect_cb(void)
{
    hal_usbd_cdc_notify_status(&usbd_dev, USBD_STATUS_CONNECT);
}

void drv_usbd_cdc_disconnect_cb(void)
{
    hal_usbd_cdc_notify_status(&usbd_dev, USBD_STATUS_DISCONNECT);
}

static void usbfs_ep0_prepare_tx(const uint8_t* data, uint16_t len)
{
    uint16_t tx_len = usbfs_min_u16(len, DEF_USBD_UEP0_SIZE);

    if (tx_len > 0) {
        memcpy(USBFS_EP0_Buf, data, tx_len);
    }

    pUSBFS_Descr = data + tx_len;
    USBFS_SetupReqLen = len - tx_len;
    USBFSD->UEP0_TX_LEN = tx_len;
    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
}

static void usbfs_ep0_status_in(void)
{
    pUSBFS_Descr = RT_NULL;
    USBFS_SetupReqLen = 0;
    USBFSD->UEP0_TX_LEN = 0;
    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
}

static void usbfs_ep0_stall(void)
{
    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_STALL;
    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_STALL;
}

static rt_bool_t usbfs_endpoint_halt(uint8_t ep, rt_bool_t halt)
{
    uint8_t tx_state = halt ? USBFS_UEP_T_RES_STALL : USBFS_UEP_T_RES_NAK;
    uint8_t rx_state = halt ? USBFS_UEP_R_RES_STALL : USBFS_UEP_R_RES_ACK;

    switch (ep) {
    case (DEF_UEP_IN | DEF_UEP1):
        USBFSD->UEP1_TX_CTRL = (USBFSD->UEP1_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | tx_state;
        return RT_TRUE;
    case (DEF_UEP_OUT | DEF_UEP2):
        USBFSD->UEP2_RX_CTRL = (USBFSD->UEP2_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | rx_state;
        rx_stopped = RT_FALSE;
        return RT_TRUE;
    case (DEF_UEP_IN | DEF_UEP3):
        USBFSD->UEP3_TX_CTRL = (USBFSD->UEP3_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | tx_state;
        tx_active = RT_FALSE;
        return RT_TRUE;
    default:
        return RT_FALSE;
    }
}

static rt_bool_t usbfs_endpoint_is_halted(uint8_t ep)
{
    switch (ep) {
    case (DEF_UEP_IN | DEF_UEP1):
        return ((USBFSD->UEP1_TX_CTRL & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_STALL) ? RT_TRUE : RT_FALSE;
    case (DEF_UEP_OUT | DEF_UEP2):
        return ((USBFSD->UEP2_RX_CTRL & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_STALL) ? RT_TRUE : RT_FALSE;
    case (DEF_UEP_IN | DEF_UEP3):
        return ((USBFSD->UEP3_TX_CTRL & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_STALL) ? RT_TRUE : RT_FALSE;
    default:
        return RT_FALSE;
    }
}

static void usbfs_handle_get_descriptor(uint16_t value, uint16_t request_len)
{
    const uint8_t* descriptor = RT_NULL;
    uint16_t len = 0;

    switch ((uint8_t)(value >> 8)) {
    case USB_DESCR_TYP_DEVICE:
        descriptor = MyDevDescr;
        len = DEF_USBD_DEVICE_DESC_LEN;
        break;
    case USB_DESCR_TYP_CONFIG:
        descriptor = MyCfgDescr;
        len = DEF_USBD_CONFIG_DESC_LEN;
        break;
    case USB_DESCR_TYP_STRING:
        switch ((uint8_t)(value & 0xFF)) {
        case DEF_STRING_DESC_LANG:
            descriptor = MyLangDescr;
            len = DEF_USBD_LANG_DESC_LEN;
            break;
        case DEF_STRING_DESC_MANU:
            descriptor = MyManuInfo;
            len = DEF_USBD_MANU_DESC_LEN;
            break;
        case DEF_STRING_DESC_PROD:
            descriptor = MyProdInfo;
            len = DEF_USBD_PROD_DESC_LEN;
            break;
        case DEF_STRING_DESC_SERN:
            descriptor = MySerNumInfo;
            len = DEF_USBD_SN_DESC_LEN;
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }

    if (descriptor == RT_NULL) {
        usbfs_ep0_stall();
        return;
    }

    usbfs_ep0_prepare_tx(descriptor, usbfs_min_u16(len, request_len));
}

static void usbfs_handle_class_request(void)
{
    switch (USBFS_SetupReqCode) {
    case CDC_GET_LINE_CODING:
        if ((USBFS_SetupReqType & USB_REQ_TYP_IN) && USBFS_SetupReqLen != 0) {
            usbfs_ep0_prepare_tx(line_coding, usbfs_min_u16(sizeof(line_coding), USBFS_SetupReqLen));
        } else {
            usbfs_ep0_stall();
        }
        break;
    case CDC_SET_LINE_CODING:
        if ((USBFS_SetupReqType & USB_REQ_TYP_IN) == 0 && USBFS_SetupReqLen == sizeof(line_coding)) {
            USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_ACK;
        } else {
            usbfs_ep0_stall();
        }
        break;
    case CDC_SET_CONTROL_LINE_STATE:
        control_line_state = USBFS_SetupReqValue;
        usbfs_ep0_status_in();
        break;
    case CDC_SEND_BREAK:
        usbfs_ep0_status_in();
        break;
    default:
        usbfs_ep0_stall();
        break;
    }
}

static void usbfs_handle_standard_request(void)
{
    static uint8_t status[2];

    switch (USBFS_SetupReqCode) {
    case USB_GET_DESCRIPTOR:
        usbfs_handle_get_descriptor(USBFS_SetupReqValue, USBFS_SetupReqLen);
        break;
    case USB_SET_ADDRESS:
        USBFS_DevAddr = (uint8_t)(USBFS_SetupReqValue & USBFS_USB_ADDR_MASK);
        set_address_pending = RT_TRUE;
        usbfs_ep0_status_in();
        break;
    case USB_GET_CONFIGURATION:
        status[0] = USBFS_DevConfig;
        usbfs_ep0_prepare_tx(status, usbfs_min_u16(1, USBFS_SetupReqLen));
        break;
    case USB_SET_CONFIGURATION:
        USBFS_DevConfig = (uint8_t)(USBFS_SetupReqValue & 0xFF);
        USBFS_DevEnumStatus = (USBFS_DevConfig != 0) ? 0x01 : 0x00;
        if (USBFS_DevConfig != 0) {
            drv_usbd_cdc_connect_cb();
        } else {
            drv_usbd_cdc_disconnect_cb();
        }
        usbfs_ep0_status_in();
        break;
    case USB_GET_INTERFACE:
        status[0] = 0;
        usbfs_ep0_prepare_tx(status, usbfs_min_u16(1, USBFS_SetupReqLen));
        break;
    case USB_SET_INTERFACE:
        usbfs_ep0_status_in();
        break;
    case USB_GET_STATUS:
        status[0] = 0;
        status[1] = 0;
        if ((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) {
            status[0] = usbfs_endpoint_is_halted((uint8_t)(USBFS_SetupReqIndex & 0xFF)) ? 1 : 0;
        } else if ((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE) {
            status[0] = (USBFS_DevSleepStatus & 0x01) ? 0x02 : 0x00;
        }
        usbfs_ep0_prepare_tx(status, usbfs_min_u16(2, USBFS_SetupReqLen));
        break;
    case USB_CLEAR_FEATURE:
        if ((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE &&
            (uint8_t)(USBFS_SetupReqValue & 0xFF) == USB_REQ_FEAT_REMOTE_WAKEUP) {
            USBFS_DevSleepStatus &= ~0x01;
            usbfs_ep0_status_in();
        } else if ((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP &&
                   (uint8_t)(USBFS_SetupReqValue & 0xFF) == USB_REQ_FEAT_ENDP_HALT &&
                   usbfs_endpoint_halt((uint8_t)(USBFS_SetupReqIndex & 0xFF), RT_FALSE)) {
            usbfs_ep0_status_in();
        } else {
            usbfs_ep0_stall();
        }
        break;
    case USB_SET_FEATURE:
        if ((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE &&
            (uint8_t)(USBFS_SetupReqValue & 0xFF) == USB_REQ_FEAT_REMOTE_WAKEUP) {
            USBFS_DevSleepStatus |= 0x01;
            usbfs_ep0_status_in();
        } else if ((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP &&
                   (uint8_t)(USBFS_SetupReqValue & 0xFF) == USB_REQ_FEAT_ENDP_HALT &&
                   usbfs_endpoint_halt((uint8_t)(USBFS_SetupReqIndex & 0xFF), RT_TRUE)) {
            usbfs_ep0_status_in();
        } else {
            usbfs_ep0_stall();
        }
        break;
    default:
        usbfs_ep0_stall();
        break;
    }
}

static void usbfs_handle_setup(void)
{
    const PUSB_SETUP_REQ setup = (PUSB_SETUP_REQ)USBFS_EP0_Buf;

    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_NAK;
    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_NAK;

    USBFS_SetupReqType = setup->bRequestType;
    USBFS_SetupReqCode = setup->bRequest;
    USBFS_SetupReqLen = setup->wLength;
    USBFS_SetupReqValue = setup->wValue;
    USBFS_SetupReqIndex = setup->wIndex;
    pUSBFS_Descr = RT_NULL;
    set_address_pending = RT_FALSE;

    if ((USBFS_SetupReqType & USB_REQ_TYP_MASK) == USB_REQ_TYP_STANDARD) {
        usbfs_handle_standard_request();
    } else if ((USBFS_SetupReqType & USB_REQ_TYP_MASK) == USB_REQ_TYP_CLASS) {
        usbfs_handle_class_request();
    } else {
        usbfs_ep0_stall();
    }
}

static void usbfs_cdc_handle_in_ep0(void)
{
    uint16_t send_len;

    if (USBFS_SetupReqLen > 0) {
        send_len = usbfs_min_u16(USBFS_SetupReqLen, DEF_USBD_UEP0_SIZE);
        memcpy(USBFS_EP0_Buf, pUSBFS_Descr, send_len);
        pUSBFS_Descr += send_len;
        USBFS_SetupReqLen -= send_len;
        USBFSD->UEP0_TX_LEN = send_len;
        USBFSD->UEP0_TX_CTRL ^= USBFS_UEP_T_TOG;
        USBFSD->UEP0_TX_CTRL = (USBFSD->UEP0_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
        return;
    }

    if (set_address_pending == RT_TRUE) {
        USBFSD->DEV_ADDR = (USBFSD->DEV_ADDR & USBFS_UDA_GP_BIT) | USBFS_DevAddr;
        set_address_pending = RT_FALSE;
    }

    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_ACK;
}

void USBFS_RCC_Init(void)
{
    if ((RCC->PLLCFGR & RCC_SYSPLL_SEL) != RCC_SYSPLL_USBHS) {
        RCC_USBHS_PLLCmd(DISABLE);
        RCC_USBHSPLLCLKConfig((RCC->CTLR & RCC_HSERDY) ? RCC_USBHSPLLSource_HSE : RCC_USBHSPLLSource_HSI);
        RCC_USBHSPLLReferConfig(RCC_USBHSPLLRefer_25M);
        RCC_USBHSPLLClockSourceDivConfig(RCC_USBHSPLL_IN_Div1);
        RCC_USBHS_PLLCmd(ENABLE);
        while (!(RCC->CTLR & RCC_USBHS_PLLRDY)) {
        }
    }

    RCC_USBFSCLKConfig(RCC_USBFSCLKSource_USBHSPLL);
    RCC_USBFS48ClockSourceDivConfig(RCC_USBFS_Div10);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_OTG_FS, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA, ENABLE);
}

void USBFS_Device_Endp_Init(void)
{
    USBFSD->UEP4_1_MOD = USBFS_UEP1_TX_EN;
    USBFSD->UEP2_3_MOD = USBFS_UEP2_RX_EN | USBFS_UEP3_TX_EN;

    USBFSD->UEP0_DMA = (uint32_t)USBFS_EP0_Buf;
    USBFSD->UEP1_DMA = (uint32_t)USBFS_EP1_Buf;
    USBFSD->UEP2_DMA = (uint32_t)USBFS_EP2_Buf;
    USBFSD->UEP3_DMA = (uint32_t)USBFS_EP3_Buf;

    usbfs_cdc_arm_rx();

    USBFSD->UEP0_TX_LEN = 0;
    USBFSD->UEP1_TX_LEN = 0;
    USBFSD->UEP3_TX_LEN = 0;

    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
    USBFSD->UEP1_TX_CTRL = USBFS_UEP_T_RES_NAK;
    USBFSD->UEP3_TX_CTRL = USBFS_UEP_T_RES_NAK;
}

void USBFS_Device_Init(FunctionalState sta)
{
    if (sta) {
        USBFSH->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
        USBFSH->BASE_CTRL = 0x00;
        USBFSD->INT_EN = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;
        USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
        USBFS_Device_Endp_Init();
        USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;
        NVIC_EnableIRQ(USBFS_IRQn);
    } else {
        USBFSH->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
        USBFSD->BASE_CTRL = 0x00;
        NVIC_DisableIRQ(USBFS_IRQn);
    }
}

void USBFS_Send_Resume(void)
{
    USBFSD->UDEV_CTRL ^= USBFS_UD_LOW_SPEED;
    rt_thread_delay(8);
    USBFSD->UDEV_CTRL ^= USBFS_UD_LOW_SPEED;
    rt_thread_delay(1);
}

void USBFS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USBFS_IRQHandler(void)
{
    GET_INT_SP();
    uint8_t intflag;
    uint8_t intst;
    uint16_t len = 0;

    rt_interrupt_enter();

    intflag = USBFSD->INT_FG;
    intst = USBFSD->INT_ST;

    if (intflag & USBFS_UIF_TRANSFER) {
        switch (intst & USBFS_UIS_TOKEN_MASK) {
        case USBFS_UIS_TOKEN_IN:
            switch (intst & (USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK)) {
            case USBFS_UIS_TOKEN_IN | DEF_UEP0:
                usbfs_cdc_handle_in_ep0();
                break;
            case USBFS_UIS_TOKEN_IN | DEF_UEP1:
                USBFSD->UEP1_TX_CTRL ^= USBFS_UEP_T_TOG;
                USBFSD->UEP1_TX_CTRL = (USBFSD->UEP1_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
                break;
            case USBFS_UIS_TOKEN_IN | DEF_UEP3:
                usbfs_cdc_complete_tx();
                break;
            default:
                break;
            }
            break;

        case USBFS_UIS_TOKEN_OUT:
            switch (intst & (USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK)) {
            case USBFS_UIS_TOKEN_OUT | DEF_UEP0:
                if ((intst & USBFS_UIS_TOG_OK) &&
                    USBFS_SetupReqCode == CDC_SET_LINE_CODING &&
                    USBFSD->RX_LEN >= sizeof(line_coding)) {
                    memcpy(line_coding, USBFS_EP0_Buf, sizeof(line_coding));
                    usbfs_ep0_status_in();
                } else if ((intst & USBFS_UIS_TOG_OK) && USBFSD->RX_LEN == 0) {
                    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_ACK;
                    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
                } else {
                    usbfs_ep0_stall();
                }
                break;

            case USBFS_UIS_TOKEN_OUT | DEF_UEP2:
                if (intst & USBFS_UIS_TOG_OK) {
                    USBFSD->UEP2_RX_CTRL ^= USBFS_UEP_R_TOG;
                    len = USBFSD->RX_LEN;
                    if (len > sizeof(rx_buffer)) {
                        len = sizeof(rx_buffer);
                    }

                    if (len > 0) {
                        memcpy(rx_buffer, USBFS_EP2_Buf, len);
                        (void)ringbuffer_put(usbd_dev.rx_rb, rx_buffer, len);
                        hal_usbd_cdc_notify_status(&usbd_dev, USBD_STATUS_RX);
                    }

                    if (usbd_dev.rx_rb != RT_NULL &&
                        ringbuffer_getlen(usbd_dev.rx_rb) >= USBFS_CDC_RX_RESUME_LEVEL) {
                        usbfs_cdc_stop_rx();
                    } else {
                        usbfs_cdc_arm_rx();
                    }
                }
                break;

            default:
                break;
            }
            break;

        case USBFS_UIS_TOKEN_SETUP:
            usbfs_handle_setup();
            break;

        default:
            break;
        }

        USBFSD->INT_FG = USBFS_UIF_TRANSFER;
    } else if (intflag & USBFS_UIF_BUS_RST) {
        USBFSD->DEV_ADDR = 0;
        USBFS_Device_Endp_Init();
        pUSBFS_Descr = RT_NULL;
        USBFS_SetupReqCode = 0;
        USBFS_SetupReqType = 0;
        USBFS_SetupReqValue = 0;
        USBFS_SetupReqIndex = 0;
        USBFS_SetupReqLen = 0;
        USBFS_DevAddr = 0;
        USBFS_DevConfig = 0;
        USBFS_DevSleepStatus = 0;
        USBFS_DevEnumStatus = 0;
        tx_active = RT_FALSE;
        tx_need_zlp = RT_FALSE;
        rx_stopped = RT_FALSE;
        set_address_pending = RT_FALSE;
        drv_usbd_cdc_disconnect_cb();
        USBFSD->INT_FG = USBFS_UIF_BUS_RST;
    } else if (intflag & USBFS_UIF_SUSPEND) {
        if (USBFSD->MIS_ST & USBFS_UMS_SUSPEND) {
            USBFS_DevSleepStatus |= 0x02;
        } else {
            USBFS_DevSleepStatus &= ~0x02;
        }
        USBFSD->INT_FG = USBFS_UIF_SUSPEND;
    } else {
        USBFSD->INT_FG = intflag;
    }

    rt_interrupt_leave();
    FREE_INT_SP();
}

struct usbd_cdc_ops usbd_ops = {
    .dev_init = RT_NULL,
    .dev_read = usbd_cdc_read,
    .dev_write = usbd_cdc_write,
    .dev_control = RT_NULL
};

rt_err_t drv_usb_cdc_init(void)
{
    rt_err_t err;

    usbd_dev.ops = &usbd_ops;

    if (tx_rb == RT_NULL) {
        tx_rb = ringbuffer_create(USBFS_CDC_TX_FIFO_SIZE);
        if (tx_rb == RT_NULL) {
            return FMT_ENOMEM;
        }
    }

    err = hal_usbd_cdc_register(&usbd_dev,
                                "usbd0",
                                RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX,
                                RT_NULL);
    if (err != RT_EOK) {
        return err;
    }

    USBFS_RCC_Init();
    USBFS_Device_Init(ENABLE);

    return RT_EOK;
}
