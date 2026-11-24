#include "stm32g4xx_hal.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include <string.h>
#include <stdlib.h>

#define USBD_MAX_NUM_INTERFACES     1
#define USBD_MAX_NUM_CONFIGURATION  1
#define USBD_MAX_STR_DESC_SIZ       512
#define USBD_DEBUG_LEVEL            0
#define USBD_SELF_POWERED           1
#define MAX_STATIC_ALLOC_SIZE       512

PCD_HandleTypeDef hpcd_USB_FS;
USBD_HandleTypeDef hUsbDeviceFS;

/* --- Static Memory Allocation for USB --- */
static uint8_t usbd_alloc_buf[MAX_STATIC_ALLOC_SIZE];
void *USBD_static_malloc(uint32_t size) { (void)size; return usbd_alloc_buf; }
void USBD_static_free(void *p) { (void)p; }
void *USBD_malloc(uint32_t size) { return USBD_static_malloc(size); }
void USBD_free(void *p) { USBD_static_free(p); }

/* --- Descriptors --- */
#define USBD_VID     0x0483
#define USBD_PID     0x5740

static uint8_t USBD_DeviceDesc[18] = {
  0x12, 0x01, 0x00, 0x02, 0x02, 0x00, 0x00, 0x40,
  0x83, 0x04, 0x40, 0x57, 0x00, 0x02, 0x01, 0x02, 0x03, 0x01
};
static uint8_t USBD_LangIDDesc[4] = { 0x04, 0x03, 0x09, 0x04 };
static uint8_t USBD_StrDesc[255];

static uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    *length = sizeof(USBD_DeviceDesc); return USBD_DeviceDesc;
}
static uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    *length = sizeof(USBD_LangIDDesc); return USBD_LangIDDesc;
}
static uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    const char *str = "STMicroelectronics";
    uint8_t len = strlen(str);
    USBD_StrDesc[0] = len * 2 + 2; USBD_StrDesc[1] = 0x03;
    for(int i=0; i<len; i++) { USBD_StrDesc[2+i*2] = str[i]; USBD_StrDesc[3+i*2] = 0; }
    *length = USBD_StrDesc[0]; return USBD_StrDesc;
}
static uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    const char *str = "STM32 Virtual ComPort";
    uint8_t len = strlen(str);
    USBD_StrDesc[0] = len * 2 + 2; USBD_StrDesc[1] = 0x03;
    for(int i=0; i<len; i++) { USBD_StrDesc[2+i*2] = str[i]; USBD_StrDesc[3+i*2] = 0; }
    *length = USBD_StrDesc[0]; return USBD_StrDesc;
}
static uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    const char *str = "00000000001A";
    uint8_t len = strlen(str);
    USBD_StrDesc[0] = len * 2 + 2; USBD_StrDesc[1] = 0x03;
    for(int i=0; i<len; i++) { USBD_StrDesc[2+i*2] = str[i]; USBD_StrDesc[3+i*2] = 0; }
    *length = USBD_StrDesc[0]; return USBD_StrDesc;
}
static uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    const char *str = "STM32 Virtual ComPort Config";
    uint8_t len = strlen(str);
    USBD_StrDesc[0] = len * 2 + 2; USBD_StrDesc[1] = 0x03;
    for(int i=0; i<len; i++) { USBD_StrDesc[2+i*2] = str[i]; USBD_StrDesc[3+i*2] = 0; }
    *length = USBD_StrDesc[0]; return USBD_StrDesc;
}
static uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
    const char *str = "STM32 Virtual ComPort Interface";
    uint8_t len = strlen(str);
    USBD_StrDesc[0] = len * 2 + 2; USBD_StrDesc[1] = 0x03;
    for(int i=0; i<len; i++) { USBD_StrDesc[2+i*2] = str[i]; USBD_StrDesc[3+i*2] = 0; }
    *length = USBD_StrDesc[0]; return USBD_StrDesc;
}

USBD_DescriptorsTypeDef FS_Desc = {
  USBD_FS_DeviceDescriptor, USBD_FS_LangIDStrDescriptor,
  USBD_FS_ManufacturerStrDescriptor, USBD_FS_ProductStrDescriptor,
  USBD_FS_SerialStrDescriptor, USBD_FS_ConfigStrDescriptor,
  USBD_FS_InterfaceStrDescriptor
};

/* --- CDC Callbacks --- */
static int8_t CDC_Init_FS(void) { return 0; }
static int8_t CDC_DeInit_FS(void) { return 0; }
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length) { return 0; }
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len) {
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &pbuf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return 0;
}

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
  CDC_Init_FS, CDC_DeInit_FS, CDC_Control_FS, CDC_Receive_FS
};

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len) {
    uint8_t result = 1;
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (hcdc->TxState == 0) {
        USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
        result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
    }
    return result;
}

/* --- Link Layer (USBD_LL) mapping to HAL --- */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev) {
    hpcd_USB_FS.Instance = USB;
    hpcd_USB_FS.Init.dev_endpoints = 8;
    hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
    hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
    hpcd_USB_FS.Init.Sof_enable = DISABLE;
    hpcd_USB_FS.Init.low_power_enable = DISABLE;
    hpcd_USB_FS.Init.lpm_enable = DISABLE;
    hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
    if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) return USBD_FAIL;

    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x00, PCD_SNG_BUF, 0x18);
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x80, PCD_SNG_BUF, 0x58);
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x81, PCD_SNG_BUF, 0xC0);
    return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev) { HAL_PCD_DeInit(&hpcd_USB_FS); return USBD_OK; }
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev) { HAL_PCD_Start(&hpcd_USB_FS); return USBD_OK; }
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev) { HAL_PCD_Stop(&hpcd_USB_FS); return USBD_OK; }
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps) {
    HAL_PCD_EP_Open(&hpcd_USB_FS, ep_addr, ep_mps, ep_type); return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
    HAL_PCD_EP_Close(&hpcd_USB_FS, ep_addr); return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
    HAL_PCD_EP_Flush(&hpcd_USB_FS, ep_addr); return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
    HAL_PCD_EP_SetStall(&hpcd_USB_FS, ep_addr); return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
    HAL_PCD_EP_ClrStall(&hpcd_USB_FS, ep_addr); return USBD_OK;
}
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
    PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*) pdev->pData;
    if((ep_addr & 0x80) == 0x80) return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
    else return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
}
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr) {
    HAL_PCD_SetAddress(&hpcd_USB_FS, dev_addr); return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size) {
    HAL_PCD_EP_Transmit(&hpcd_USB_FS, ep_addr, pbuf, size); return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size) {
    HAL_PCD_EP_Receive(&hpcd_USB_FS, ep_addr, pbuf, size); return USBD_OK;
}
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
    return HAL_PCD_EP_GetRxCount(&hpcd_USB_FS, ep_addr);
}
void USBD_LL_Delay(uint32_t Delay) { HAL_Delay(Delay); }

/* --- HAL PCD Callbacks --- */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd) { USBD_LL_SetupStage((USBD_HandleTypeDef*)hpcd->pData, (uint8_t *)hpcd->Setup); }
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) { USBD_LL_DataOutStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff); }
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) { USBD_LL_DataInStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff); }
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd) { USBD_LL_SOF((USBD_HandleTypeDef*)hpcd->pData); }
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd) { USBD_LL_Reset((USBD_HandleTypeDef*)hpcd->pData); }
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd) { USBD_LL_Suspend((USBD_HandleTypeDef*)hpcd->pData); }
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd) { USBD_LL_Resume((USBD_HandleTypeDef*)hpcd->pData); }
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) { USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef*)hpcd->pData, epnum); }
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) { USBD_LL_IsoINIncomplete((USBD_HandleTypeDef*)hpcd->pData, epnum); }
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd) { USBD_LL_DevConnected((USBD_HandleTypeDef*)hpcd->pData); }
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd) { USBD_LL_DevDisconnected((USBD_HandleTypeDef*)hpcd->pData); }

/* --- MSP Init --- */
void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = 10; // AF10 for USB
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(USB_LP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_IRQn);
}

void MX_USB_DEVICE_Init(void) {
    USBD_Init(&hUsbDeviceFS, &FS_Desc, 0); // 0 = DEVICE_FS
    USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
    USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);
    USBD_Start(&hUsbDeviceFS);
}

void USB_LP_IRQHandler(void) {
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
}