// Serial over CAN emulation for STM32 boards.
//
// Copyright (C) 2019 Eug Krashtan <eug.krashtan@gmail.com>
// Copyright (C) 2020 Pontus Borg <glpontus@gmail.com>
// Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "autoconf.h" // CONFIG_MACH_STM32F1
#include "board/irq.h" // irq_disable
#include "command.h" // DECL_CONSTANT_STR
#include "fasthash.h" // fasthash64
#include "generic/armcm_boot.h" // armcm_enable_irq
#include "generic/canbus.h" // canbus_notify_tx
#include "generic/serial_irq.h" // serial_rx_byte
#include "internal.h" // enable_pclock
#include "sched.h" // DECL_INIT

#if CONFIG_STM32_CANBUS_PB0_PB1
 DECL_CONSTANT_STR("RESERVE_PINS_CAN", "PB0,PB1");
 #define GPIO_Rx GPIO('B', 0)
 #define GPIO_Tx GPIO('B', 1)
#endif

#if CONFIG_MACH_STM32G0
 #if CONFIG_STM32_CANBUS_PB0_PB1
  #define SOC_CAN FDCAN2
 #else
  #error Uknown pins for STMF32G0 CAN
 #endif

 #define CAN_IT0_IRQn  TIM16_FDCAN_IT0_IRQn
 #define CAN_IT1_IRQn  TIM17_FDCAN_IT1_IRQn
 #define CAN_FUNCTION  GPIO_FUNCTION(3) // Alternative function mapping number

 #define FDCAN_IE_RX_FIFO0  (FDCAN_IE_RF0NE | FDCAN_IE_RF0FE | FDCAN_IE_RF0LE)
 #define FDCAN_IE_RX_FIFO1  (FDCAN_IE_RF1NE | FDCAN_IE_RF1FE | FDCAN_IE_RF1LE)
 #define FDCAN_IE_TC        (FDCAN_IE_TCE | FDCAN_IE_TCFE | FDCAN_IE_TFEE)

 #define SRAMCAN_FLS_NBR                  (28U)         /* Max. Filter List Standard Number      */
 #define SRAMCAN_FLE_NBR                  ( 8U)         /* Max. Filter List Extended Number      */
 #define SRAMCAN_RF0_NBR                  ( 3U)         /* RX FIFO 0 Elements Number             */
 #define SRAMCAN_RF1_NBR                  ( 3U)         /* RX FIFO 1 Elements Number             */
 #define SRAMCAN_TEF_NBR                  ( 3U)         /* TX Event FIFO Elements Number         */
 #define SRAMCAN_TFQ_NBR                  ( 3U)         /* TX FIFO/Queue Elements Number         */

 #define SRAMCAN_FLS_SIZE            ( 1U * 4U)         /* Filter Standard Element Size in bytes */
 #define SRAMCAN_FLE_SIZE            ( 2U * 4U)         /* Filter Extended Element Size in bytes */
 #define SRAMCAN_RF0_SIZE            (18U * 4U)         /* RX FIFO 0 Elements Size in bytes      */
 #define SRAMCAN_RF1_SIZE            (18U * 4U)         /* RX FIFO 1 Elements Size in bytes      */
 #define SRAMCAN_TEF_SIZE            ( 2U * 4U)         /* TX Event FIFO Elements Size in bytes  */
 #define SRAMCAN_TFQ_SIZE            (18U * 4U)         /* TX FIFO/Queue Elements Size in bytes  */

 #define SRAMCAN_FLSSA ((uint32_t)0)                                                      /* Filter List Standard Start Address */
 #define SRAMCAN_FLESA ((uint32_t)(SRAMCAN_FLSSA + (SRAMCAN_FLS_NBR * SRAMCAN_FLS_SIZE))) /* Filter List Extended Start Address */
 #define SRAMCAN_RF0SA ((uint32_t)(SRAMCAN_FLESA + (SRAMCAN_FLE_NBR * SRAMCAN_FLE_SIZE))) /* Rx FIFO 0 Start Address            */
 #define SRAMCAN_RF1SA ((uint32_t)(SRAMCAN_RF0SA + (SRAMCAN_RF0_NBR * SRAMCAN_RF0_SIZE))) /* Rx FIFO 1 Start Address            */
 #define SRAMCAN_TEFSA ((uint32_t)(SRAMCAN_RF1SA + (SRAMCAN_RF1_NBR * SRAMCAN_RF1_SIZE))) /* Tx Event FIFO Start Address        */
 #define SRAMCAN_TFQSA ((uint32_t)(SRAMCAN_TEFSA + (SRAMCAN_TEF_NBR * SRAMCAN_TEF_SIZE))) /* Tx FIFO/Queue Start Address        */
 #define SRAMCAN_SIZE  ((uint32_t)(SRAMCAN_TFQSA + (SRAMCAN_TFQ_NBR * SRAMCAN_TFQ_SIZE))) /* Message RAM size                   */

 uint32_t SramCanInstanceBase = SRAMCAN_BASE;
#endif

#ifndef SOC_CAN
 #error No known CAN device for configured MCU
#endif

// Read the next CAN packet
int
canbus_read(uint32_t *id, uint8_t *data)
{
    if (!(SOC_CAN->RXF0S & FDCAN_RXF0S_F0FL)) {
        // All rx mboxes empty, enable wake on rx IRQ
        irq_disable();
        SOC_CAN->IE |= FDCAN_IE_RF0NE;
        irq_enable();
        return -1;
    }

    // Read and ack packet
    uint32_t GetIndex = ((SOC_CAN->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos);
    uint32_t *RxAddress = (uint32_t *)(SramCanInstanceBase + SRAMCAN_RF0SA  + (GetIndex * SRAMCAN_RF0_SIZE));

    uint32_t rir_id = ((*RxAddress & 0x1FFFFFFFU) >> 18U);
    RxAddress++;
    uint32_t dlc = (*RxAddress & 0x000F0000U) >> 16;
    RxAddress++;
    /* Retrieve Rx payload */
    uint8_t *pData = (uint8_t *)RxAddress;

    // Return packet
    *id = rir_id;
    for (uint8_t i = 0; i < dlc; i++) {
        data[i] = pData[i];
    }
    SOC_CAN->RXF0A = GetIndex;
    return dlc;
}

// Transmit a packet
int
canbus_send(uint32_t id, uint32_t len, uint8_t *data)
{
    uint32_t txfqs = SOC_CAN->TXFQS;
    if (txfqs & FDCAN_TXFQS_TFQF) {
        // No space in transmit fifo - enable tx irq
        irq_disable();
        SOC_CAN->IE |= FDCAN_IE_TC;
        irq_enable();
        return -1;
    }

    uint32_t PutIndex = ((txfqs & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos);

    /* Calculate Tx element address */
    uint32_t *TxAddress = (uint32_t *)(SramCanInstanceBase + SRAMCAN_TFQSA + (PutIndex * SRAMCAN_TFQ_SIZE));
    /* Set up the ID */
    *TxAddress = (id << 18U);
    TxAddress++;
    /* Set up the DLC */
    *TxAddress = len << 16;
    TxAddress++;

    /* Set up the data field */
    if (len) {
        *TxAddress = (((uint32_t)data[3] << 24)
                    | ((uint32_t)data[2] << 16)
                    | ((uint32_t)data[1] << 8)
                    | ((uint32_t)data[0] << 0));
        TxAddress++;
        *TxAddress = (((uint32_t)data[7] << 24)
                    | ((uint32_t)data[6] << 16)
                    | ((uint32_t)data[5] << 8)
                    | ((uint32_t)data[4] << 0));
    }

    /* Request transmission */
    SOC_CAN->TXBAR = ((uint32_t)1 << PutIndex);
    return len;
}

void can_filter(uint32_t id, uint8_t index)
{
  uint32_t *FilterAddress = (uint32_t *)(SramCanInstanceBase + SRAMCAN_FLSSA + (index * SRAMCAN_FLS_SIZE));
  *FilterAddress = ((0x02 << 30)
                    | (0x01 << 27)
                    | (id << 16)
                    | 0x7FFU);
}

// Setup the receive packet filter
void
canbus_set_filter(uint32_t id)
{
    /* Request initialisation */
    SOC_CAN->CCCR |= FDCAN_CCCR_INIT;
    /* Wait the acknowledge */
    while (!(SOC_CAN->CCCR & FDCAN_CCCR_INIT))
        ;
    /* Enable configuration change */
    SOC_CAN->CCCR |= FDCAN_CCCR_CCE;

    can_filter(CANBUS_ID_ADMIN, 0);

    /*  List size standard */
    SOC_CAN->RXGFC &= ~(FDCAN_RXGFC_LSS);
    SOC_CAN->RXGFC |= 1 << FDCAN_RXGFC_LSS_Pos;

    /* Filter remote frames with 11-bit standard IDs
       Non-matching frames standard reject or accept in Rx FIFO 1 */
    SOC_CAN->RXGFC &= ~(FDCAN_RXGFC_RRFS | FDCAN_RXGFC_ANFS);
    SOC_CAN->RXGFC |= ((0 << FDCAN_RXGFC_RRFS_Pos)
                       | ((id ? 0x01 : 0x02)<< FDCAN_RXGFC_ANFS_Pos));

    /* Leave the initialisation mode for the filter */
    SOC_CAN->CCCR &= ~FDCAN_CCCR_CCE;
    SOC_CAN->CCCR &= ~FDCAN_CCCR_INIT;
}

// This function handles CAN global interrupts
void
CAN_IRQHandler(void)
{
    uint32_t ir = SOC_CAN->IR;
    uint32_t ie = SOC_CAN->IE;

    if (ir & FDCAN_IE_RX_FIFO1 && ie & FDCAN_IE_RX_FIFO1) {
        SOC_CAN->IR = FDCAN_IE_RX_FIFO1;

        if (SOC_CAN->RXF1S & FDCAN_RXF1S_F1FL) {
            // Read and ack data packet
            /* Calculate Rx FIFO 0 element address */
            uint32_t GetIndex = ((SOC_CAN->RXF1S & FDCAN_RXF1S_F1GI) >> FDCAN_RXF1S_F1GI_Pos);
            uint32_t *RxAddress = (uint32_t *)(SramCanInstanceBase + SRAMCAN_RF1SA + (GetIndex * SRAMCAN_RF1_SIZE));

            uint32_t rir_id = ((*RxAddress & 0x1FFFFFFFU) >> 18U);
            RxAddress++;
            uint32_t dlc = (*RxAddress & 0x000F0000U) >> 16;
            RxAddress++;
            /* Retrieve Rx payload */
            uint8_t *pData = (uint8_t *)RxAddress;
            uint8_t data[8];
            for (uint32_t i = 0; i < dlc; i++)
            {
              data[i] = pData[i];
            }
            SOC_CAN->RXF1A = GetIndex;

            // Process packet
            canbus_process_data(rir_id, dlc, data);
        }
    }
    if (ie & FDCAN_IE_RX_FIFO0 && ir & FDCAN_IE_RX_FIFO0) {
        // Admin Rx
        SOC_CAN->IR = FDCAN_IE_RX_FIFO0;
        canbus_notify_rx();
    }
    if (ie & FDCAN_IE_TC && ir & FDCAN_IE_TC) {
        // Tx
        SOC_CAN->IR = FDCAN_IE_TC;
        canbus_notify_tx();
    }
}

static inline const uint32_t
make_btr(uint32_t sjw,       // Sync jump width, ... hmm
         uint32_t time_seg1, // time segment before sample point, 1 .. 16
         uint32_t time_seg2, // time segment after sample point, 1 .. 8
         uint32_t brp)       // Baud rate prescaler, 1 .. 1024
{
    return (((uint32_t)(sjw-1)) << FDCAN_NBTP_NSJW_Pos
            | ((uint32_t)(time_seg1-1)) << FDCAN_NBTP_NTSEG1_Pos
            | ((uint32_t)(time_seg2-1)) << FDCAN_NBTP_NTSEG2_Pos
            | ((uint32_t)(brp - 1)) << FDCAN_NBTP_NBRP_Pos);
}

static inline const uint32_t
compute_btr(uint32_t pclock, uint32_t bitrate)
{
    /*
        Some equations:
        Tpclock = 1 / pclock
        Tq      = brp * Tpclock
        Tbs1    = Tq * TS1
        Tbs2    = Tq * TS2
        NominalBitTime = Tq + Tbs1 + Tbs2
        BaudRate = 1/NominalBitTime

        Bit value sample point is after Tq+Tbs1. Ideal sample point
        is at 87.5% of NominalBitTime

        Use the lowest brp where ts1 and ts2 are in valid range
     */

    uint32_t bit_clocks = pclock / bitrate; // clock ticks per bit

    uint32_t sjw =  2;
    uint32_t qs;
    // Find number of time quantas that gives us the exact wanted bit time
    for (qs = 18; qs > 9; qs--) {
        // check that bit_clocks / quantas is an integer
        uint32_t brp_rem = bit_clocks % qs;
        if (brp_rem == 0)
            break;
    }
    uint32_t brp       = bit_clocks / qs;
    uint32_t time_seg2 = qs / 8; // sample at ~87.5%
    uint32_t time_seg1 = qs - (1 + time_seg2);

    return make_btr(sjw, time_seg1, time_seg2, brp);
}

void
can_init(void)
{
    enable_pclock((uint32_t)SOC_CAN);

    gpio_peripheral(GPIO_Rx, CAN_FUNCTION, 1);
    gpio_peripheral(GPIO_Tx, CAN_FUNCTION, 0);

    uint32_t pclock = get_pclock_frequency((uint32_t)SOC_CAN);

    uint32_t btr = compute_btr(pclock, CONFIG_CANBUS_FREQUENCY);

    /*##-1- Configure the CAN #######################################*/

    /* Exit from sleep mode */
    SOC_CAN->CCCR &= ~FDCAN_CCCR_CSR;
    /* Wait the acknowledge */
    while (SOC_CAN->CCCR & FDCAN_CCCR_CSA)
        ;
    /* Request initialisation */
    SOC_CAN->CCCR |= FDCAN_CCCR_INIT;
    /* Wait the acknowledge */
    while (!(SOC_CAN->CCCR & FDCAN_CCCR_INIT))
        ;
    /* Enable configuration change */
    SOC_CAN->CCCR |= FDCAN_CCCR_CCE;

    if (SOC_CAN == FDCAN1)
        FDCAN_CONFIG->CKDIV = 0;

    /* Disable automatic retransmission */
    SOC_CAN->CCCR |= FDCAN_CCCR_DAR;
    /* Disable protocol exception handling */
    SOC_CAN->CCCR |= FDCAN_CCCR_PXHD;

    SOC_CAN->NBTP = btr;

    /* Leave the initialisation mode */
    SOC_CAN->CCCR &= ~FDCAN_CCCR_CCE;
    SOC_CAN->CCCR &= ~FDCAN_CCCR_INIT;

    if (SOC_CAN == FDCAN2)
        SramCanInstanceBase += SRAMCAN_SIZE;

    /*##-2- Configure the CAN Filter #######################################*/
    canbus_set_filter(0);

    /*##-3- Configure Interrupts #################################*/
    armcm_enable_irq(CAN_IRQHandler, CAN_IT0_IRQn, 0);
    if (CAN_IT0_IRQn != CAN_IT1_IRQn)
        armcm_enable_irq(CAN_IRQHandler, CAN_IT1_IRQn, 0);
    SOC_CAN->ILE |= 0x03;
    SOC_CAN->IE |= FDCAN_IE_RX_FIFO0 | FDCAN_IE_RX_FIFO1;

    // Convert unique 96-bit chip id into 48 bit representation
    uint64_t hash = fasthash64((uint8_t*)UID_BASE, 12, 0xA16231A7);
    canbus_set_uuid(&hash);
}
DECL_INIT(can_init);
