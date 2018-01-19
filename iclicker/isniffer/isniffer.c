/*
 * Copyright 2010 Michael Ossmann
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <cc1110.h>
#include "ioCCxx10_bitdef.h"
#include "display.h"
#include "keys.h"
#include "stdio.h"
#include "isniffer.h"
#include "pm.h"

/* globals */
__bit sleepy;
static volatile u8 lcdupdate = 0;
static volatile u8 rf_mode_tx = 0;
static volatile u8 rfdone = 0;
static volatile u8 channame[2];
__xdata DMA_DESC dmaConfig_rx;
__xdata DMA_DESC dmaConfig_tx;

static volatile u8 input_state = INP_NORMAL;

u16 pktcount = 0;
u16 count_a = 0;
u16 count_b = 0;
u16 count_c = 0;
u16 count_d = 0;
u16 count_e = 0;
u16 count_err = 0;
u8 choice = 'A';

//#define NUM_CLICKERS 100
//xdata clicker clicker_table[NUM_CLICKERS];

#define LEN 6
__xdata u8 rxbuf[LEN];

__xdata u8 txbuf[LEN];

void setup_dma_tx()
{
	dmaConfig_tx.PRIORITY       = 2;  // high priority
	dmaConfig_tx.M8             = 0;  // not applicable
	dmaConfig_tx.IRQMASK        = 0;  // disable interrupts
	dmaConfig_tx.TRIG           = 19; // radio
	dmaConfig_tx.TMODE          = 2;  // single byte mode
	dmaConfig_tx.WORDSIZE       = 0;  // one byte words;
	dmaConfig_tx.VLEN           = 0;  // use LEN
	SET_WORD(dmaConfig_tx.LENH, dmaConfig_tx.LENL, LEN);

	SET_WORD(dmaConfig_tx.SRCADDRH, dmaConfig_tx.SRCADDRL, txbuf);
	SET_WORD(dmaConfig_tx.DESTADDRH, dmaConfig_tx.DESTADDRL, &X_RFD);
	dmaConfig_tx.SRCINC         = 1;  // increment by one
	dmaConfig_tx.DESTINC        = 0;  // do not increment

	SET_WORD(DMA1CFGH, DMA1CFGL, &dmaConfig_tx);

	return;
}

void setup_dma_rx()
{
	dmaConfig_rx.PRIORITY       = 2;  // high priority
	dmaConfig_rx.M8             = 0;  // not applicable
	dmaConfig_rx.IRQMASK        = 0;  // disable interrupts
	dmaConfig_rx.TRIG           = 19; // radio
	dmaConfig_rx.TMODE          = 0;  // single byte mode
	dmaConfig_rx.WORDSIZE       = 0;  // one byte words;
	dmaConfig_rx.VLEN           = 0;  // use LEN
	SET_WORD(dmaConfig_rx.LENH, dmaConfig_rx.LENL, LEN);

	SET_WORD(dmaConfig_rx.SRCADDRH, dmaConfig_rx.SRCADDRL, &X_RFD);
	SET_WORD(dmaConfig_rx.DESTADDRH, dmaConfig_rx.DESTADDRL, rxbuf);
	dmaConfig_rx.SRCINC         = 0;  // do not increment
	dmaConfig_rx.DESTINC        = 1;  // increment by one

	SET_WORD(DMA0CFGH, DMA0CFGL, &dmaConfig_rx);

	return;
}

void radio_setup() {
	/* IF setting */
    FSCTRL1   = 0x06;
    FSCTRL0   = 0x00;

	/* 905.5 MHz */
	FREQ2     = 0x22;
	FREQ1     = 0xD3;
	FREQ0     = 0xAC;
    CHANNR    = 0x00;
    /* 917.0 MHz */
	//FREQ2     = 0x23;
	//FREQ1     = 0x44;
	//FREQ0     = 0xec;

	/* maximum channel bandwidth (812.5 kHz), 152.34 kbaud */
    MDMCFG4   = 0x1C;
    MDMCFG3   = 0x80;

	/* DC blocking enabled, 2-FSK */
	MDMCFG2   = 0x01; // 15/16 bit sync
	//MDMCFG2   = 0x02; // 16/16 bit sync

	/* no FEC, 2 byte preamble, 250 kHz channel spacing */
    MDMCFG1   = 0x43;
    MDMCFG0   = 0x3B;
	
	/* 228.5 kHz frequency deviation */
    DEVIATN   = 0x71;
	/* 253.9 kHz frequency deviation */
    //DEVIATN   = 0x72;

    FREND1    = 0x56;   // Front end RX configuration.
    FREND0    = 0x10;   // Front end RX configuration.

	/* automatic frequency calibration */
    MCSM0     = 0x14;
	MCSM1     = 0x30; // TXOFF_MODE = IDLE
    BSCFG     = 0x6d;   // Bit Sync
    FSCAL3    = 0xE9;   // Frequency synthesizer calibration.
    FSCAL2    = 0x2A;   // Frequency synthesizer calibration.
    FSCAL1    = 0x00;   // Frequency synthesizer calibration.
    FSCAL0    = 0x1F;   // Frequency synthesizer calibration.
    TEST2     = 0x88;   // Various test settings.
    TEST1     = 0x31;   // Various test settings.
    TEST0     = 0x09;   // high VCO (we're in the upper 800/900 band)
    PA_TABLE0 = 0xC0;   // PA output power setting.

	/* no preamble quality check, no address check, no append status */
    //PKTCTRL1  = 0x00;
    //PKTCTRL1  = 0x84;
	/* preamble quality check 2*4=6, address check, append status */
    //PKTCTRL1  = 0x45;
    PKTCTRL1  = 0x01;
	/* no whitening, no CRC, fixed packet length */
    PKTCTRL0  = 0x00;

	/* packet length in bytes */
    PKTLEN    = LEN;

    SYNC1     = 0x85;
    SYNC0     = 0x85;
    ADDR      = 0x85;
}

/* tune the radio to a particular channel */
void tune(char *channame) {
	//FIXME bounds checking
    if (channame[0] < 'A' || channame[0] > 'D' || channame[1] < 'A' || channame[1] > 'D')
    {
        return;
    }
	CHANNR = channel_table[channame[0] - 'A'][channame[1] - 'A'];
}
void send_pkt(u8 choice)
{
    u16 csum;
	DMAARM = DMAARM_ABORT | DMAARM0;  // Arm DMA channel 0
    RFST = RFST_SIDLE;
    txbuf[0] = 0x85;
    txbuf[1] = 0x0c;
    txbuf[2] = 0x64;
    txbuf[3] = 0xf8;
    txbuf[4] = 0x70;
    switch(choice)
    {
        case 'A':
            txbuf[4]|=BUTTON_A;
            break;
        case 'B':
            txbuf[4]|=BUTTON_B;
            break;
        case 'C':
            txbuf[4]|=BUTTON_C;
            break;
        case 'D':
            txbuf[4]|=BUTTON_D;
            break;
        case 'E':
            txbuf[4]|=BUTTON_E;
            break;
    }
    csum = (txbuf[1]+txbuf[2]+txbuf[3]+txbuf[4]);
    txbuf[5] = (csum&0xff);
    rf_mode_tx = 1;
    IEN2 |= IEN2_RFIE;
    RFIM=RFIM_IM_DONE;
	DMAARM |= DMAARM1;  // Arm DMA channel 1
    RFST = RFST_STX;
    rfdone = 0;
    while (!rfdone) {};
    rfdone = 0;
    rf_mode_tx = 0;
	//DMAARM = DMAARM_ABORT | DMAARM1;  // Arm DMA channel 0
    IEN2 |= IEN2_RFIE;
    RFIM=RFIM_IM_DONE;
    RFST = RFST_SRX;
	//DMAARM ^= ~DMAARM1;  // Arm DMA channel 0
	DMAARM |= DMAARM0;  // Arm DMA channel 0
    
    
    
}

void poll_keyboard() {
    u8 key = getkey();
	switch (key) {
    case 'A':
    case 'B':
    case 'C':
    case 'D':
    case 'E':
        while(getkey() == key){};
        switch (input_state){
            case INP_NORMAL:
                choice = key;
                lcdupdate = 1;
                send_pkt(choice);
                break;
            case INP_CH0:
                if (key == 'E')
                    break;
                channame[0] = key;
                input_state = INP_CH1;
                lcdupdate = 1;
                break;
            case INP_CH1:
                if (key == 'E')
                    break;
                channame[1] = key;
                tune(channame);
                input_state = INP_NORMAL;
                lcdupdate = 1;
                break;
        }
        break;
    case KALT:
        while ( getkey() == KALT) {};
        if (input_state == INP_NORMAL)
            lcdupdate = 1;
            input_state = INP_CH0;
        break;
    case KBACK:
        while ( getkey() == KBACK) {};
        switch (input_state){
            case INP_NORMAL:
                break;
            case INP_CH0:
                lcdupdate = 1;
                input_state = INP_NORMAL;
                break;
            case INP_CH1:
                lcdupdate = 1;
                input_state = INP_CH0;
                break;
        }
        break;
	//case ' ':
	//	/* pause */
	//	while (getkey() == ' ');
	//	while (getkey() != ' ')
	//		sleepMillis(200);
	//	break;
	case KPWR:
		sleepy = 1;
		break;
	default:
		break;
	}
}

void main(void) {
	u16 i;
    channame[0]='A';
    channame[1]='A';
reset:
	sleepy = 0;
	xtalClock();
	setIOPorts();
	configureSPI();
	LCDReset();
	radio_setup();
	tune("AA");
	setup_dma_rx();
	setup_dma_tx();
	clear();
    count_a = 0;
    count_b = 0;
    count_c = 0;
    count_d = 0;
    count_e = 0;
    count_err = 0;
    pktcount=0;

	SSN = LOW;
	setCursor(0, 0);
	printf("isniffer");
	SSN = HIGH;

	while (1) {
		EA = 1; // enable interrupts globally
		IEN2 |= IEN2_RFIE; // enable RF interrupt
		RFIM = RFIM_IM_DONE; // mask IRQ_DONE only
		DMAARM |= DMAARM0;  // Arm DMA channel 0
    	RFST = RFST_SRX;
        // lcdupdate is set by the ISR which decodes packets
        // previously it was done here but was worried about missing packets
        // during display routines.
		if (lcdupdate)
        {
		    lcdupdate = 0;
            /*
             * we should be only counting one answer for each id, but for now we
             * just count every packet
             */
		    SSN = LOW;
            setCursor(0,60);
            if (input_state == INP_CH0)
                printf("CH: _");
            else
                printf("CH: %c",channame[0]);
            if (input_state == INP_CH1)
                printf("_");
            else
                printf("%c",channame[1]);
            setCursor(1,60);
            printf("Choice: %c",choice);
		    setCursor(1, 0);
		    printf("err: %d", count_err);
		    setCursor(2, 0);
		    printf("A: %d", count_a);
		    setCursor(3, 0);
		    printf("B: %d", count_b);
		    setCursor(4, 0);
		    printf("C: %d", count_c);
		    setCursor(5, 0);
		    printf("D: %d", count_d);
		    setCursor(6, 0);
		    printf("E: %d", count_e);
		    setCursor(7, 0);
		    printf("total packets: %d", pktcount);
		    SSN = HIGH;
	    }
		poll_keyboard();

		/* go to sleep (more or less a shutdown) if power button pressed */
		if (sleepy) {
			clear();
            RFST = RFST_SIDLE;
			sleepMillis(1000);
			SSN = LOW;
			LCDPowerSave();
			SSN = HIGH;

			while (1) {
				sleep();

				/* power button depressed long enough to wake? */
				sleepy = 0;
				for (i = 0; i < DEBOUNCE_COUNT; i++) {
					sleepMillis(DEBOUNCE_PERIOD);
					if (keyscan() != KPWR) sleepy = 1;
				}
				if (!sleepy) break;
			}

			/* reset on wake */
			goto reset;
		}
	}
}
void parse_pkt()
{
    u16 csum;
    u8 button;
    pktcount++;
    button = rxbuf[4]&0xf;
    csum = (rxbuf[1] + rxbuf[2]+rxbuf[3]+rxbuf[4]);

    if (rxbuf[5]!=(csum&0xff))
    {
        count_err++;
        return;
    }
    switch (button) {
        case BUTTON_A:
            count_a++;
            break;
        case BUTTON_B:
            count_b++;
            break;
        case BUTTON_C:
            count_c++;
            break;
        case BUTTON_D:
            count_d++;
            break;
        case BUTTON_E:
            count_e++;
            break;
        default:
            break;
    }
	lcdupdate = 1;
}

/* IRQ_DONE interrupt service routine */
void rf_isr() __interrupt (RF_VECTOR) {
	/* clear the interrupt flags */
	RFIF &= ~RFIF_IRQ_DONE;
	S1CON &= ~0x03;           // Clear the general RFIF interrupt registers
    if (! rf_mode_tx)
    {
        parse_pkt();
        RFST = RFST_SRX;
    }
    rfdone = 1;
}
