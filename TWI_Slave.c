//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 09.07.2018.
//  Copyright __Ruedi Heimlicher__ 2018. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
//#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>


//***********************************
//Reset							*
//									*
//***********************************

#define TWI_PORT		PORTB
#define TWI_PIN		PINB
#define TWI_DDR		DDRB


#define LOOPLEDPIN            0        // Blink-LED

#define RASPIPIN              1        // Eingang von SDA/I2C
#define WEBSERVERPIN				2			// Eingang vom WebServer

#define OSZIPIN               3
#define REPORTPIN             3       // Wie OSZI. Meldet Reset an Webserver, active LO

#define RELAISPIN             4        // Schaltet Relais


#define DELTA            0x28   // 10s: Fehlercounter: Zeit bis Reset ausgeloest wird
#define RESETFAKTOR           1       // Vielfaches von DELTA

#define REBOOTFAKTOR          1

#define RESETDELAY            0x02   // Waitcounter: Blockiert wiedereinschalten
#define RASPIRESETFAKTOR       12

#define shutdownfaktor        5
#define killfaktor            5
#define relaxfaktor           3
#define rebootfaktor          10

#define WAIT                  0
#define SHUTDOWNWAIT          1
#define KILLWAIT              2
#define RELAXWAIT             3

#define REBOOTWAIT             4 // gesetzt, wenn SDA zulange LO ist

#define CHECK                 7 // in ISR gesetzt, resetcount soll erhoeht werden


#define Raspi_LO_MAX            0x8000
#define Raspi_HI_MAX            0xFFFF

void delay_ms(unsigned int ms);

volatile uint16_t	loopcount0=0;
volatile uint16_t	loopcount1=0;

volatile uint16_t	resetcount=0;
volatile uint16_t	delaycount=0; // Zaehlt wenn WAIT gesetzt ist: Delay fuer Relais

volatile uint16_t	rebootdelaycount=0; // Zaehler fuer Zeit, die der Raspi fuer Reboot braucht 

volatile uint8_t statusflag=0;

volatile uint16_t	overflowcount=0;



void slaveinit(void)
{
    
    CLKPR |= (1<<3);
    TWI_DDR |= (1<<LOOPLEDPIN);
    TWI_DDR |= (1<<RELAISPIN);       // Ausgang: Schaltet Reset-Ausgang fuer Zeit RESETDELAY auf LO
    TWI_PORT |= (1<<RELAISPIN);     // HI	
    
    TWI_DDR |= (1<<OSZIPIN);        // Ausgang
    TWI_PORT &= ~(1<<OSZIPIN);       // HI
    
    TWI_DDR &= ~(1<<RASPIPIN);        // Eingang: Verbunden mit Raspi, misst LO-Zeit, um Stillstand zu erkennen
    TWI_PORT |= (1<<RASPIPIN);        // HI
    
   TWI_DDR &= ~(1<<PB2);

//   TWI_DDR &= ~(1<<WEBSERVERPIN);        // Eingang: Verbunden mit Webserver, empfŠngt Signal zum reset
//   TWI_PORT |= (1<<WEBSERVERPIN);        // HI

   
   
   //TWI_DDR &= ~(1<<VCCPIN);	// Eingang, Abfragen von VCC von Master
    //TWI_PORT |= (1<<VCCPIN);	// HI
    
    //TWI_DDR &= ~(1<<SCLPIN);	// Eingang
    //TWI_PORT |= (1<<SCLPIN);	// HI
    
}



void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms)
	{
		_delay_ms(0.96);
		ms--;
	}
}

/* Initializes the hardware timer  */
void timer_init(void)
{
	/* Set timer to CTC mode */
	//TCCR0A = (1 << WGM01);
	/* Set prescaler */
	TCCR0B = (1 << CS00)|(1 << CS02); // clock/1024
	/* Set output compare register for 1ms ticks */
	//OCR0A = (F_CPU / 8) / 1000;
	/* Enable output compare A interrupts */
	TIMSK0 = (1 << TOIE0); // TOV0 Overflow
}

ISR(TIM0_OVF_vect) // Aenderung an SDA
{
   statusflag |= (1<<CHECK);
   TWI_PORT ^=(1<<LOOPLEDPIN);
   //TWI_PORT ^= (1<<OSZIPIN);
}

ISR(PCINT0_vect) // Potential-Aenderung 
{
   //TWI_PORT ^=(1<<OSZIPIN);
   if ((!(statusflag & (1<<WAIT))))// WAIT verhindert, dass Relais von Raspi_HI nicht sofort wieder zurueckgesetzt wird
   {
      // counter zuruecksetzen, alles OK
      resetcount=0;
      
    }

}

ISR(INT0_vect) // Potential-Aenderung von Raspi
{
   //TWI_PORT |=(1<<OSZIPIN);
   /*
   if ((!(statusflag & (1<<WAIT))))// WAIT verhindert, dass Relais von Raspi_HI nicht sofort wieder zurueckgesetzt wird
   {
      // counter zuruecksetzen, alles OK
      resetcount=0;
      Raspi_HI_counter=0;
      Raspi_LO_counter=0;
   }
   */
}

/*
ISR (SPI_STC_vect) // Neue Zahl angekommen
{
   OSZI_B_LO;
   if (inindex==0)
   {
      //OSZI_B_LO;
      //OSZI_B_HI;
      //isrcontrol = spi_txbuffer[inindex] ;
   }
   isrcontrol++;
   spi_rxbuffer[inindex] = SPDR;
   //isrcontrol = inindex;
   //isrcontrol +=inindex;
   SPDR = spi_txbuffer[inindex];
   //uint8_t input = SPDR;
   
   spi_rxdata=1;
   //inindex = inc(&inindex);
   inindex++;
   //inindex &= 0x0F;
   //SPI_Data_counter++;
   OSZI_B_HI;
}
*/


void main (void) 
{
   cli();
	wdt_disable();
//	MCUSR &= ~(1<<WDRF);
	wdt_reset();
//	WDTCR |= (1<<WDCE) | (1<<WDE);
//	WDTCR = 0x00;
//   WDTCR |= (1 << WDP3) | (1 << WDP0); // timer goes off every 8 seconds
	slaveinit();
   MCUCR |= (1<<ISC01);
//   GIMSK |= (1<<INT0);
   GIMSK |= 1<<PCIE;
   PCMSK |= 1<<PCINT2;
   timer_init();
   sei();
#pragma mark while
	while (1)
   {
      wdt_reset();
      //Blinkanzeige
      loopcount0++;
      if (loopcount0>=0x00AF)
      {
         //lcd_gotoxy(0, 0);
         //lcd_putint(loopcount1);
         loopcount0=0;
         
         loopcount1++;
         if (loopcount1 >0x4F)
         {
            //TWI_PORT ^=(1<<LOOPLEDPIN);
            loopcount1=0;
            //           TWI_PORT ^= (1<<RELAISPIN);            
         }         
      }
      
      
      if (statusflag & (1<<CHECK))// Timer gibt Takt der Anfrage an
      {         
         //TWI_PORT ^=(1<<OSZIPIN);
         statusflag &= ~(1<<CHECK);
         // resetcount wird bei Aenderungen am RaspiPIN  in ISR von INT0 zurueckgesetzt. (Normalbetrieb)
         //resetcount++;
         if ((resetcount > RESETFAKTOR * DELTA) & (!(statusflag & (1<<WAIT))) & (!(statusflag & (1<<REBOOTWAIT))))     // Zeit erreicht, kein wait-status
         {
            //TWI_PORT ^=(1<<OSZIPIN);
            // 3 Impuldse zum Abschalten
            uint8_t i = 0;
            for (i=0;i<3;i++)
            {
               TWI_PORT &= ~(1<<RELAISPIN);    // RELAISPIN LO, Reset fuer raspi
               _delay_ms(100);
               TWI_PORT |= (1<<RELAISPIN); //Ausgang wieder HI
               _delay_ms(100);
            }
           
            statusflag |= (1<<WAIT);      // WAIT ist gesetzt, Ausgang wird von Raspi_HI nicht sofort wieder zurueckgesetzt
            delaycount = 0;
         
         }
           
         if (statusflag & (1<<WAIT))
         {
            delaycount++; // Counter fuer Warten bis Raspi-shutdown, anschliessend ausschalten: Relasipin low fuer 5 sec 
            //TWI_PORT ^=(1<<OSZIPIN);
            if (delaycount > RESETDELAY) //Raspi ist down
            {
               //TWI_PORT |=(1<<OSZIPIN);
               statusflag &= ~0x1B ; // alle reset-Bits (3,4)
           //    TWI_PORT &= ~(1<<RELAISPIN); //Ausgang wieder LO
               statusflag &= ~(1<<WAIT);// WAIT zurueckgesetzt, Raspi_HI ist wieder wirksam
               statusflag |= (1<<REBOOTWAIT); //  Warten auf Ausschalten
               resetcount =0; 
               rebootdelaycount = 0;
            }            
         }
         else if (statusflag & (1<<REBOOTWAIT)) // reboot-procedure beginnen
         {
            rebootdelaycount++; // fortlaufend incrementieren, bestimmt ablauf
            
            
            if (rebootdelaycount > RESETDELAY) //Raspi ist down
            {
               statusflag &= ~(1<<REBOOTWAIT);
            }
         
         
         }
         else
         {
            // resetcounter inkrementieren, Normalbetrieb
            resetcount++;
            
         }
         //TWI_PORT ^=(1<<OSZIPIN);
         
         
        /* 
         // Reset durch Webserver: WEBSERVERPIN abfragen: Reset wenn LO
         
         if (TWI_PIN & (1 << WEBSERVERPIN))
         {
            //HI, alles OK
            webserverresetcount =0;
            delaycount=0;
            statusflag &= ~(1<<WAIT);
            //           TWI_PORT &= ~(1<<RELAISPIN);
         }
         else // webserverreset inc, reset wenn Eingang vom Webserver lange genug LO ist: Fehlerfall auf Webserver
         {
            webserverresetcount++;
            TWI_PORT ^=(1<<OSZIPIN);
            if (webserverresetcount > RASPIRESETDELAY)
            {
               TWI_PORT |= (1<<RELAISPIN);    // RELAISPIN Hi, Relais schaltet aus
               statusflag |= (1<<WAIT);      // WAIT ist gesetzt, Relais wird von Raspi_HI nicht zurueckgesetzt
               
            }
            
            if (webserverresetcount > (RASPIRESETDELAY + RESETDELAY))
            {
               //TWI_PORT |=(1<<OSZIPIN);
               TWI_PORT &= ~(1<<RELAISPIN);
               statusflag &= ~(1<<WAIT);// WAIT zurueckgesetzt, Raspi_HI ist wieder wirksam
               webserverresetcount =0;
               resetcount =0;
            }
            
         }
         */
      } // if check
      
   }//while
   
   
    //return 0;
}
