/* 
 * File:   main.c
 * Author: t_uebo
 *
 * 
 * " AF PSN"
 * 2018.07.06  Ver. 1.0
 * 
 * dsPIC33FJ64GP802 / XC16 (Ver.1.26)
 *
 */

#include <xc.h>
#include <dsp.h>


//Complex Coefficient for FIR Filter (300Hz-3.0kHz @fs=10.556kHz)
#include "H_filter.h"




// DSPIC33FJ64GP802 Configuration Bit Settings
// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)
// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)
// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)
// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Mode (Internal Fast RC (FRC) w/ PLL)
#pragma config IESO = ON                // Internal External Switch Over Mode
// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function (OSC2 pin has digital I/O function)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)
// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)
// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)


//#define LED LATBbits.LATB7
//#define dIN PORTBbits.RB5

#define Ndat 512
fractional datbf[Ndat*2], Icoeff[Ndat], Qcoeff[Ndat];

fractional Ich, Qch;
char f_ANch;

int Pt=0;

/*------------------------------------------------------------------------------------------
 * -----------------------------------------------------------------------------------------
 *           main
 * -----------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------*/
int main(int argc, char** argv) {
    int k;
    
	//------- Clock -------------------------------------
	CLKDIVbits.PLLPRE = 0;		//N1=1/2
	CLKDIVbits.PLLPOST = 0;		//N2=1/2
	PLLFBD = 42;				//M=42+2, Fin=(7.37(FSC)/2* 44/2 )/2 = 40.535MHz(Fcy)
    
	//------ Port ----------------------------------
	//TRISAbits.TRISA0 = 0;
	//LATB  = 0x0000;		// 
	//TRISB = 0xFF7F;			// RB7: output for LED 

	//--- ADC ------------------------------------------------
	AD1CON1 = 0x07E0;		// 12bit signed fractional, Manual Sample & Auto Start Conv.  
	AD1CON2 = 0;            // Vref: AVdd, AVss    
	AD1CON3 = 0x0802;		// Tsmp=8*Tad, Tad = 3*Tcy,  Tconv=(8+14)*Tad= 66/Fcy   
    AD1CHS0=0x0404;   
	AD1PCFGL = 0xFFCF;		// Use AN4,AN5    
	AD1CON1bits.ADON = 1;	// Enabled AD
    f_ANch=0;           
    IFS0bits.AD1IF=0;
    IPC3bits.AD1IP=7;
    IEC0bits.AD1IE=1;
    AD1CON1bits.SAMP=1;   // Start sampling
        
    ACLKCON=0x0700;  // ACLK=Fcy/1
    //#define Ndiv 30  // Fcy/64/1/Ndiv=21kHz
    #define Ndiv 60  // Fcy/64/1/Ndiv=10.556kHz

    //--- DAC ------------------------------------------------
    DAC1STATbits.ROEN = 1; // Enabled Right Channel DAC Output
    DAC1STATbits.LOEN = 1; // Enabled Left Channel DAC Output
    DAC1STATbits.RITYPE = 0; // Right Channel Interrupt: if FIFO is not Full
    DAC1STATbits.LITYPE = 0; // Left Channel Interrupt: if FIFO is not Full 
    DAC1CONbits.DACFDIV = Ndiv-1;  // Divide Clock by Ndiv
    DAC1CONbits.FORM = 1; // Data Format : Signed
    DAC1DFLT = 0x0000; // Default value when the FIFO is empty
    DAC1LDAT = 0x0000;
    DAC1RDAT = 0x0000;   
    DAC1CONbits.DACEN = 1; // Enabled DAC1 Module
    IFS4bits.DAC1LIF = 0; // Clear Left Channel Interrupt Flag  
    IFS4bits.DAC1RIF = 0; // Clear Right Channel Interrupt Flag
    IEC4bits.DAC1LIE = 1; // Enabled Left Channel Interrupt
    IEC4bits.DAC1RIE = 0; // Disabled Right Channel Interrupt    
    
    for(k=0; k<Ndat; k++){
        Icoeff[k]=H_Re[k];
        Qcoeff[k]=H_Im[k];
    }
         
    /*--------------------------------------------------------------------------
          Main Loop
     -------------------------------------------------------------------------*/
    while(1){}
    /*--------------------------------------------------------------------------
          End of Main Loop
     -------------------------------------------------------------------------*/  
    return (EXIT_SUCCESS);
}


void __attribute__((interrupt, no_auto_psv))_ADC1Interrupt(void){   
    IFS0bits.AD1IF=0;   
    if(f_ANch==0){
        Ich = ADC1BUF0;
        AD1CHS0=0x0505;
        AD1CON1bits.SAMP=1; // Start sampling
    }
    else{
        Qch = ADC1BUF0;
        AD1CHS0=0x0404;        
    }    
    f_ANch^=1;    
}


/*-------------------------------------------------------------------
  Complex Coefficient FIR Filter
---------------------------------------------------------------------*/
void __attribute__((interrupt, no_auto_psv))_DAC1LInterrupt(void)
{
    fractional Isig,Qsig;
    
    IFS4bits.DAC1LIF = 0;  // Clear Left Channel Interrupt Flag    
    Isig=Ich;
    Qsig=Qch;    
    AD1CON1bits.SAMP=1; // Start sample
       
    //----- FIR Filter  -------------------       
    datbf[Pt]=Isig; datbf[Pt+Ndat]=Isig;  
    Isig=VectorDotProduct(Ndat, &datbf[Pt], &Icoeff[0]);
    Qsig=VectorDotProduct(Ndat, &datbf[Pt], &Qcoeff[0]);
    
    Pt--;
    Pt&=(Ndat-1);

    DAC1LDAT=Isig;
    DAC1RDAT=Qsig; 
}

void __attribute__((interrupt, no_auto_psv))_DAC1RInterrupt(void)
{
    IFS4bits.DAC1RIF = 0;
}
