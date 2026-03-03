
#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#pragma config FOSC = HS
#pragma config PWRTE = ON
#pragma config WDTE = ON 
#pragma config CP = ALL
#pragma config LVP = OFF
#pragma config BOREN = ON
#pragma config CPD = OFF
#pragma config WRT = OFF

 // outputs
#define LEDC        RC3
#define DE          RC5

//analog input channels
#define JX   0
#define JY   1 

            //napeti

#define BUFMAX     5        

//filtered dig.inputs
union
{
    uint8_t B;
    struct
    {
      uint8_t       : 1;    
      uint8_t DOLU  : 1; 
      uint8_t SVETLO: 1; 
      uint8_t RES   : 1; 
      uint8_t NAHOR  :1; 
      uint8_t BRZDA : 1; 
      uint8_t       : 1; 
      uint8_t       : 1; 
    };
}
fil, fh;  // filtr, hrana vstupu
//} ai,  fil, fh,fd;  //vzorek, filtr, hrany vstupu

        //16bit. slovo w=[H,L]
typedef union
{
    uint16_t w;
    struct
    {
        uint8_t L;
        uint8_t H;
    }  ;
}word;

//_Bool  LBLIK, QBLIK, SBLIK, STALE;
const uint8_t COP=0x41, ALT= 0xa5, ALTINV = (uint8_t)(~ALT), AA= 0xaa, AAINV= (uint8_t)(~AA);
uint8_t in[8], set, res, film ;//prom. fitru
uint8_t step,stepold,i,  k, j , startime,bufout[BUFMAX];//krok programu, predch. krok,.., citac prodlevy 50 ms 
//uint8_t blik;// odmer. blik.LED,
uint16_t blik;// odmer. blik.LED,
uint16_t kniply,kniplx;//, naratim;;
word  kniplyfil, kniplxfil;
uint32_t timvyp;
uint8_t *ptr, prijaty, prijac, kom;

uint16_t adc_read(unsigned char channel)//mereni Adc
{  
ADCON0 = (channel << 3) + 0x81;		// !T enable ADC,  osc.32*Tosc
 //DelayUs(20); 
 j=6;   //t
 while(j)
     j--;
 GO_DONE = 1;
 while(GO_DONE)
 continue;	// wait for conversion complete
 return (((uint16_t)ADRESH)<<8) + ADRESL;
}

void adc_filter(uint16_t *act,uint16_t *filt)//filtr hodnot, merenych Adc
{
    int16_t ax= (int16_t)((*act)>>2) - (int16_t)((*filt)>>2);
    *filt += (uint16_t)ax; 
}

int main(int argc, char** argv)
{
    STATUS= 0x18;
    for(ptr=(uint8_t *)0x20; ptr < (uint8_t *)0x80; ptr++)
    {
        if(ptr  != (&ptr) )
        {
          *ptr=0;  
        }
    }
        //config
   TRISC=0x80;     //outs enable
   PORTC=0x0; 
   TRISB=0b00111110;
   PORTA= 0b00000011;
   OPTION_REGbits.nRBPU=0;// pull up
   OPTION_REGbits.PSA=0;// timer0 prescaller
   TRISA= 0b00000011;
   ADCON1bits.ADFM= 0;//left just. ADC
   ADCON1bits.PCFG= 4;//AN0,1,3
    //init dig. filters
   film= ~PORTB;; //inverted inputs
   for (j=0; j<8; j++)
   {
           in[j]= film;
   }
   fil.B= film;
   fh.B=0;
   
    //init adc's
   kniplxfil.w= adc_read(JX);
   kniplyfil.w= adc_read(JY);
   T2CONbits.TMR2ON= 1;//start T2   
   T1CONbits.T1CKPS= 1; //prescaller 2
  // TMR1= (uint16_t)(-20000);
   TMR1= (uint16_t)(-6000);
   TMR1ON=1;
   BRGH=1;
   SPBRG=25;		/*19200baud*/ 
   SPEN=1;		/*povoli seriovku*/
   TXEN=1;      /*povoli vysilani*/
   CREN=1;		/*povoli prijem*/ 
   //infinited cycle
   while(1)
   {    
     CLRWDT();  //clear watchdog timer
     if(TMR1IF)//1ms cyklus
     {
//         TMR1=(uint16_t)(-2000);
         TMR1= (uint16_t)(-1000);
         TMR1IF =0;
                   //digital filters Td=8*2=16ms
       k++;
       k %=8;
       in[k]= ~PORTB;    //inverted inputs
       set=0xff;
       res=0;
    
       for (j=0; j<8; j++)
       {
           set &= in[j];   //all 8 last 5ms samples must be 1 for set to 1  
           res |= (in[j]); //all 8 last 5ms samples must be 0 for reset to 0  
       }
       fil.B= ((~film) & set) | (film & (res));
       fh.B= ((~film) & fil.B ); //rise edge 
       film= fil.B;// memory     
       kniplx= adc_read(JX);
       kniply= adc_read(JY);
       adc_filter( &kniplx,&kniplxfil.w);
       adc_filter( &kniply,&kniplyfil.w);
       if(kom > 0)
       {
           kom--;
          
       }
       else
       {
            prijac=0;
       } 
       if(prijac==AA)   //sviti pri brzdeni
       {
           LEDC=1;
           blik=0;
       }
       else
       if(LEDC)
       {
         if(blik > 100)//jinak sviti 100ms puls
         {   
             LEDC=0;
             blik=0;
         }
       }
       else
       if(prijac==AAINV)//pri kniplu v nule blika
       {
        if(blik> 175)
        {
            LEDC=1;
            blik=0;
        } 
       }
       else
       if(prijac==ALT)//jede rovne nebo zataci
       {
        if(blik> 500)  
        {
            LEDC=1;
            blik=0;
        } 
       }
       else
       if(prijac==ALTINV)//prijat chybny pocet bytu
       {
        if(blik> 800)
        {
            LEDC=1;
            blik=0;
        } 
       }
       else
       //if(prijac==0)    
       {                //zadny prijem
        if(blik> 1400)
        {
            LEDC=1;
            blik=0;
        } 
       }     
       blik++;
     }
     if(RCIF && !DE)
     {
          if(OERR)
          {
                CREN = 0;
                CREN = 1;
          }
          prijaty= RCREG;  
     
          if((prijaty==ALT) || (prijaty==ALTINV) || (prijaty==AA) || (prijaty==AAINV))
          {
                kom=100;
                DE=1;
                TXIF=1;
                //bufout[0]=(fil.B & COP) | (fh.B & ~(COP));
                bufout[0]= fil.B;
                bufout[1]= kniplxfil.H;
                bufout[2]= kniplxfil.L;
                bufout[3]= kniplyfil.H;
                bufout[4]= kniplyfil.L;
                i=0;
                TXREG= bufout[i];
                prijac= prijaty;
                i++;
                
                        
          }
     }
     if(DE)
     {
            if(TRMT && (i >= BUFMAX))
            {
               DE=0;
               i=0;             
            } 
            else
            while(TXIF && (i<BUFMAX))
            {                   
               TXREG= bufout[i];
               i++;               
            }                  
           
            
     }
  }  
  return (EXIT_SUCCESS);
}

