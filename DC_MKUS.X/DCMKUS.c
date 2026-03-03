
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
#define LEDC        RA0

#define NAHORU      RB0
#define DOLUP       RB1
#define SVETLOP     RB2
#define RESP        RB3
#define BRZDA_R     RB4
#define BRZDA_L     RB5

#define VZAD_R      RC0
#define PWM_L       RC1
#define PWM_R       RC2
#define VZAD_L      RC3
#define DE          RC5

#define BUFMAX      5       //5 byte komun. protokol 
#define PWMAX      0x40
//#define ERRMAX      10
#define ERRMAX      250
#define MIN      0x1400     //  pro 0,7V min. akcelerator 
#define SILNA ((0x100 * 83) / 100)  //85% PWM
#define NEJVETSI (0xff)     //100% PWM


        //16bit. slovo w=[H,L]
typedef union
{
    uint16_t w;
    int16_t sigw;
    struct
    {
        uint8_t L;
        uint8_t H;
    }  ;
}word;

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
state,statepre;  

typedef union
{
    uint8_t B;
    struct {
        unsigned b0 : 1;
        unsigned b1 : 1;
        unsigned b2 : 1;
        unsigned b3 : 1;
        unsigned b4 : 1;
        unsigned b5 : 1;
        unsigned b6 : 1;
        unsigned b7 : 1;
    };
}Bbits;
Bbits timled;
uint8_t svetlo,res,brzda ;          
_Bool VPRED, VZAD, VLEVO, VPRAVO, BRZDA, OSA, VPREDPRE, VZADPRE, VLEVOPRE, VPRAVOPRE, BRZDAPRE, OSAPRE;
_Bool FRELINK=0, COMOK= 0, PRIJATBYTE=0, BRZDENI, STOP;
// _Bool PAUSE;// priznak pauza
_Bool LBLIK, QBLIK;
const uint8_t COP=0x41, ALT= 0xa5, ALTINV = (uint8_t)(~ALT), AA= 0xaa, AAINV= (uint8_t)(~AA);
uint8_t in[8], set, res, film, ax , rychlost, rejd;//prom. fitru
uint8_t i,k,j, prodleva,startime,bufout, bufin[BUFMAX];//krok programu, predch. krok,.., citac prodlevy 50 ms 
uint8_t plyn,blik;//prepoctena hod. akc.do  pwm,  odmer. blik.LED, stav brzd. pedalu, citac po brzdeni
uint16_t kniply,kniplx;//, naratim;;
word  kniplyfil, kniplxfil, jizda, zatoc;
uint32_t timvyp;
uint8_t *ptr, prijaty, vadnych=0;
word axpc;

inline _Bool  nastav(uint8_t *vstup, _Bool b)//funkce vraci 1 jen prave v momentu sepnuti tlacitka -  2stavy sepnuto za sebou  nasleduji po 2 vzorcich stavu vypnuto]
{
    Bbits pom;
    pom.B= *vstup;
    pom.B <<=1;
    pom.b0= b;
    *vstup= pom.B;
    return (pom.b0 && pom.b1 && !pom.b2 && !pom.b3);
}

inline void setPWM1(void)   //nastavi 6 hornich bitu PWM v CCPRXL a 2 spodni do CCPXCON 
{
   CCPR1L = axpc.H;
   CCP1CON= axpc.L;
}

inline void setPWM2(void)   //nastavi 6 hornich bitu PWM v CCPRXL a 2 spodni do CCPXCON 
{
   CCPR2L = axpc.H;
   CCP2CON= axpc.L;
}


inline void countPWM(uint8_t strida) //prednastavi PWM
{
   axpc.H= strida;
   axpc.L=0xff;
   axpc.w >>= 2;
   axpc.L >>=2;
}

inline void vystupy(void)   //ridi vystupy
{
       if(statepre.DOLU == state.DOLU) 
        DOLUP= state.DOLU;
       if(statepre.NAHOR == state.NAHOR) 
        NAHORU= state.NAHOR;
       statepre= state;
       if(nastav(&svetlo, state.SVETLO ))
          SVETLOP= !SVETLOP;
       if(nastav(&brzda, state.BRZDA ))
       {
          BRZDENI= !BRZDENI;
       }
       if(nastav(&res, state.RES ))
          RESP= !RESP;
       jizda.sigw = (int16_t)(kniplxfil.w - 0x8000);//poloha kniplu jako int16_t, + =vpred, -=vzad
       if(jizda.sigw > MIN)
       {
           BRZDA= 0;
           VZAD_R=0;
           VZAD_L=0;
           jizda.w<<=1;
           rychlost= jizda.H;
       }
       else if(jizda.sigw < -(MIN))
       {
           BRZDA= 0;
           VZAD_R=1;
           VZAD_L=1;
           jizda.sigw= -jizda.sigw;
           jizda.w<<=1;
           rychlost= jizda.H;   //max. 0xff
       }
       else
       {
           BRZDA= 1;
       }
       zatoc.sigw = (int16_t)(kniplyfil.w - 0x8000);    //poloha kniplu zataceni + = vlevo, -= vpravo
       if(zatoc.sigw > MIN)
       {
           OSA= 0;
           VPRAVO=0;
           VLEVO=1;
           zatoc.w<<=1;
           rejd= zatoc.H;
       }
       else if(zatoc.sigw < -(MIN))
       {
           OSA= 0;
           VLEVO=0;
           VPRAVO=1;
           zatoc.sigw= -zatoc.sigw;
           zatoc.w<<=1;
           rejd = zatoc.H;
       }
       else
       {
           OSA= 1;
       }
       STOP= 0;
       if(BRZDENI)
       {
           BRZDA_L=1;
           BRZDA_R=1;
           countPWM(NEJVETSI);
           setPWM1();
           setPWM2(); 
       }
       else 
       if(BRZDAPRE && BRZDA)
       {
           STOP=1;
           BRZDA_L=1;
           BRZDA_R=1;
           countPWM(SILNA);
           setPWM1();
           setPWM2(); 
       }
       else
       if(OSAPRE && OSA)
       {
           BRZDA_L=0;
           BRZDA_R=0;
           countPWM(rychlost);
           setPWM1();
           setPWM2();
       }    
       else
       if (VPRAVOPRE && VPRAVO)
       {
           BRZDA_L=0;
           BRZDA_R=1;
           countPWM(rychlost);  //jet vlevo
           setPWM2();
           countPWM(rejd);      //brzdit vpravo
           setPWM1();
       }
       else if (VLEVOPRE && VLEVO )
       {
           BRZDA_L=1;
           BRZDA_R=0;
           countPWM(rejd);
           setPWM2();
           countPWM(rychlost);
           setPWM1();
       }
       OSAPRE= OSA;
       BRZDAPRE = BRZDA;
       VPRAVOPRE = VPRAVO;
       VLEVOPRE = VLEVO;      
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
   PORTC=0x0; 
   TRISC=0x80;     //outs enable
   TRISB= 0;
   PORTA= 0b00000000;
   OPTION_REGbits.nRBPU=0;// pull up
   OPTION_REGbits.PSA=0;// timer0 prescaller
   TRISA= 0b11100000;  
   CCPR2L=0;
   CCPR1L=0;
   CCP2CONbits.CCP2M= 0xf;//PWM RC1 VPRED
   CCP1CONbits.CCP1M= 0xf;//PWM RC2 VZAD
   T2CONbits.T2CKPS= 0;//prescaller=1
   PR2=PWMAX-2;   //32us TMR2 period
   //T2CONbits.TOUTPS=1;//postscalller=2, T2IF 1ms
   T1CONbits.T1CKPS= 1; //prescaller 2
   T2CONbits.TMR2ON= 1;//start T2   
   //TMR1=(uint16_t)(-40000);     //20ms
   TMR1=(uint16_t)(-6000);     //6ms
   TMR1ON=1;        //start T1
   T0CS=0;        //start T0
   PSA=0;           //prescaller
   BRGH=1;
   SPBRG=25;		/*19200baud*/ 
   SPEN=1;		/*povoli seriovku*/
   TXEN=1;		/*povoli vysilani*/
   CREN=1;		/*povoli prijem*/ 
   kniplyfil.w= 0x8000;
   kniplxfil.w= 0x8000;
   BRZDAPRE=1;
   vystupy();
   //infinited cycle
   while(1)
   {    
    // CLRWDT();  //clear watchdog timer      
       
       while(RCIF)
       {
           if(OERR)
           {
                CREN = 0;
                CREN = 1;
           }
           PRIJATBYTE=1;
           ax= RCREG;
           TMR1=(uint16_t)(-1000); //1ms       
           switch(i)
           {
                   case 0:
                       state.B= ax;
                   break;
                   case 1:
                       kniplxfil.H= ax;
                   break;
                   case 2:
                       kniplxfil.L= ax;
                   break;
                   case 3:
                       kniplyfil.H= ax;
                   break;
                   case 4:
                       kniplyfil.L= ax;                      
                   break;
                   default:
                       break;                    
           }
           i++;    
       }
       
       if(TMR1IF)   //uplynula nastavena prodleva bez prijmu dat
       {
           CLRWDT();  //clear watchdog timer    
           TMR1IF=0;
           TMR1=(uint16_t)(-6000);     //6ms
           FRELINK= 1;  //volna linka
           if(i==BUFMAX)//prijato 5Bytu?
           {
               if(BRZDENI)
                bufout= AA;
               else
               if(STOP)
                bufout= AAINV;
               else
                bufout= ALT;
               COMOK=  1;
               vadnych=0;
           }
           else
           {
               bufout= ALTINV;
               COMOK=0;
               if(i==0)//prijato 0 Bytu
               {                            
                        if(vadnych <= ERRMAX)
                        {
                            
                           vadnych++;
                           if(vadnych >= (ERRMAX-5))
                               LEDC=1;
                               
                        }
                        else 
                        {
                            TMR1= 0;    //nastavi 64ms, mezitim nastane watchdog reset
#ifdef __DEBUG
                            vadnych=0;
#endif
                        }
                }
                else
                vadnych=0;     
           }
           i=0;
       }
       if(DE)
       {
            if(TXIF)
            {      
                while(!TRMT)
                {
                    if(TMR1IF)
                    {
                        TMR1IF=0;
                        break;
                    }
                }                            
                i=0;
                DE=0;
             }
       }
       else if(FRELINK)   //volna linka a dosud se nevysila, pak se vysle byte
       {
           FRELINK=0;
         //  TMR1= (uint16_t)(-20000);//10ms
         //  TMR1= (uint16_t)(-12000);//6ms
           DE=1;
           i=0;
           PRIJATBYTE=0;
           TXREG= bufout;
           if(COMOK)        //pokud je prijem v poradku, 
               vystupy();   //nastavi se vystupy(jen LED se nastavi zvlast)
           TXIF=0;        //!t
       }   
       if(T0IF) //blikani LED
       {
        T0IF=0;
        timled.B ++;
        if(timled.b0 &&  timled.b1)
            LEDC=0;
        else
        if(COMOK)               //spravna komunikace
        {
            if((timled.B & 0x7)==0)
                LEDC=1;
        }
        /*
        else
        if(vadnych <= ERRMAX)   // komunikace, ale spatna
        {
             if((timled.B & 0x1f)==0)
                LEDC=1;
        }
        else
        if(timled.B==0)//
             LEDC=1; //bez komunikace blikne 1xza 8s 
         *          */
       }  
   
  }  
  return (EXIT_SUCCESS);
}
