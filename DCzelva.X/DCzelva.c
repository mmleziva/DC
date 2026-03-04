/* 
 * File:   DC.c
 * Author: mleziva
 *
 * Created on  2024
 */
//#define TEST
//#define ACU24V
#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#ifndef TEST
  #pragma config FOSC = XT
  #pragma config PWRTE = ON
  #pragma config WDTE = ON 
  #pragma config CP = ALL
#else
  #pragma config FOSC = HS    //!t
  #pragma config PWRTE = OFF //!t
  #pragma config WDTE = OFF  //!t
  #pragma config DEBUG = 0   //!t
#endif
#pragma config LVP = OFF
#pragma config BOREN = ON 
#pragma config CPD = OFF
#pragma config WRT = OFF

 // outputs
#define RESERVE     RC0
#define PWM_VPRED   RC1	
#define RES2        RC2
#define BEZP_RELE   RC3	
#define VPRED       RC4
#define OUT_BRZDA   RC5
#ifndef TEST
    #define LEDZ        RC6
    #define LEDC        RC7
    #define LEDPORT PORTC 
    #define LEDORAN 0xc0 
#else
    #define LEDZ        RE0 //!t
    #define LEDC        RE1 //!t
    #define LEDPORT PORTE   //!t
    #define LEDORAN 0x3 //!t
#endif
#define LEDONEG (~LEDORAN) 
//#define OUT_START  RA4
//#define NAP_OCHRA  RB2
#define U9_21     RB0

//analog input channels
#define AKCELERATOR 0
#ifndef TEST
  #define U_BATERIE   1 
#else
    #define U_BATERIE   3 //!t
#endif
#define U_FUSE      4

        //stavy programu
#define INI         0
#define READY       2
#define VYBITA      8
#define GOVPRED    10
#define PWMVPRED   12
#define BRZDENI    30
#define OCHR_BAT   50
#define NADBAT     52
#define FUSE       60


            //napeti
#define PWMAX      251
#define PWMIN      (((PWMAX)*15)/100)   //15% PWMAX
#define AKCMIN    (256/5)  //bitu /V 
#define AKCMIN16b (0x10000/5)  //bitu /V 
#define AKCMAX    ((256*4)/5)   //bitu/4V
#define AKCMUL     (((PWMAX - PWMIN)*5)/(4-1))
#define BRZDMIN      (((PWMAX)*20)/100)   //20% PWMAX
#define BRZDMUL     (((PWMAX - BRZDMIN)*5)/(4-1))
#ifdef ACU24V
   #ifndef TEST
    #define KBAT         ((0x10000*2)/(10*10*5))//24V
   #else
    #define KBAT         ((0x10000 * 2)/200)  //!t 24V
   #endif
#else
    #ifndef TEST
      #define KBAT         (0x10000/(10*10*5))
    #else
      #define KBAT         (0x10000/200)  //!t
    #endif
#endif
#define UBATOCH      (KBAT*82)//8,5V 
#define UBATVYB      (KBAT*110)// 11,4V
#define UBATOK       (KBAT*102)// 11,0V
#define UBAT30       (KBAT*115)// 12,0V
#define UBATLOD      (KBAT*137)// 14,5V
#define UBATHOD      (KBAT*140)// 15V
#define FUSEMIN      (KBAT*67)//6V  //n 
#define U9V          (KBAT*88)//9V  //n 
#define U21V         (KBAT*196)//21V  //n 

        //casy 1bit=2ms
#define ZPOZBAT    15000      //30s
#define TIMINC     50        //100ms
#define TRAMPINC   (900/(PWMAX-PWMIN))  //pridavani plynu 3,2s //nn
#define TIMAX      (150000UL)//1000ms *60 *5= 5 min.


//filtered dig.inputs
union
{
    uint8_t B;
    struct
    {
      uint8_t      : 1;    
      uint8_t BRZDA :1; 
      uint8_t       : 1; 
      uint8_t       : 1; 
      uint8_t        :1; 
      uint8_t START_IN: 1; 
      uint8_t       : 1; 
      uint8_t       : 1; 
    };
} ai,  fil, fh,fd;  //vzorek, filtr, hrany vstupu

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

_Bool PAUSE;// priznak pauza
_Bool POJISTKA;
_Bool ULOW, LBLIK, QBLIK, AKCEL_SEP, VYB;
uint8_t in[8], set, res, film ;//prom. fitru
uint8_t step,stepold, k, j ,lt,ltnaraz, prodleva,probrzd,startime;//krok programu, predch. krok,.., citac prodlevy 50 ms 
uint8_t plyn,blik,brac;//prepoctena hod. akc.do  pwm,  odmer. blik.LED, stav brzd. pedalu
uint16_t baterie,akcel,fuse,baterfil, akcelfil,fusefil,zpozdeni;//, naratim;;
uint32_t timvyp;

uint16_t adc_read(unsigned char channel)//mereni Adc
{  
ADCON0 = (channel << 3) + 0x41;		// !T enable ADC,  osc.8*Tosc
 //DelayUs(20); 
 j=3;
 while(j)
     j--;
 GO_DONE = 1;
 while(GO_DONE)
 continue;	// wait for conversion complete
 return (((uint16_t)ADRESH)<<8) + ADRESL;
}

void adc_filter(uint16_t *act,uint16_t *filt)//filtr hodnot, merenych Adc
{
    int16_t ax= (int16_t)((*act)>>4) - (int16_t)((*filt)>>4);
    *filt += (uint16_t)ax; 
}

uint8_t pwmakc(uint16_t uak)// prepocet napeti akceleratoru na pwm 
{
    word uacc, resw;
    uint8_t huac, result, difhu;
    uacc.w= uak;
    huac= uacc.H;
    if(huac <= AKCMIN)
        result=0;
    else if(huac >= AKCMAX)
        result= PWMAX;
    else
    {
        difhu = huac-AKCMIN;
        resw.w= AKCMUL * difhu;
        result=resw.H +PWMIN;
    }
    return result;
}

int main(int argc, char** argv)
{
        //config
   PORTC=0x0; 
   TRISC=0x0;     //outs enable
   TRISBbits.TRISB0=0;//out U9_21
   //OUT_START=1;
   PORTA= 0b00011100;
   OPTION_REGbits.nRBPU=0;// pull up
   ///TRISAbits.TRISA4=0;     //OUT_START out enable
   #ifndef TEST
    TRISA= 0b11100011;
   #else
    TRISA= 0b11101111;//t
    TRISEbits.TRISE0=0;//!t
    TRISEbits.TRISE1=0;//!t
   #endif
   ADCON1bits.ADFM= 0;//left just. ADC
   step=INI;
    //init dig. filters
   ai.B= ~PORTB;
   film= ai.B; //inverted inputs
   for (j=0; j<8; j++)
   {
           in[j]= film;
   }
   fil.B= film;
   fh.B=0;
   
    //init adc's
   akcelfil= adc_read(AKCELERATOR);
   baterfil= adc_read(U_BATERIE);
   fusefil= adc_read(U_FUSE);
   OUT_BRZDA=1; //odblokovani
   CCPR2L=0;
   CCPR1L=0;
   CCP2CONbits.CCP2M= 0xf;//PWM RC1 VPRED
   CCP1CONbits.CCP1M= 0xf;//PWM RC2 VZAD
   T2CONbits.T2CKPS= 1;//prescaller=4
   PR2=PWMAX-1;   //250*4=1000us TMR2 period
  // T2CONbits.TOUTPS=1;//postscalller=2, T2IF 2ms
   T2CONbits.TMR2ON= 1;//start T2   
   TMR1=-2000;
   TMR1ON=1;
   //infinited cycle
   while(1)
   {    
     CLRWDT();  //clear watchdog timer
     if(TMR1IF)//2ms cyklus
 //    if(TMR2IF)//2ms cyklus
     {
         TMR1=-2000;
         TMR1IF =0;
                   //digital filters Td=8*2=16ms
       k++;
       k %=8;
       ai.B= ~PORTB;    //inverted inputs
       in[k]= ai.B;
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
       akcel= adc_read(AKCELERATOR);
       baterie= adc_read(U_BATERIE);
       fuse= adc_read(U_FUSE);
       adc_filter( &akcel,&akcelfil);
       adc_filter( &baterie,&baterfil);
       adc_filter( &fuse,&fusefil);
       LBLIK= ((blik & 0x80) != 0);//priznak blikani
       QBLIK= ((blik & 0x20) != 0);//priznak blikani 4x rychleji
       blik++;
       if(probrzd >0)//prodleva 8ms pro vypinani rele
       {
        probrzd++;
        if(probrzd >4)
           probrzd=0;
       }
       AKCEL_SEP=(akcelfil > AKCMIN16b);
       BEZP_RELE= AKCEL_SEP;     
       U9_21=(baterfil> U9V)&&(baterfil<U21V);
            
                //prioritni akce
       if(AKCEL_SEP && (step != FUSE)&& (fusefil< FUSEMIN)) //sledovani stavu silove pojistky
       {
             LEDPORT &= LEDONEG;      //zhasni obe LED
             step= FUSE;
             VPRED=0; 
             probrzd=1; //zapina odpocitani prodlevy pro vypinani rele
             LEDC=1;  //rozsvit cervenou
       }
     
       else 
       if((baterfil < UBATOCH) && (step != OCHR_BAT))   //podpetova ochrana baterie  
       {
             LEDPORT &= LEDONEG;      //zhasni obe LED   //nn
             VPRED=0;  
             CCPR2L= 0;
             probrzd=1;//zapina odpocitani prodlevy pro vypinani rele
             LEDC= 1;   //n
             step= OCHR_BAT;
       }
       else
       if(fil.BRZDA && (step != BRZDENI)&&(step != FUSE)&& (step != OCHR_BAT))     //pouziti  brzdy
       {
           probrzd=1;//zapina odpocitani prodlevy pro vypinani rele
           VPRED=0;   
           CCPR2L= 0;
           LEDPORT &= LEDONEG;//led zhasnuty
           LEDC=1;
           OUT_BRZDA=0;
           step= BRZDENI;           
       }
       else
       if(ULOW && (step != BRZDENI))    //blikani cervene pri vybite bat.
       {                
              VPRED=0;
              CCPR2L=0;
              step= VYBITA;
       }
       else
       if((baterfil > UBATHOD)&&(step != NADBAT))   //nadpetova ochrana
       {
           VPRED=0;
           LEDPORT &= LEDONEG;//led zhasnuty
           OUT_BRZDA=0;
           LEDC=1;
           CCPR2L=0;           
           step= NADBAT;
       }
       else
       if(step != BRZDENI)
           plyn = pwmakc(akcelfil);// jinak se meri pedal plynu     
       if(zpozdeni > ZPOZBAT)// po prodleve 30s
       {
           zpozdeni= 0;
           if((baterfil< UBAT30)&& VYB)
           {
               ULOW=1;  //nastav priznak vybite baterie
           }
           VYB=0;
       }
       else 
       {
         if(zpozdeni > 0)
             zpozdeni ++; 
       }
       
       if( AKCEL_SEP )   //test oddaleni vypnuti elektroniky
       {
           timvyp=0;
       }
       else 
       if(timvyp > TIMAX)// test vypnuti po 5 minutach
       {
            //  OUT_START=0;
              PORTA=0x0;
              TRISA=0b11111111;
       }
       else
       {
         //     OUT_START=1;
#ifndef  TEST
           PORTA=0b00011100;
           TRISA=0b11100011;
#else
           PORTA=0b00010000;//t
           TRISA=0b11101111;//t
#endif

           timvyp++;
       }    
                        //prodleva po startu
       if(startime< TIMINC)
       {
         startime++;
         step= INI;
       }      
                //vyber akce dle aktualniho stavu
       switch (step)
       {
        case INI: //po zapnuti napajeni
            step= READY;
            LEDZ=1;
            break;
        case READY: //pripraven pro rozjezd
       if(plyn > 0)
            {
             step= GOVPRED; 
             VPRED= 1;
            }
            break;                       
       case VYBITA:
            LEDC=LBLIK;
            break;   

       case GOVPRED: //prodleva 100ms pred rozjezdem
            prodleva++;
            if(prodleva> TIMINC)
            {
                prodleva=0;
                step = PWMVPRED;
                CCPR2L= PWMIN;
                VYB=0;
            }
            break;
        case PWMVPRED: //plyn vpred
             if(plyn < PWMIN)  //po uvolneni plynu navrat na ready
             {
                 CCPR2L=0;
                 VPRED=0;
                 step= READY;   
                 LEDPORT &= LEDONEG;//oranz zhasnuta
                 LEDZ= 1;
                 zpozdeni=1; //zacina pocitat zpozdeni pro test vybite bat.  
             }
             else
             if((plyn >= PWMAX)&&(CCPR2L >= plyn)) // plyn nadoraz neblika
             {
                  LEDZ=1;
             }
             else
             {
              LEDZ= QBLIK;    
              if(plyn > CCPR2L)//POSTUPNE PRIDAVA PLYN
              {
                 lt++;
                 if(lt > TRAMPINC)
                 {
                     lt=0;
                     CCPR2L++;
                 }
              }
              else
              if(plyn < PWMAX)    //nn
                   CCPR2L= plyn;
             }      
             ULOW= 0;
             probrzd=1;
             if(baterfil < UBATVYB)
                  VYB=1;
             break;
         
        case BRZDENI: //
            if(!fil.BRZDA) //konec  brzdy  //tt
            {
                OUT_BRZDA=1;
                LEDPORT &= LEDONEG;//led zhasnuty
                LEDZ=1;
                step= READY;
            }
            else
            if(probrzd==0)    //brzdeni 
            {      
              //  OUT_BRZDA=0;      //klidovym kontaktem 
                PORTC &= 0xdf;   //klidovym kontaktem 
                LEDC=1;
            }
            break;

        case OCHR_BAT:
            if(baterfil> UBATOK)//testuje napeti bat. OK
            {
                   step= READY;
                   LEDPORT &= LEDONEG;//obe LED zhasnuty
                   LEDZ= 1;
                   OUT_BRZDA=1; //vypnuti brzdy 
            }        
            else        //brzdeni naplno
            { 
               OUT_BRZDA=0;      //klidovym kontaktem 
            }
            break;
        case NADBAT:
            if(baterfil < UBATLOD)
            {
                step= READY;
                LEDPORT &= LEDONEG;//obe LED zhasnuty
                LEDZ= 1;
                OUT_BRZDA=1; //vypnuti brzdy                 
            }
            break;    
        case FUSE: //  spalena silova pojistka  
            if(probrzd!=0)//pro jistotu pauza na vyp. rele
                break;
            else
            if(fh.START_IN && (fusefil > UBATOCH))//napeti za sil. poj. obnoveno
            {
                OUT_BRZDA=1; //vypnuti brzdy 
                LEDPORT &= LEDONEG;//obe LED zhasnuty
                LEDZ= 1;
                step= READY;
            }
            else
   //         if(fil.BRZDA)
            {                   //brzdeni pri vypadku sil. pojistky
                  OUT_BRZDA=0;      //klidovym kontaktem               
            }
     //       else
     //       {
     //             OUT_BRZDA=1; //vypnuti brzdy 
     //       }
            break;

        default:
            LEDC= QBLIK;    //divny stav
             break;
       }     
     }
   }
   return (EXIT_SUCCESS);
}

