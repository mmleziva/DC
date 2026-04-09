/* 
 * File:   DC.c
 * Author: mleziva
 *
 * Created on  2024
 */
//#define TEST
//#define ACU24V
//#define NUCENA   //zpatecka
#define MASTER   //
//#define TETA
//#define DRON
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
//  #pragma config PWRTE = OFF //!t
  #pragma config PWRTE = ON //!t
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
#define PWM_VZAD    RC2
#define BEZP_RELE   RC3	
#define MASTER_OUT  RC3	
#define VPRED       RC4
#define VZAD        RC5
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
#define NAP_OCHRA  RB2

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
#define READY_VZAD  4
#define GOVPRED    10
#define PWMVPRED   12
#define GOVZAD     20
#define PWMVZAD    22
#define BRZDENI    30
#define VYBITA     38
#define PO_NARAZU  40
#define OCHR_BAT   50
#define PREBITA    55
#define FUSE       60


            //napeti
#define DELPWM     3    //hystereze
#define PWMAX      251
#define PWMIN      (((PWMAX)*15)/100-DELPWM)   //15% PWMAX
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
#define UBAT30       (KBAT*119)// 12,5V (KBAT*115)// 12,0V
#define UBATLOD      (KBAT*137)// 14,5V
#define UBATHOD      (KBAT*140)// 15V
#define UBATPRE      (KBAT*152)// 16V
#define FUSEMIN      (KBAT*67)//6V  //n 

        //casy 1bit=2ms
#define ZPOZBAT    15000      //30s
#define TIMINC     50        //100ms//t1s
#define TIMSET     10        //20ms
#ifndef DRON
 //#define TRAMPINC   (900/(PWMAX-PWMIN))  //pridavani plynu 1,8s //nn
 //#define TRBRINC   (750/(PWMAX-BRZDMIN))  //pridavani brzdy 1,5s //tt
 #define TRAMPINC   (500/(PWMAX-PWMIN))  //pridavani plynu 2s //nn
 #define TRBRINC   (125/(PWMAX-BRZDMIN))  //pridavani brzdy 0,5s //tt
#else
 #define TRAMPINC   (700/(PWMAX-PWMIN))  //pridavani plynu 1,4s //nn
 #define TRBRINC   (400/(PWMAX-BRZDMIN))  //pridavani brzdy 0,8s //tt
#endif
#define NARINC   (250/(PWMAX-PWMIN))  //pridavani plynu naraz //nn
#define TIMAX      (150000UL)//1000ms *60 *5= 5 min.
#define PRODL     100       //200ms prodleva po konci brzdeni
#define PROVYP     25       //50ms prodleva po po vypnuti kontaktu
#define MOSVYP     10       //20ms prodleva po po vypnuti kontaktu
//filtered dig.inputs
union
{
    uint8_t B;
    struct
    {
      uint8_t NARAZ: 1;    
      uint8_t BRZDA: 1; 
      uint8_t       : 1; 
      uint8_t       : 1; 
      uint8_t        :1; 
      uint8_t START_IN: 1; 
      uint8_t       : 1; 
      uint8_t       : 1; 
    };
}
fil, fh;  // filtr, hrana vstupu

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

_Bool ENA_GO=0;
_Bool PO_BRZDENI;
_Bool PAUSE;// priznak pauza
_Bool POJISTKA;
_Bool ULOW, LBLIK, QBLIK, AKCEL_SEP, VYB;
_Bool ZPET=0, ZPETMEM=0;
uint8_t in[8], set, res, film ;//prom. fitru
uint8_t step, k, j ,lt,ltnaraz, prodleva,probrzd,startime;//krok programu, predch. krok,.., citac prodlevy 50 ms 
uint8_t plyn,blik,brac,pb;//prepoctena hod. akc.do  pwm,  odmer. blik.LED, stav brzd. pedalu, citac po brzdeni
uint16_t baterie,akcel,fuse,baterfil, akcelfil,fusefil,zpozdeni;//, naratim;;
uint32_t timvyp;
uint8_t *ptr, pulsmaster, nulmaster;
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

uint8_t pwmbrzd(uint16_t uak)// prepocet napeti akceleratoru na pwm brzdeni
{
    word uacc, resw;
    uint8_t huac, result, difhu;
    uacc.w= uak;
    huac= uacc.H;
    if(huac <= AKCMIN)
        result= BRZDMIN;
    else if(huac >= AKCMAX)
        result= PWMAX;
    else
    {
        difhu = huac-AKCMIN;
        resw.w= BRZDMUL * difhu;
        result=resw.H +BRZDMIN;
    }
    return result;
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
   TRISC=0x0;     //outs enable
   TRISBbits.TRISB2=0;
   PORTA= 0b00011100;
   OPTION_REGbits.nRBPU=0;// pull up
   #ifndef TEST
   TRISA= 0b11100011;
   RC0=1;
   #else
    TRISA= 0b11101111;//t
    TRISEbits.TRISE0=0;//!t
    TRISEbits.TRISE1=0;//!t
   #endif
   ADCON1bits.ADFM= 0;//left just. ADC
   step=INI;
    //init dig. filters
   film= ~PORTB;; //inverted inputs
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
   #ifdef MASTER
                MASTER_OUT=1;
                pulsmaster=50;//100ms puls MASTER_OUT
   #endif
   //infinited cycle
   while(1)
   {    
     CLRWDT();  //clear watchdog timer
     if(TMR1IF)//2ms cyklus
     {
         TMR1=-2000;
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
       akcel= adc_read(AKCELERATOR);
       baterie= adc_read(U_BATERIE);
       fuse= adc_read(U_FUSE);
       adc_filter( &akcel,&akcelfil);
       adc_filter( &baterie,&baterfil);
       adc_filter( &fuse,&fusefil);
       LBLIK= ((blik & 0x80) != 0);//priznak blikani
       QBLIK= ((blik & 0x20) != 0);//priznak blikani 4x rychleji
       blik++;
       if(probrzd >0)//prodleva  vypinani rele
       {
        probrzd++;
        if(probrzd >(PROVYP+MOSVYP))// 50ms + 20ms
           probrzd=0;
        else
        if(probrzd > MOSVYP)
        {
            VPRED=0;
            VZAD= 0;
        }
       }
       AKCEL_SEP=(akcelfil > AKCMIN16b);
#ifndef MASTER 
       BEZP_RELE= AKCEL_SEP;     
#else
       if(pulsmaster > 0)
       {
           pulsmaster--;
          if(pulsmaster==0)
          {
              MASTER_OUT=0;
              nulmaster=50;
          }
       }    
       else
       if(nulmaster > 0)
       {
           nulmaster--;
          if(nulmaster==0)
          {
              MASTER_OUT=ZPETMEM;              
          }  
       }
#endif
#ifdef DRON
        if((step > READY_VZAD) && (step != BRZDENI))
#else
       if(step > READY_VZAD)
#endif
               ENA_GO=0;    
                //prioritni akce
       if(startime< TIMINC)
       {
         startime++;
         step= INI;
       }      
       else
       if(AKCEL_SEP && (step != FUSE)&& (fusefil< FUSEMIN)) //sledovani stavu silove pojistky
       {
             LEDPORT &= LEDONEG;      //zhasni obe LED
             step= FUSE;
             probrzd=1; //zapina odpocitani prodlevy pro vypinani rele
             LEDC=1;  //rozsvit cervenou
 
       }
       else
       //if(fil.NARAZ && (step != PO_NARAZU)&&(step != GOVPRED)&& (step != PWMVPRED))  //pri narazu vzad
       if(fil.NARAZ && (step != PO_NARAZU) && ZPET)  //pri narazu vzad
       {
#ifdef MASTER
           MASTER_OUT=0;
#endif
         LEDPORT &= LEDONEG;      //zhasni obe LED  //tt
         probrzd=1;//zapina odpocitani prodlevy pro vypinani rele
         CCPR1L=0;
         CCPR2L=0;
         ltnaraz=0;
         LEDZ=1;    //tt
         step= PO_NARAZU; 
       }   
#ifndef DRON
       else 
       if((baterfil < UBATOCH) && (step != OCHR_BAT))   //podpetova ochrana baterie  
       {
             LEDPORT &= LEDONEG;      //zhasni obe LED   //nn
             probrzd=1;//zapina odpocitani prodlevy pro vypinani rele
             LEDC= 1;   //n
             step= OCHR_BAT;
       }
#endif
       else
       if(fil.BRZDA && ((step < BRZDENI)||(step==VYBITA)))     //pouziti  brzdy , ve stavu po brzdeni   
       {
           probrzd=1;//zapina odpocitani prodlevy pro vypinani rele           
           CCPR1L= 0;
           CCPR2L= 0;
           LEDPORT &= LEDONEG;//led zhasnuty
           LEDC=1;
           fh.START_IN=0;
           MASTER_OUT=0;
           step= BRZDENI;
        }
       else
       if(ULOW && (step < BRZDENI))    //blikani cervene pri vybite bat.
       {                
              CCPR1L=0;
              CCPR2L=0;
              if(step != VYBITA)
              {
                step= VYBITA;
                LEDPORT &= LEDONEG;//led zhasnuty //t
              }
              probrzd=1;
        }
       
#ifndef DRON
       else
       if((baterfil > UBATPRE)&& (step != PREBITA)&&(step != PO_NARAZU))
       {
           probrzd=1;
           step= PREBITA;
           LEDPORT &= LEDONEG;//led zhasnuty //n
           LEDC=1;//n
       }
#else 
#endif
       else
       if(step != BRZDENI)
           if(plyn ==0)//hystereze
           {
             plyn = pwmakc(akcelfil);// jinak se meri pedal plynu
             if(plyn< (PWMIN+DELPWM))
                     plyn=0;
           }
           else
              plyn = pwmakc(akcelfil);// jinak se meri pedal plynu 
       else
           plyn=0;
       if(baterfil > UBATHOD)   //napetova ochrana
       {
           NAP_OCHRA=1;
       }
       else if(baterfil < UBATLOD)
       {
           NAP_OCHRA=0;
       };
     //  if((step < BRZDENI) || (step == PO_NARAZU))
       if(!ULOW)
       {
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
           if(VYB) 
           {
               if(baterfil > UBATVYB)
               {
                   zpozdeni=0;
                   VYB=0;
               }
               else
                    zpozdeni ++;
           }
           else
           if(baterfil < UBATVYB)
             VYB=1;
               
           else           
             zpozdeni=0;  
       }
   //    else
   //        zpozdeni=0;
       
       if( AKCEL_SEP )   //test oddaleni vypnuti elektroniky
       {
           timvyp=0;
       }
       else 
       if(timvyp > TIMAX)// test vypnuti po 5 minutach
       {
       //  OUT_START=0;
              startime=0;
              PORTA=0x0;
              TRISA=0b11111111;
              PORTC=0x0;
              TRISC=0b11111111;
       }
       else
       {
         //     OUT_START=1;
#ifndef  TEST
           PORTA=0b00011100;
           TRISA=0b11100011;
           RC0=1;
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
        if(startime< TIMINC)
        {
           step= INI;
        }
        else
        {
            step= READY;
            LEDZ=1;
        }
        break;
        
        case READY: //pripraven pro rozjezd
            if(!AKCEL_SEP) ENA_GO= 1;
            if(baterfil < UBATVYB)
                VYB=1;
            if(PO_BRZDENI)  //nastaveno pri skonceni brzdeni
            {
#ifdef DRON              
                pb++;
                if(pb > PRODL)  //po prodleve... 
                {
                  pb=0;
                  PO_BRZDENI=0;
                  if(fil.START_IN) //...snimani tlacitka kvuli vyhodnoceni zpatecky
                  {
                    LEDPORT |= LEDORAN;   //rozsviti zelenou i rudou LED
                    step= READY_VZAD;//a prepne na pripraven pro zpatecku
                  }
                }                
  #else  
                PO_BRZDENI=0;
  #endif                
            }
            else if(fh.START_IN)     //startovaci spinac
            {
             LEDPORT |= LEDORAN;   //rozsviti zelenou i rudou LED
             step= READY_VZAD;//a prepne na pripraven pro zpatecku
             ZPET=1;
#ifdef MASTER
            MASTER_OUT=1;
#endif
            }
            else if(ENA_GO &&(plyn > 0))
            {
             ENA_GO= 0;   
             step= GOVPRED;   
             prodleva=0;
            }
            break;
            
        case READY_VZAD: //pripraven pro zpatecku
            if(!AKCEL_SEP) ENA_GO= 1;
            if(baterfil < UBATVYB)//t
                VYB=1;
#if defined (NUCENA) 
 //|| defined (MASTER)            
            if(!fil.START_IN)
#else
            if(fh.START_IN)
#endif
            {
#ifdef MASTER
                    MASTER_OUT=0;
#endif
             LEDPORT &= LEDONEG;//led zhasnuty
             step= READY;
             ZPET=0;
             LEDZ=1;
            }
            else 
            if(ENA_GO &&(plyn > 0))
            {
             ENA_GO= 0;      
             step= GOVZAD; 
             prodleva=0;
            }
            break;
                       
       case VYBITA:
            if(baterfil> UBAT30)//testuje napeti bat. OK
            {
                LEDPORT &= LEDONEG;//obe LED zhasnuty
                CCPR1L=0;
                CCPR2L=0;
                //VYB=0; //t
                ULOW=0;
                if(ZPET)
                {
                    LEDPORT |= LEDORAN;//oranz sviti
                    step= READY_VZAD;
                } 
                else
                {
                   LEDPORT &= LEDONEG;//obe LED zhasnuty
                   LEDZ= 1;
                   step= READY;
                }                   
            } 
            else
                LEDC=LBLIK;
            break;   

       case GOVPRED: //prodleva 100ms pred rozjezdem
            prodleva++;
            if(prodleva== TIMSET)
            {
                VZAD=0;
                VPRED= 1;
            }
            else
            if(prodleva> TIMINC)
            {
                step = PWMVPRED;
                CCPR2L= PWMIN;
                CCPR1L= 0;
            }
            break;
        case PWMVPRED: //plyn vpred
             zpozdeni=0; //zacina pocitat zpozdeni pro test vybite bat.
             if(plyn == 0)  //po uvolneni plynu navrat na ready
             {
                 CCPR2L=0;
                 step= READY;   
                 LEDPORT &= LEDONEG;//oranz zhasnuta
                 LEDZ= 1;
                 probrzd=1;
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
             break;

        case GOVZAD: //prodleva 50ms pred zpateckou

            prodleva++;
            if(prodleva== TIMSET)
            {
                VZAD=1;
                VPRED= 0;
            }
            else
            if(prodleva> TIMINC)
            {
                step = PWMVZAD;
                CCPR1L= PWMIN;
                CCPR2L= 0; 
            }
            break;
        case PWMVZAD://plyn zpatecky
             zpozdeni=0; //zacina pocitat zpozdeni pro test vybite bat.
             if(plyn == 0)
             {
                 CCPR1L=0;
                 probrzd=1;
                 step= READY_VZAD;  
                 LEDPORT |= LEDORAN;
             }
#if defined (NUCENA)             
             else
                 if(!fil.START_IN)
                 {
                     CCPR1L=0;
                     VZAD=0;
                     LEDPORT &= LEDONEG;
                     LEDZ=1;
                     step= READY;  
                     ZPET=0;
                  }
#endif
             else
             {
             if((plyn >= PWMAX)&&(CCPR1L >= plyn))// plyn nadoraz:LED sviti trvale
              {
                     LEDPORT |= LEDORAN;
              }
              else
              {
                if(QBLIK)      //obe led rychle blikaji
                        LEDPORT |= LEDORAN;//oranz sviti
                else
                        LEDPORT &= LEDONEG;//oranz zhasnuta
                if(plyn > CCPR1L)//POSTUPNE PRIDAVA PLYN
                {
                 lt++;
                 if(lt > TRAMPINC)
                 {
                     lt=0;
                     CCPR1L++;
                 }
                }
                else  
                    if(plyn < PWMAX)    //nn
                      CCPR1L= plyn; //nepridava plyn, resp. okamzite ubira
               }
              }
              break;
         
        case BRZDENI: //
#ifdef TETA
            if(!fil.BRZDA) //konec  brzdy  //tt
            {
                CCPR1L= 0;
                CCPR2L= 0;
                LEDPORT &= LEDONEG;//led zhasnuty
   //             LEDZ=1;
   //             step= READY;
   //             ZPET=0;
                if(!ZPET)
                {
                   step= READY;
                   LEDZ=1;
                }
                else
                {
                   step= READY_VZAD;
                   LEDPORT |= LEDORAN;                   
                }

            }
            else
            if(probrzd==0)    //brzdeni akceleratorem
            {   
             if(CCPR2L >= PWMAX) // brzda nadoraz neblika
             {
                   LEDC=1;
             }
             else
             {
               if(!AKCEL_SEP)
                        LEDC= 1;
               else
                        LEDC= QBLIK;
              lt++;
              if(lt > TRAMPINC)
              {
                     lt=0;
                     CCPR2L++;
                     CCPR1L= CCPR2L;
               }
             }
            }
#else
    #ifndef DRON
            if((fh.START_IN)&&(!AKCEL_SEP)) //konec  brzdy  //tt
    #else
             ENA_GO = 1;
             if((fh.START_IN) && !fil.BRZDA ) //konec  brzdy  //tt
    #endif                    
            {
                CCPR1L= 0;
                CCPR2L= 0;
                LEDPORT &= LEDONEG;//led zhasnuty
                PO_BRZDENI=1;
                pb=0;
#ifdef MASTER
                    ZPETMEM= ZPET;
                    MASTER_OUT=1;
                    pulsmaster=50;//100ms puls MASTER_OUT
#endif

                if(!ZPET)
                {
                   step= READY;
                   LEDZ=1;
                }
                else
                {
                   step= READY_VZAD;
                   LEDPORT |= LEDORAN;                 
                }
            }
            else
            if(probrzd==0)    //brzdeni akceleratorem
            {      
                 brac= pwmbrzd(akcelfil);
             if((brac >= PWMAX)&&(CCPR2L >= brac)) // brzda nadoraz neblika
             {
                     LEDC=1;
             }          
             else 
             {
                 
#ifdef DRON
                     LEDC= QBLIK;
#else
                     if(!AKCEL_SEP)
                         LEDC= 1;
                     else
                        LEDC= QBLIK;
#endif
                     if(brac > CCPR2L)//POSTUPNE PRIDAVA brzdu
                     {
                        lt++;
                        if(lt > TRBRINC)
                        {
                            lt=0;
                            CCPR2L++;
                        }
                    }
                    else
                     CCPR2L= brac;
             }
             CCPR1L= CCPR2L;
            }
#endif
            break;
        case PO_NARAZU: //  
            if(CCPR2L >= PWMAX)
            {
              CCPR2L= 0;
              probrzd=1;    //zapina odpocitani prodlevy pro vypinani rele
              LEDPORT &= LEDONEG;//obe LED zhasnuty
              LEDZ= 1;
              step= READY;
              ZPET=0;
             }
            else
            if(probrzd==0)
            {
                 if(CCPR2L< PWMIN)
                 {          
                    CCPR2L= PWMIN;
                    VPRED=1;        //n
                 }
                 ltnaraz++;
                 if(ltnaraz >NARINC)    //nn
                 {
                     ltnaraz=0;
                     CCPR2L++;
                 }
                 LEDZ=QBLIK;
            }          
            break;

        case OCHR_BAT:
            if(baterfil> UBATOK)//testuje napeti bat. OK
            {
                LEDPORT &= LEDONEG;//obe LED zhasnuty
                CCPR1L=0;
                CCPR2L=0;
                VYB=0; //t
                if(ZPET)
                {
                    LEDPORT |= LEDORAN;//oranz sviti
                    step= READY_VZAD;
                } 
                else
                {
                   LEDPORT &= LEDONEG;//obe LED zhasnuty
                   LEDZ= 1;
                   step= READY;
                }                   
            } 
            else        //brzdeni naplno
            { 
              if((CCPR1L != PWMAX)&&(probrzd==0))
               CCPR1L= PWMAX;
             if((CCPR2L != PWMAX)&&(probrzd==0))
               CCPR2L= PWMAX;
            }
            break;
        case PREBITA:
            if(baterfil< UBATLOD)
            {
                CCPR2L=0; //n
                CCPR1L=0;
                if(ZPET)
                {
                    LEDPORT |= LEDORAN;//oranz sviti
                    step= READY_VZAD;
                }
                else
                {
                   LEDPORT &= LEDONEG;//obe LED zhasnuty
                   LEDZ= 1;
                   step= READY;
                }
            }
            else
            {
             if((CCPR1L != PWMAX)&&(probrzd==0))
               CCPR1L= PWMAX;
             if((CCPR2L != PWMAX)&&(probrzd==0))
               CCPR2L= PWMAX;
            }
            break;               
        case FUSE: //  spalena silova pojistka  
            if(probrzd!=0)//pro jistotu pauza na vyp. rele
                break;
            else
            if(fh.START_IN && (fusefil > UBATOCH))//napeti za sil. poj. obnoveno
            {
                CCPR1L= 0;
                CCPR2L= 0;
 #ifdef MASTER
                MASTER_OUT=0;
 #endif
                LEDPORT &= LEDONEG;//obe LED zhasnuty
                LEDZ= 1;
                step= READY;
                ZPET=0;
            }
            else
            if(AKCEL_SEP)
            {                   //brzdeni pri vypadku sil. pojistky
                 CCPR1L= PWMAX;//PWMAX;
                 CCPR2L= PWMAX;//              
            }
            else
            {
                 CCPR1L= 0;
                 CCPR2L= 0;
            }
            break;

        default:
            LEDC= QBLIK;    //divny stav
             break;
       }    
       if(CCPR2L > 0)
           VZAD=0;
       if(CCPR1L > 0)
           VPRED=0;
     }
   }
   return (EXIT_SUCCESS);
}

