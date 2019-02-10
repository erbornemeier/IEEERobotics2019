#ifndef _DISPLAY_LETTERS_H
#define _DISPLAY_LETTERS_H

#include <Colorduino.h>
enum LETTERS{A,B,C,D,E,F};

void loading_screen(){

  uint8_t letter_m[] = {
                    0x11,
                    0x12,
                    0x13,
                    0x14,
                    0x15,
                    0x16,
                    0x25,
                    0x34,
                    0x44,
                    0x55,
                    0x66,
                    0x65,
                    0x64,
                    0x63,
                    0x62,
                    0x61
                      };
  
    //blue on
    for (int i = 0; i < 16; i++){
        Colorduino.FlipPage();
        int x = letter_m[i]&0x0F;
        int y = letter_m[i]>>4;
        Colorduino.SetPixel(x,y, 0x21, 0x31, 0x8d);
        Colorduino.FlipPage();
        delay(50);
        
    }
    delay(1000);
    
    //blue off
    for (int i = 0; i < 16; i++){
        Colorduino.FlipPage();
        int x = letter_m[15-i]&0x0F;
        int y = letter_m[15-i]>>4;
        Colorduino.SetPixel(x,y, 0x0, 0x0, 0x0);
        Colorduino.FlipPage();
        delay(50);
        
    }
}


void _draw_array(uint8_t* a){
    while (a[0] != 0xFF){
        Colorduino.SetPixel(a[0]>>4, a[0]&0x0F, 0x20, 0x20, 0x20);
        a++;
    }
}

void displayLetter(int letter){
    
uint8_t letter_a[] = {0x01,
                     0x11,
                     0x21,
                     0x31,
                     0x41,
                     0x51,
                     0x62,
                     0x63,
                     0x64,
                     0x55,
                     0x45,
                     0x35,
                     0x25,
                     0x15,
                     0x05,
                     0x32,
                     0x33,
                     0x34,
                     0xFF};

uint8_t letter_b[] = {0x01,
                     0x11,
                     0x21,
                     0x31,
                     0x41,
                     0x51,
                     0x61,
                     0x62,
                     0x63,
                     0x64,
                     0x55,
                     0x45,
                     0x34,
                     0x33,
                     0x32,
                     0x25,
                     0x15,
                     0x04,
                     0x03,
                     0x02,
                     0xFF};

uint8_t letter_c[] = {0x11,
                     0x21,
                     0x31,
                     0x41,
                     0x51,
                     0x62,
                     0x63,
                     0x64,
                     0x55,
                     0x51,
                     0x04,
                     0x03,
                     0x02,
                     0xFF };   

uint8_t letter_d[] = { 0x01,
                     0x11,
                     0x21,
                     0x25,
                     0x31,
                     0x35,
                     0x45,
                     0x41,
                     0x51,
                     0x61,
                     0x62,
                     0x63,
                     0x64,
                     0x55,
                     0x15,
                     0x04,
                     0x03,
                     0x02,
                     0xFF };

uint8_t letter_e[] = {0x01,
                     0x05,
                     0x65,
                     0x11,
                     0x21,
                     0x31,
                     0x41,
                     0x51,
                     0x61,
                     0x62,
                     0x63,
                     0x64,
                     0x34,
                     0x33,
                     0x33,
                     0x32,
                     0x04,
                     0x03,
                     0x02,
                     0xFF};

uint8_t letter_f[] = {0x01,
                     0x65,
                     0x11,
                     0x21,
                     0x31,
                     0x32,
                     0x41,
                     0x51,
                     0x61,
                     0x62,
                     0x63,
                     0x64,
                     0x34,
                     0x33,
                     0x33,
                     0xFF};


  
    Colorduino.FlipPage();
    Colorduino.ColorFill(0,0,0);
  
    switch(letter){
        case A:
          _draw_array(letter_a);
          break;
        case B:
          _draw_array(letter_b);
          break;
        case C:
          _draw_array(letter_c);
          break;
        case D:
          _draw_array(letter_d);
          break;
        case E:
          _draw_array(letter_e);
          break;
        case F:
          _draw_array(letter_f);
          break;
        default:
          break;
    }

    Colorduino.FlipPage();
}

#endif
