/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TextOutputBox.h
 * Author: wangy384
 *
 * Created on March 19, 2017, 9:54 PM
 */

#ifndef TEXTOUTPUTBOX_H
#define TEXTOUTPUTBOX_H
#include <iostream>
#include <string>
#include <tuple>
#include <vector>
#include "graphics.h"
#include "graphics_types.h"
#include "../inc/m1_globals.h"
#include "../inc/m2_graphics.h"
#include "../inc/ColorPalette.h"
#include "../src/easygl/graphics_state.h"
#include "../src/easygl/fontcache.h"

using namespace std;

extern t_x11_state x11_state;
extern t_gl_state gl_state;

class TextOutputBox{
    
    public:
        double TOx, TOy, TOwidth, TOheight, size, widthDiv, heightDiv, posXMult, posYMult;
        string curText;
        bool visible;
        bool centered;
        t_color fieldColor,textColor;
        t_color edgeColor1, edgeColor2;
        t_color xColor,xBoxColor;
        TextOutputBox();
        
//         TextInputBox(string init_text, bool _autoActive, bool _suggestActive, bool _active, ){
//             
//         }
        
        ~TextOutputBox();
        
        void initConditions(double _posXMult, double _posYMult,double _widthDiv , double _heightDiv, string _text, double _size ,  bool _visible,bool _centered = false, t_color _textColor = BLACK, t_color _fieldColor = WHITE, t_color _edgeColor1 = WARM_GRAY, t_color _edgeColor2 = DARK_GRAY,t_color _xColor = DARK_GRAY,t_color _xBoxColor = WARM_GRAY);
        
        void updateWorldConditions();
    
        void drawOutputBox();
        
        void setVisible(bool _visible);
        
        bool clickedClosed(float x, float y);
        
        void setText(string _text);
    
};


#endif /* TEXTOUTPUTBOX_H */

