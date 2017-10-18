/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Button.h
 * Author: wangy384
 *
 * Created on March 18, 2017, 5:07 PM
 */


#pragma once

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

class Button{
    
    public:
        double Bx, By, Bwidth, Bheight, size, widthDiv, heightDiv, posXMult, posYMult;
        bool active;
        string text;
        bool visible;
        bool isClicked  =false;
        //Surface s;
        double borderMult;
        t_color outer, inner;
        
        Button();
        
//         TextInputBox(string init_text, bool _autoActive, bool _suggestActive, bool _active, ){
//             
//         }
        
        ~Button();
        
        void updateWorldConditions();
        
        void initConditions(double _posXMult, double _posYMult,double _widthDiv , double _heightDiv,  double _size , bool _active , bool _visible,string _text,double _border, t_color _outer = PASTEL_BLUE, t_color _inner = WHITE );
        void drawButton();
        
        void setActive(bool _active);
        
        void setVisible(bool _visible);
        
        bool clicked(float x, float y);
        void setText(string s);
        
    
};



