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

class TextInputBox{
    
    public:
        double Tx, Ty, Twidth, Theight, size, widthDiv, heightDiv, posXMult, posYMult;
        bool active;
        string curText;
        bool autoActive;
        bool suggestActive;
        bool visible;
        tuple <string,string,double,unsigned>  autocomplete;
        vector< tuple<string,string, double, unsigned> > textSuggestions;
        t_color fieldColor;
        t_color highlightColor;
        t_color edgeColor1, edgeColor2;
        TextInputBox();
        
//         TextInputBox(string init_text, bool _autoActive, bool _suggestActive, bool _active, ){
//             
//         }
        
        ~TextInputBox();
        
        void initConditions(double _posXMult, double _posYMult,double _widthDiv , double _heightDiv,  double _size , bool _active , bool _visible, bool _autoActive, bool _suggestActive, t_color _fieldColor = WHITE, t_color _highlightColor = PASTEL_YELLOW, t_color _edgeColor1 = WARM_GRAY, t_color _edgeColor2 = DARK_GRAY);
        
        void updateWorldConditions();
        
//        void initConditions(double _Tx = get_visible_world().left(), double _Ty=get_visible_world().top(),double _widthDiv = 4, double _heightDiv=20,  double _size = 15, bool _active = false, bool _autoActive = false ,bool _suggestActive = false){
//            active = _active;
//            widthDiv = _widthDiv;
//            heightDiv = _heightDiv;
//            Twidth = get_visible_world().get_width()/_widthDiv;
//            Theight = get_visible_world().get_height()/_heightDiv ;
//            Tx = _Tx;
//            Ty = _Ty;
//            size = _size;
//            curText = "Initials";
//            autoActive = _autoActive;
//            suggestActive = _suggestActive;
//            visible = true;
//        }
        void drawBox();
        
        void loadSuggestions(vector< tuple<string,string, double, unsigned> > _textSuggestions);
        
        void loadAutocomplete(tuple <string,string,double,unsigned>  _autocomplete);
        
        bool inputTaken(char c);
        
        void setActive(bool _active);
        
        void setAutoActive(bool _autoActive);
        
        void setSuggestActive(bool _suggestActive);
        
        void setVisible(bool _visible);
        
        bool clickedOn(float x, float y);
        
        bool menuClicked(float x, float y);
        
        int menuSelection(float x, float y);
    
};