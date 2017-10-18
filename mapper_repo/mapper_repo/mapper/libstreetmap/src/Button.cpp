/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "../inc/Button.h"

Button::Button(){
    active = true;
    widthDiv = 20;
    heightDiv = 30;
    Bwidth = get_visible_world().get_width()/widthDiv;
    Bheight = get_visible_world().get_height()/heightDiv ;
    posXMult = 0;
    posYMult = 0;
    
    Bx = get_visible_world().left() + posXMult*(abs(get_visible_world().left()- get_visible_world().right()));
    By = get_visible_world().top() - posYMult *(abs(get_visible_world().top()-get_visible_world().bottom()));
    
    borderMult = 2;
    size = 10;
    text = "Default";
    visible = true; 
}
        
//         TextInputBox(string init_text, bool _autoActive, bool _suggestActive, bool _active, ){
//             
//         }
       
Button::~Button(){

}
void Button::updateWorldConditions(){
    Bwidth = get_visible_world().get_width()/widthDiv;
    Bheight = get_visible_world().get_height()/heightDiv ;
    Bx = get_visible_world().left() + posXMult*(abs(get_visible_world().left()- get_visible_world().right()));
    By = get_visible_world().top() - posYMult *(abs(get_visible_world().top()-get_visible_world().bottom()));
}

void Button::initConditions(double _posXMult, double _posYMult,double _widthDiv , double _heightDiv,  double _size , bool _active , bool _visible,string _text,double _border, t_color _outer , t_color _inner){
    outer = _outer;
    inner = _inner;
    active = _active;
    widthDiv = _widthDiv;
    heightDiv = _heightDiv;
    Bwidth = get_visible_world().get_width()/widthDiv;
    Bheight = get_visible_world().get_height()/heightDiv ;
    posXMult = _posXMult;
    posYMult = _posYMult;
    Bx = get_visible_world().left() + posXMult*(abs(get_visible_world().left()- get_visible_world().right()));
    By = get_visible_world().top() - posYMult *(abs(get_visible_world().top()-get_visible_world().bottom()));
    borderMult = _border;
    size = _size;
    text = _text;
    visible = _visible;
}

void Button::drawButton(){
    if (visible){
        if(isClicked)
            setcolor(inner);
        else
            setcolor(outer);
        
        fillrect(Bx-borderMult*get_trans_coord().stow_xmult,By+borderMult*get_trans_coord().ps_ymult, Bx+Bwidth+borderMult*get_trans_coord().stow_xmult,By -Bheight-borderMult*get_trans_coord().ps_ymult);
        
         if(isClicked)
            setcolor(outer);
        else 
            setcolor(inner);
        fillrect(Bx,By,Bx+Bwidth,By-Bheight);
        settextattrs(size,0);
        setcolor(BLACK);
        drawtext(Bx + Bwidth/2,By-Bheight/2, text);
        //cout <<"Width is " << Bwidth <<"compared to screen " << get_visible_world().get_width() <<  " with divisor of " << widthDiv << endl; 
    }
        
}

void Button::setActive(bool _active){
    active = _active;
    if (active)
        visible = true;
}

void Button::setVisible(bool _visible){
    visible = _visible;
    active = _visible;
}

void Button::setText(string s){
    text = s;
}
bool Button::clicked(float x, float y){
    if (active)
        return (x >= Bx && x <= Bx + Bwidth && y <= By && y >= By - Bheight);
    else
        return false;
}
