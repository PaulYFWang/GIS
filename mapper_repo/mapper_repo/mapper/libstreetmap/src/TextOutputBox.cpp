/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "../inc/TextOutputBox.h"

 TextOutputBox::TextOutputBox(){
    curText = "";
    visible = true;
    centered = false;
    widthDiv = 4;
    heightDiv = 3;
    TOwidth = get_visible_world().get_width()/widthDiv;
    TOheight = get_visible_world().get_height()/heightDiv ;
    posXMult = 0;
    posYMult = 0;
    TOx = get_visible_world().left() + (get_visible_world().right()-get_visible_world().left())*posXMult;
    TOy = get_visible_world().top()- (get_visible_world().top()-get_visible_world().bottom())*posYMult;
    size = 15;

 }
        
//         TextInputBox(string init_text, bool _autoActive, bool _suggestActive, bool _active, ){
//             
//         }
        
        TextOutputBox::~TextOutputBox(){
            
        }
        
        void TextOutputBox::initConditions(double _posXMult, double _posYMult,double _widthDiv , double _heightDiv, string _text, double _size ,  bool _visible, bool _centered, t_color _textColor,t_color _fieldColor, t_color _edgeColor1 , t_color _edgeColor2 ,t_color _xColor, t_color _xBoxColor){
            edgeColor1 = _edgeColor1;
            edgeColor2 = _edgeColor2;
            textColor = _textColor;
            fieldColor = _fieldColor;
            xColor = _xColor;
            xBoxColor = _xBoxColor;
            visible = _visible;
            widthDiv = _widthDiv;
            heightDiv = _heightDiv;
            TOwidth = get_visible_world().get_width()/widthDiv;
            TOheight = get_visible_world().get_height()/heightDiv ;
            posXMult = _posXMult;
            posYMult = _posYMult;
            TOx = get_visible_world().left() + (get_visible_world().right()-get_visible_world().left())*posXMult;
            TOy = get_visible_world().top()- (get_visible_world().top()-get_visible_world().bottom())*posYMult;
            size = _size;
            curText = _text;
            centered = _centered;
           
        }
        
       void TextOutputBox::updateWorldConditions(){
        TOwidth = get_visible_world().get_width()/widthDiv;
        TOheight = get_visible_world().get_height()/heightDiv ;
        TOx = get_visible_world().left() + (get_visible_world().right()-get_visible_world().left())*posXMult;
        TOy = get_visible_world().top()- (get_visible_world().top()-get_visible_world().bottom())*posYMult;
    }
        

        void TextOutputBox::drawOutputBox(){
              if (visible){
                setcolor(fieldColor);
                fillrect(TOx,TOy,TOx+TOwidth,TOy-TOheight);
                
                //setcolor(255,105,97,255);
                setcolor(xBoxColor);
                fillrect(TOx+TOwidth-TOwidth/20,TOy,TOx+TOwidth,TOy-TOwidth/20);
                setcolor(xColor);
                setlinewidth(3);
                drawline(TOx+TOwidth-TOwidth/20-2*get_trans_coord().stow_xmult,TOy-2*get_trans_coord().stow_xmult,TOx+TOwidth-2*get_trans_coord().stow_xmult,TOy-TOwidth/20-2*get_trans_coord().stow_xmult);
                drawline(TOx+TOwidth-TOwidth/20-2*get_trans_coord().stow_xmult,TOy-TOwidth/20-2*get_trans_coord().stow_xmult,TOx+TOwidth-2*get_trans_coord().stow_xmult,TOy-2*get_trans_coord().stow_xmult);
//                drawline(TOx + TOwidth/2,TOy,TOx + TOwidth/2,TOy - TOheight);
//                drawline(TOx ,TOy- TOheight/2,TOx + TOwidth,TOy - TOheight/2);
                setlinewidth(5);
                //setcolor(255,105,97,255);
                setcolor(xBoxColor);
                drawrect(TOx + TOwidth - TOwidth / 20-2*get_trans_coord().stow_xmult, TOy-2*get_trans_coord().stow_xmult, TOx + TOwidth-2*get_trans_coord().stow_xmult, TOy - TOwidth / 20-2*get_trans_coord().stow_xmult);
              setcolor(edgeColor1);
                setlinewidth(2);
                drawline(TOx,TOy,TOx+TOwidth,TOy);
                drawline(TOx,TOy,TOx,TOy-TOheight);
                setcolor(edgeColor2);
                drawline(TOx,TOy-TOheight,TOx+TOwidth,TOy-TOheight);
                drawline(TOx+TOwidth,TOy,TOx+TOwidth,TOy-TOheight);
                
                settextattrs(size,0);
                int textwidth=0, textheight=0;
                bool newLine =false;
                string substring ="";
                int counter = 0;
                int index = 0;
                while(index < curText.length()){
                    while(textwidth*get_trans_coord().stow_xmult<TOwidth*0.85 && index < curText.length()&&!newLine){
                        if (curText[index] != '\n')
                        substring.append(1,curText[index]);
                        else
                            newLine=true;
                        const char* text = substring.c_str();  
                        int text_byte_length = strlen(text);

                        font_ptr current_font = gl_state.font_info.get_font_info(
                            gl_state.currentfontsize,
                            gl_state.currentfontrotation
                            );
                        XGlyphInfo extents;
                        XftTextExtentsUtf8(x11_state.display, current_font, reinterpret_cast<const FcChar8*>(text), text_byte_length, &extents);
                        textwidth = extents.width;
                        if (textheight == 0)
                            textheight = extents.height;
                        index++;
                       // cout <<"INDEX AT" << index << endl;
                    }
                    setcolor(textColor);
                    if (centered)
                        drawtext(TOx+TOwidth*0.5, TOy-TOheight*0.01+get_trans_coord().stow_ymult*textheight/2.0+counter*1.05*textheight*get_trans_coord().stow_ymult,substring);
                    else
                    drawtext(TOx+TOwidth*0.05, TOy-TOheight*0.01+counter*1.05*textheight*get_trans_coord().stow_ymult,substring, FLT_MAX,FLT_MAX,false);
                    if (newLine){
                        //counter++;
                        newLine = false;
                    }
                    substring.erase();
                    textwidth = 0;
                    counter++;
                    
                }
            }
              
        }
        
        void TextOutputBox::setVisible(bool _visible){
          visible = _visible;   
        }
        
        bool TextOutputBox::clickedClosed(float x, float y){
            return (x >=TOx+TOwidth-TOwidth/20 && x <=TOx+TOwidth && y >=TOy-TOwidth/20 && y <=TOy);
        }
        
        void TextOutputBox::setText(string _text){
            curText = _text;
        }