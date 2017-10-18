#include "../inc/TextInputBox.h"


TextInputBox::TextInputBox(){
    active = false;
    visible = true;
    widthDiv = 4;
    heightDiv = 20;
    Twidth = get_visible_world().get_width()/widthDiv;
    Theight = get_visible_world().get_height()/heightDiv ;
    posXMult = 0;
    posYMult = 0;
    Tx = get_visible_world().left() + (get_visible_world().right()-get_visible_world().left())*posXMult;
    Ty = get_visible_world().top()- (get_visible_world().top()-get_visible_world().bottom())*posYMult;
    size = 15;
    curText = "";
    autoActive = false;
    suggestActive = false;
}

TextInputBox::~TextInputBox(){
}

void TextInputBox::initConditions(double _posXMult, double _posYMult,double _widthDiv , double _heightDiv,  double _size , bool _active , bool _visible, bool _autoActive, bool _suggestActive, t_color _fieldColor, t_color _highlightColor , t_color _edgeColor1, t_color _edgeColor2){
    edgeColor1 = _edgeColor1;
    edgeColor2 = _edgeColor2;
    fieldColor = _fieldColor;
    highlightColor = _highlightColor;
    active = _active;
    visible = _visible;
    widthDiv = _widthDiv;
    heightDiv = _heightDiv;
    Twidth = get_visible_world().get_width()/widthDiv;
    Theight = get_visible_world().get_height()/heightDiv ;
    posXMult = _posXMult;
    posYMult = _posYMult;
    Tx = get_visible_world().left() + (get_visible_world().right()-get_visible_world().left())*posXMult;
    Ty = get_visible_world().top()- (get_visible_world().top()-get_visible_world().bottom())*posYMult;
    size = _size;
    curText = "";
    autoActive = _autoActive;
    suggestActive = _suggestActive;
}

void TextInputBox::updateWorldConditions(){
    Twidth = get_visible_world().get_width()/widthDiv;
    Theight = get_visible_world().get_height()/heightDiv ;
    Tx = get_visible_world().left() + (get_visible_world().right()-get_visible_world().left())*posXMult;
    Ty = get_visible_world().top()- (get_visible_world().top()-get_visible_world().bottom())*posYMult;
}

void TextInputBox::drawBox(){
            
    if (visible){
            setcolor(gl_state.background_color);
            fillrect(Tx,Ty,Tx+Twidth,Ty-Theight);
            if (active)
                setcolor(highlightColor);
            else 
                setcolor(fieldColor);
            if (active)
                cout << "IS AVCTIVE!!" <<  endl;
            fillrect(Tx,Ty,Tx+Twidth,Ty-Theight);
            setcolor(edgeColor1);
            setlinewidth(2);
            drawline(Tx,Ty,Tx+Twidth,Ty);
            drawline(Tx,Ty,Tx,Ty-Theight);
            setcolor(edgeColor2);
            drawline(Tx,Ty-Theight,Tx+Twidth,Ty-Theight);
            drawline(Tx+Twidth,Ty,Tx+Twidth,Ty-Theight);
            //cout << "Visible height is " << get_visible_world().get_height() << " while height is " << Theight << endl;
            
            settextattrs(size,0);
            if (curText.length() >0){
                int textwidth, textheight;
                int textwidthA, textheightA;
                int cutoff=0;
                int cutoffAuto = 0;
                
                do{
                    const char* text = curText.substr(cutoff).c_str();  
                    int text_byte_length = strlen(text);

                    font_ptr current_font = gl_state.font_info.get_font_info(
                        gl_state.currentfontsize,
                        gl_state.currentfontrotation
                        );
                    XGlyphInfo extents;
                    XftTextExtentsUtf8(x11_state.display, current_font, reinterpret_cast<const FcChar8*>(text), text_byte_length, &extents);
                    textwidth = extents.width;
                    textheight = extents.height;
                    cutoff++;
                }while(textwidth*get_trans_coord().stow_xmult>Twidth && cutoff != curText.length());
                
                if (autoActive&&get<1>(autocomplete)!= "N"){
                        do{
                        const char* text = get<0>(autocomplete).substr(cutoffAuto).c_str();  
                        int text_byte_length = strlen(text);

                        font_ptr current_font = gl_state.font_info.get_font_info(
                            gl_state.currentfontsize,
                            gl_state.currentfontrotation
                            );
                        XGlyphInfo extents;
                        XftTextExtentsUtf8(x11_state.display, current_font, reinterpret_cast<const FcChar8*>(text), text_byte_length, &extents);
                        textwidthA = extents.width;
                        textheightA = extents.height;
                        cutoffAuto++;
                    }while(textwidthA*get_trans_coord().stow_xmult>Twidth && cutoffAuto != get<0>(autocomplete).length());
                }
                
                if(autoActive){
                    setcolor(DARK_GRAY);
                    drawtext(Tx, Ty -Theight/2 - textheight*get_trans_coord().stow_ymult/2, (cutoffAuto < 1)?(get<0>(autocomplete)):(get<0>(autocomplete)).substr(cutoffAuto-1),FLT_MAX, FLT_MAX,false);
                }
                
                setcolor(BLACK);
                drawtext(Tx, Ty -Theight/2- textheight*get_trans_coord().stow_ymult/2, (cutoff < 1)?curText:curText.substr(cutoff-1),FLT_MAX, FLT_MAX,false);
                
                if(suggestActive && !textSuggestions.empty()){
                    for (unsigned i = 0; i < textSuggestions.size() && active; i ++){

                        setcolor(fieldColor);
                        fillrect(Tx,Ty - Theight*(i+1),Tx+Twidth,Ty-Theight*(i+2));
//                        setcolor(edgeColor2);
//
//                        fillrect(Tx,Ty - Theight*(i+1),Tx+Twidth,Ty-Theight*(i+2));
                        setcolor(edgeColor1);
                        setlinewidth(2);
                        drawline(Tx,Ty - Theight*(i+1),Tx+Twidth,Ty - Theight*(i+1));
                        drawline(Tx,Ty - Theight*(i+1),Tx,Ty - Theight*(i+2));
                        setcolor(edgeColor2);
                        drawline(Tx,Ty - Theight*(i+1),Tx+Twidth,Ty - Theight*(i+1));
                        drawline(Tx+Twidth,Ty-Theight*(i+2),Tx+Twidth,Ty - Theight*(i+2));

                        setcolor(BLACK);
                        drawtext(Tx, Ty -Theight/2 - textheight*get_trans_coord().stow_ymult/2 - Theight*(i+1), get<0>(textSuggestions[i]) + " " + get<1>(textSuggestions[i]) ,FLT_MAX, FLT_MAX,false);

                    }
                }

                //drawline(x,0,x,100);
        }
    }
}

void TextInputBox::loadSuggestions(vector< tuple<string,string, double, unsigned> > _textSuggestions){
    if (suggestActive)
        textSuggestions = _textSuggestions;
    else
        cout << "Suggestions arent active" << endl;
}

void TextInputBox::loadAutocomplete(tuple <string,string,double,unsigned>  _autocomplete){
    if (autoActive)
        autocomplete = _autocomplete;
    else
        cout << "Autocomplete isnt active" << endl;
}

bool TextInputBox::inputTaken(char c){
    if (active){
        curText+=c;
        return true;
    }
    else 
        return false;

}

void TextInputBox::setActive(bool _active){
    active = _active;
}

void TextInputBox::setAutoActive(bool _autoActive){
    autoActive = _autoActive;
}

void TextInputBox::setSuggestActive(bool _suggestActive){
    suggestActive = _suggestActive;
}

void TextInputBox::setVisible(bool _visible){
    visible = _visible;
    if (!visible){
        active = false;
    }
        
}

bool TextInputBox::clickedOn(float x, float y){
    
    return (x >= Tx && x<=Tx+Twidth && y <= Ty && y >=(Ty-Theight*(1)) );
        
}

bool TextInputBox::menuClicked(float x, float y){
    return(active && suggestActive && x >= Tx && x<=Tx+Twidth &&y <= Ty-Theight && y >=(Ty-Theight*(1+textSuggestions.size())) );
}

int TextInputBox::menuSelection(float x, float y){
    if (menuClicked(x,y)){
        return ((int)((Ty-Theight-y)/(Theight))) ;
    }
    else return -1;
}