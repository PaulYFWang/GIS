#include "../inc/ColorPalette.h"

Colors::Colors(){
    t_color CUSTOM_WHITE(255,255,255,255);
    t_color PASTEL_GREEN(179,255,171,255);
    t_color PASTEL_BLUE(195,228,253,255);
    t_color WARM_GRAY(211,211,211,255);
    t_color PASTEL_YELLOW(255,255,153,255);
    t_color PASTEL_LIGHT_YELLOW(255,255,160,255);
    t_color DARK_GRAY(140,140,140,255);
    t_color LIGHT_GRAY_TRANS(251,251,251,160);
    t_color YELLOW_ORANGE(245,200,66,255);
    t_color STREET_RED(237, 149, 149,255);
    t_color STREET_DARK_ORANGE(237, 173, 149,255);
    t_color STREET_ORANGE(237, 198, 149,255);
    t_color STREET_REGULAR(255,255,170,255);
    
    background = CUSTOM_WHITE;
    street_1 = STREET_RED;
    street_2 = STREET_DARK_ORANGE;
    street_3 = STREET_ORANGE;
    street_4 = STREET_REGULAR;
    street_outline = STREET_DARK_ORANGE;
    street_highlight = PASTEL_BLUE;
    greenery = PASTEL_GREEN;
    water = PASTEL_BLUE;
    sand = PASTEL_YELLOW;
    building = WARM_GRAY;
    building_shadow = DARK_GRAY;
    textbox = t_color(240,240,240);
    textbox_highlight = PASTEL_YELLOW;
    text = t_color(0,0,0);
}

Colors::Colors(t_color _background, t_color _street_1, t_color _street_2, t_color _street_3,
    t_color _street_4, t_color _street_outline, t_color _street_highlight, t_color _greenery, t_color _water,
    t_color _sand, t_color _building, t_color _building_shadow, t_color _textbox,
    t_color _textbox_highlight, t_color _text){
    background = _background;
    street_1 = _street_1;
    street_2 = _street_2;
    street_3 = _street_3;
    street_4 = _street_4;
    street_outline = _street_outline;
    street_highlight = _street_highlight;
    greenery = _greenery;
    water = _water;
    sand = _sand;
    building = _building;
    building_shadow = _building_shadow;
    textbox = _textbox;
    textbox_highlight = _textbox_highlight;
    text = _text;
}

ColorPalette::ColorPalette(){
  Colors new_palette;
  palette_colors = new_palette;
  palette_name = "default";
}

ColorPalette::ColorPalette(Colors new_palette, string new_name){
  palette_colors = new_palette;
  palette_name = new_name;
}

ColorPalette::ColorPalette(PaletteCode palette_code){
    t_color CUSTOM_WHITE(255,255,255,255);
    t_color PASTEL_GREEN(179,255,171,255);
    t_color PASTEL_BLUE(195,228,253,255);
    t_color WARM_GRAY(211,211,211,255);
    t_color PASTEL_YELLOW(255,255,153,255);
    t_color PASTEL_LIGHT_YELLOW(255,255,160,255);
    t_color DARK_GRAY(140,140,140,255);
    t_color LIGHT_GRAY_TRANS(251,251,251,160);
    t_color YELLOW_ORANGE(245,200,66,255);
    t_color STREET_RED(237, 149, 149,255);
    t_color STREET_DARK_ORANGE(237, 173, 149,255);
    t_color STREET_ORANGE(237, 198, 149,255);
    t_color STREET_REGULAR(219,216,157,255);
    
  if(palette_code == CARIBBEAN){
    Colors new_palette(t_color(250,255,245),t_color(220,145,15),t_color(235,170,65),
      t_color(250,200,110),t_color(250,220,170),t_color(115,140,80),t_color(255,205,0),
      t_color(180,245,90),t_color(80,215,200),t_color(250,240,170),t_color(100,150,160),
      t_color(60,100,100),t_color(180,245,90),t_color(255,240,153),t_color(14,74,43));
    palette_colors = new_palette;
    cout << palette_name << endl;
  }
  else if(palette_code == CUTE){
    Colors new_palette(t_color(235,235,235),t_color(255,166,144),t_color(255,188,119),
      t_color(255,245,176),t_color(255,226,239),t_color(255,255,255),t_color(255,205,0),
      t_color(158,232,178),t_color(158,215,232),t_color(205,192,149),t_color(225,200,205),
      t_color(205,155,180),t_color(255,129,184),t_color(243,205,253),t_color(94,18,51));
    palette_colors = new_palette;
    palette_name = "Cute";
    cout << palette_name << endl;
  }
  else if(palette_code == DUSK){
    Colors new_palette(PASTEL_GREEN,PASTEL_GREEN,PASTEL_GREEN,PASTEL_GREEN,PASTEL_GREEN,
      PASTEL_GREEN,PASTEL_GREEN,PASTEL_GREEN,PASTEL_GREEN,PASTEL_GREEN,PASTEL_GREEN,PASTEL_GREEN,
      PASTEL_GREEN,PASTEL_GREEN,PASTEL_GREEN);
    palette_colors = new_palette;
    palette_name = "Dusk";
    cout << palette_name << endl;
  }
  else{
    Colors new_palette;
    palette_colors = new_palette;
    palette_name = "default";
    //cout << palette_name << endl;
  }
}

ColorPalette::ColorPalette(t_color background, t_color street_1, t_color street_2, t_color street_3,
  t_color street_4, t_color street_outline, t_color street_highlight, t_color greenery, t_color water,
  t_color sand, t_color building, t_color building_shadow, t_color textbox,
  t_color textbox_highlight, t_color text, string new_name){
    Colors new_palette(background,street_1,street_2,street_3,street_4,street_outline,
    street_highlight,greenery,water,sand,building,building_shadow,textbox,
    textbox_highlight,text);
    palette_colors = new_palette;
    palette_name = new_name;
}

const Colors& ColorPalette::get_colors(){
  return palette_colors;
}

const string& ColorPalette::get_name(){
  return palette_name;
}

void ColorPalette::set_name(string new_name){
  palette_name = new_name;
}

CustomPalette::CustomPalette(){
}

void CustomPalette::set_background(const t_color& new_color){
  palette_colors.background = new_color;
}

void CustomPalette::set_street_1(const t_color& new_color){
  palette_colors.street_1 = new_color;
}

void CustomPalette::set_street_2(const t_color& new_color){
  palette_colors.street_2 = new_color;
}
void CustomPalette::set_street_3(const t_color& new_color){
  palette_colors.street_3 = new_color;
}

void CustomPalette::set_street_4(const t_color& new_color){
  palette_colors.street_4 = new_color;
}

void CustomPalette::set_street_outline(const t_color& new_color){
    palette_colors.street_outline = new_color;
}

void CustomPalette::set_street_highlight(const t_color& new_color){
  palette_colors.street_highlight = new_color;
}

void CustomPalette::set_greenery(const t_color& new_color){
  palette_colors.greenery = new_color;
}

void CustomPalette::set_water(const t_color& new_color){
  palette_colors.water = new_color;
}

void CustomPalette::set_sand(const t_color& new_color){
  palette_colors.sand = new_color;
}

void CustomPalette::set_building(const t_color& new_color){
  palette_colors.building = new_color;
}

void CustomPalette::set_building_shadow(const t_color& new_color){
  palette_colors.building_shadow = new_color;
}

void CustomPalette::set_textbox(const t_color& new_color){
  palette_colors.textbox = new_color;
}

void CustomPalette::set_textbox_highlight(const t_color& new_color){
  palette_colors.textbox_highlight = new_color;
}

void CustomPalette::set_text(const t_color& new_color){
  palette_colors.text = new_color;
}
