#ifndef COLOR_PALETTE_H
#define COLOR_PALETTE_H

#include <string>
#include "graphics.h"

using namespace std;

//colors go here
//extern t_color CUSTOM_BLACK;
//extern t_color CUSTOM_WHITE;
//extern t_color PASTEL_GREEN;
//extern t_color PASTEL_BLUE;
//extern t_color WARM_GRAY;
//extern t_color PASTEL_YELLOW;
//extern t_color PASTEL_LIGHT_YELLOW;
//extern t_color DARK_GRAY;
//extern t_color LIGHT_GRAY_TRANS;
//extern t_color YELLOW_ORANGE;
//extern t_color STREET_RED;
//extern t_color STREET_DARK_ORANGE;
//extern t_color STREET_ORANGE;
//extern t_color STREET_REGULAR;

const t_color CUSTOM_BLACK(0,0,0,255);
const t_color CUSTOM_WHITE(255,255,255,255);
const t_color PASTEL_GREEN(179,255,171,255);
const t_color PASTEL_BLUE(195,228,253,255);
const t_color WARM_GRAY(211,211,211,255);
const t_color PASTEL_YELLOW(255,255,153,255);
const t_color PASTEL_LIGHT_YELLOW(255,255,160,255);
const t_color DARK_GRAY(140,140,140,255);
const t_color LIGHT_GRAY_TRANS(251,251,251,160);
const t_color YELLOW_ORANGE(245,200,66,255);
const t_color STREET_RED(237, 149, 149,255);
const t_color STREET_DARK_ORANGE(237, 173, 149,255);
const t_color STREET_ORANGE(237, 198, 149,255);
const t_color STREET_REGULAR(219,216,157,255);

enum PaletteCode {
  CARIBBEAN = 0,
  CUTE,
  DUSK,
};

struct Colors{

  //public colors
  t_color background;
  t_color street_1;
  t_color street_2;
  t_color street_3;
  t_color street_4;
  t_color street_outline;
  t_color street_highlight;
  t_color greenery;
  t_color water;
  t_color sand;
  t_color building;
  t_color building_shadow;
  t_color textbox;
  t_color textbox_highlight;
  t_color text;

  //private colors
  //not available to be altered by user

  Colors();
  Colors(t_color background, t_color street_1, t_color street_2, t_color street_3,
    t_color street_4, t_color street_outline, t_color street_highlight, t_color greenery, t_color water,
    t_color sand, t_color building, t_color building_shadow, t_color textbox,
    t_color textbox_highlight, t_color text);

};

class ColorPalette {
protected:
  string palette_name;
  Colors palette_colors;
public:
  ColorPalette();
  ColorPalette(PaletteCode palette_code);
  ColorPalette(Colors new_palette, string new_name);
  ColorPalette(t_color background, t_color street_1, t_color street_2, t_color street_3,
    t_color street_4, t_color street_outline, t_color street_highlight, t_color greenery, t_color water,
    t_color sand, t_color building, t_color building_shadow, t_color textbox,
    t_color textbox_highlight, t_color text, string new_name);

  const Colors& get_colors();
  const string& get_name();
  void set_name(string new_name);
};

class CustomPalette : public ColorPalette {
public:
  CustomPalette();

  void set_background(const t_color& new_color);
  void set_street_1(const t_color& new_color);
  void set_street_2(const t_color& new_color);
  void set_street_3(const t_color& new_color);
  void set_street_4(const t_color& new_color);
  void set_street_outline(const t_color& new_color);
  void set_street_highlight(const t_color& new_color);
  void set_greenery(const t_color& new_color);
  void set_water(const t_color& new_color);
  void set_sand(const t_color& new_color);
  void set_building(const t_color& new_color);
  void set_building_shadow(const t_color& new_color);
  void set_textbox(const t_color& new_color);
  void set_textbox_highlight(const t_color& new_color);
  void set_text(const t_color& new_color);
};

#endif /* COLOR_PALETTE_H */
