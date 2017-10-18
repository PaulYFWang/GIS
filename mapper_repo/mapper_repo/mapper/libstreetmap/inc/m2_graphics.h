#pragma once
#include "StreetsDatabaseAPI.h"
#include "graphics.h"

#define MIN_HIGHLIGHT_DISTANCE 0.0001
#define BACKGROUND_COLOR WHITE

class LatLon;

enum ElementType {
    
    FEATURE = 0,
    
    //feature subtypes
    PARK,
    GREENSPACE,
    GOLFCOURSE,
    ISLAND,
    LAKE,
    RIVER,
    STREAM,
    BEACH,
    SHORELINE,
    BUILDING,
    
    INTERSECTION,
    
    POINTOFINTEREST,
    
    //point of interest 
    FOOD,
    ENTERTAINMENT,
    RETAIL,
    PUBLICSERVICE,
    GOVERNMENT,
    RELIGIOUS,
    OTHER
};

int levDistance (string query, string compare);
vector< tuple<string,string, double, unsigned> > textSuggest(string query, string filter="SPIF", int numMax=5);
string toUpperCase (string in);
int consecutive (string query, string compare);
tuple <string,string,double,unsigned> autoComplete(string query, string filter="SPIF");


struct testStruct {
    vector<unsigned> road;
    pair<unsigned,unsigned> endpoints;
};

testStruct testm3();

void init_screen();

void draw_screen();

double cartesian_to_lon(double x, double avg_lat);

double cartesian_to_lon(double x);

double cartesian_to_lat(double y);

void draw_info_box(string name, string type, unsigned id, float Ix = get_visible_world().left(),float Iy = get_visible_world().bottom(), float Iwidth = get_visible_world().get_width()/4,float Iheight = get_visible_world().get_height()/3);
    
void draw_text_input_field();

void key_press(char keyVal, int keysym);

void mouse_click(float x,float y, t_event_buttonPressed event);

void act_on_mousemove(float x, float y);

void draw_in_radius(double x, double y, double radius);

double latitude_to_cartesian(double lat);

double longitude_to_cartesian(double lon, double avg_lat);

double longitude_to_cartesian(double lat);

void draw_intersections();

void draw_street_segment(unsigned street_seg_id);

void draw_street_segment(unsigned street_seg_id, const t_color& seg_color);

void draw_street_type(unsigned type, const t_color& color);

void draw_street(vector<unsigned> street_segs, int width, const t_color& color);

void draw_streets();

void draw_street(const vector<unsigned>& street_seg_ids);

void draw_features(FeatureType drawtype);

void draw_test_point(double lat, double lon, const Surface& icon, bool test);

void draw_point(double lat, double lon, double radius);

void draw_point(double lat, double lon, double radius, const t_color& color);

void draw_pointsOfInterest(ElementType type);

void draw_pointsOfInterest();

bool check_street_names(unsigned distance,unsigned x);

void draw_street_names();

float check_angle(unsigned x1,unsigned y1,unsigned x2,unsigned y2);

void highlight_street(unsigned street_id);

void set_street_size(unsigned x);

string featureDataTypeStr(FeatureType f);

bool check_point_on_screen(LatLon point);

bool check_point_on_screen(double x, double y);

bool check_on_screen();

bool check_on_screen(vector<LatLon> points);

bool check_on_screen(vector<t_point> points);

//vector<pair<SearchReturnType,unsigned> search_in_radius(vector<string> keywords);