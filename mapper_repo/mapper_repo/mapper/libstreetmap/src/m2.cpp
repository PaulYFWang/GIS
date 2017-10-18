#include <algorithm>
#include <cmath>
#include <map>
#include <unordered_map>
#include <vector>
#include <cmath>
#include "m1.h"
#include "../inc/m1_globals.h"
#include "m2.h"
#include "../inc/m2_graphics.h"
#include "m3.h"
#include "../inc/path_finding_struct.h"
#include "m4.h"
#include "graphics.h"
#include "graphics_types.h"
#include "OSMDatabaseAPI.h"
#include "StreetsDatabaseAPI.h"
#include "../inc/KDMap.h"
#include "../inc/ColorPalette.h"
#include "../inc/ScreenManager.h"
#include "../inc/TextInputBox.h"
#include "../inc/TextOutputBox.h"
#include "../inc/Button.h"
#include <iostream>
#include <string>
#include <tuple>
#include <cmath>
#include <sstream>
#include "easygl/graphics_state.h"
#include "easygl/fontcache.h"

using namespace std;

extern t_x11_state x11_state;
extern t_gl_state gl_state;

LatLon click;

string text_field="Search...";
tuple <string,string,double,unsigned>  text_field_suggest;
bool enable_text_input = false;
double Twidth, Theight,Tx, Ty, Ix,Iy,Iwidth,Iheight;
bool autoTaken = false;
bool suggestTaken = false;
bool drawInfo = false;
bool highlight = false;
unsigned highlightID = 0;
bool blinked = false;
double mouse_move_prev[2] = {0,0};
int themeNum = 0;


vector< tuple<string,string, double, unsigned> > textSuggestions;

ScreenManager* screen;

ColorPalette pastel;
ColorPalette caribbean(CARIBBEAN);
ColorPalette cute(CUTE);
ColorPalette dusk(DUSK);
CustomPalette custom;
vector<ColorPalette> themes;
Surface icon;
int clickInter1 = -1, clickInter2 = -1;
bool clickTogOn = false;
ColorPalette* current_palette;
vector<unsigned> shortestPath;
bool intersection_toggle = false;
bool poi_toggle = false;
bool drawShortPath = false;
Button swagButton;
Button findDirections;
//Button testButton;
TextInputBox input1;

//testStruct test;
vector<unsigned> test;

TextInputBox input2;
TextOutputBox BGWindowSearch;
Button colorSwitch;

Button helpTog;
TextOutputBox helpMe;

Button clickPathTog;

extern vector < vector <int_cost> > searchDataStruct;

// Draws the map. You can assume your load_map (string map_name)
// function is called before this function in the unit tests.
// Your main () program should do the same.
void draw_map(){
    
    current_palette = &pastel;
    themes.push_back(caribbean);
    themes.push_back(cute);
    themes.push_back(pastel);
    themes.push_back(dusk);
    
//    cout << numStreetSegs << endl;
//    cout << motorwayStreets.size() + truckStreets.size() + primaryStreets.size() +
//            secondaryStreets.size() + tertiaryStreets.size() + regularStreets.size() +
//            smallStreets.size() << endl;
//    for(int i = 26190; i < 26201; i++){
//        vector<unsigned> thing = find_adjacent_intersections(i);
//        cout << "for intersection " << i << " adjacent intersections are";
//        for(auto itr = thing.begin(); itr != thing.end(); itr++){
//            cout << " " << *itr;
//        }
//        cout << endl;
//    }
//    
//    for(int i = 26190; i < 26201; i++){
//        vector<int_cost> thing = searchDataStruct[i];
//        cout << "for intersection " << i << " processed adjacent intersections are";
//        for(auto itr = thing.begin(); itr != thing.end(); itr++){
//            cout << " " << itr->intersectionid;
//        }
//        cout << endl;
//    }
//    
//    for(int i = 26190; i < 26201; i++){
//        vector<unsigned> thing = searchByIntersection[i];
//        cout << "for intersection " << i << " all adjacent street segments are";
//        for(auto itr = thing.begin(); itr != thing.end(); itr++){
//            cout << " " << *itr;
//        }
//        cout << endl;
//    }
//    
//    for(int i = 26190; i < 26201; i++){
//        vector<int_cost> thing = searchDataStruct[i];
//        cout << "for intersection " << i << " processed adjacent street segments are";
//        for(auto itr = thing.begin(); itr != thing.end(); itr++){
//            cout << " " << itr->streetsegid;
//        }
//        cout << endl;
//    }
//    
//    for(int i = 26190; i < 26201; i++){
//        vector<int_cost> thing = searchDataStruct[i];
//        cout << "for intersection " << i << " processed adjacent street segments costs are";
//        for(auto itr = thing.begin(); itr != thing.end(); itr++){
//            cout << " " << itr->cost;
//        }
//        cout << endl;
//    }
    
    init_graphics("RAINDROPS (DROP TOP)", current_palette->get_colors().background);
    
    set_drawing_buffer(OFF_SCREEN);
   
    screen = ScreenManager::get_instance();
    
    //intersection_toggle = true;
    //poi_toggle = true;
    
    icon = load_png_from_file("libstreetmap/resources/location2.png");
    
    //destroy_button(button_state.button[0].text);
    //set the range for the map
    double maximum_latitude = latLonIntersection[0].lat();
    double minimum_latitude = latLonIntersection[0].lat();
    double maximum_longitude = latLonIntersection[0].lon();
    double minimum_longitude = latLonIntersection[0].lon();
    
    set_keypress_input(true);
    set_mouse_move_input(true);
    for(auto i = latLonIntersection.begin(); i < latLonIntersection.end() ; i++ ){
        
        maximum_latitude = max(maximum_latitude,i->lat());
        minimum_latitude = min(minimum_latitude,i->lat());
        maximum_longitude = max(maximum_longitude,i->lon());
        minimum_longitude = min(minimum_longitude,i->lon());
        
    }
    
    double average_latitude = (maximum_latitude + minimum_latitude)*DEG_TO_RAD/2;
    double average_longitude = (maximum_longitude + minimum_longitude)*DEG_TO_RAD/2;
    
    double max_x = longitude_to_cartesian(maximum_longitude,average_latitude);
    double min_x = longitude_to_cartesian(minimum_longitude,average_latitude);
    double max_y = latitude_to_cartesian(maximum_latitude);
    double min_y = latitude_to_cartesian(minimum_latitude);
    
    maximum_latitude *= DEG_TO_RAD;
    minimum_latitude *= DEG_TO_RAD;
    maximum_longitude *= DEG_TO_RAD;
    minimum_longitude *= DEG_TO_RAD;
    //TextInputBox t();
    
    double x_diff = max_x - min_x;
    double y_diff = max_y - min_y;
    
    double reduced_x = 3*x_diff/8;
    double reduced_y = 3*y_diff/8;
    
    
    t_bound_box init_box(min_x + reduced_x,
                        min_y + reduced_y,
                        max_x - reduced_x,
                        max_y - reduced_y);
    
    set_visible_world(init_box);
    

    screen->set_screen_state_prev(init_box);
    
    ScreenProperties global_properties(
    average_latitude, average_longitude, 
    maximum_latitude, minimum_latitude, maximum_longitude, minimum_longitude,
    max_x, min_x, max_y, min_y, x_diff, y_diff);
    
    screen->set_global_properties(global_properties);

    vector<DeliveryInfo> deliveries = {DeliveryInfo(79455, 73885), DeliveryInfo(14430, 10485), DeliveryInfo(79455, 99841), DeliveryInfo(108559, 26485), DeliveryInfo(38749, 65506), DeliveryInfo(20835, 87067), DeliveryInfo(84376, 107360), DeliveryInfo(42725, 29784), DeliveryInfo(32762, 75186), DeliveryInfo(34213, 39393), DeliveryInfo(69426, 107360), DeliveryInfo(55325, 28231), DeliveryInfo(33647, 31763), DeliveryInfo(68294, 15356), DeliveryInfo(70155, 24347), DeliveryInfo(28514, 55990), DeliveryInfo(46387, 88018), DeliveryInfo(37630, 15626), DeliveryInfo(86263, 58035), DeliveryInfo(82461, 20408), DeliveryInfo(91578, 24347), DeliveryInfo(107158, 42714), DeliveryInfo(35313, 7843), DeliveryInfo(17403, 47053), DeliveryInfo(77278, 63670), DeliveryInfo(37630, 93854), DeliveryInfo(33647, 107676), DeliveryInfo(37995, 30867), DeliveryInfo(71720, 73392), DeliveryInfo(4875, 89987), DeliveryInfo(57530, 37233), DeliveryInfo(71726, 50174), DeliveryInfo(47045, 15356), DeliveryInfo(97081, 15356), DeliveryInfo(34213, 67172), DeliveryInfo(80829, 63670), DeliveryInfo(103400, 101884), DeliveryInfo(107667, 64172), DeliveryInfo(57530, 83862), DeliveryInfo(26535, 101729), DeliveryInfo(578, 4701), DeliveryInfo(82, 2027), DeliveryInfo(20059, 51404), DeliveryInfo(43367, 54326), DeliveryInfo(37995, 97563), DeliveryInfo(6469, 102816), DeliveryInfo(106029, 42732), DeliveryInfo(36836, 106282), DeliveryInfo(71616, 12665), DeliveryInfo(17189, 31763), DeliveryInfo(51160, 63670), DeliveryInfo(2030, 1781), DeliveryInfo(52135, 58035), DeliveryInfo(60829, 38153), DeliveryInfo(70329, 27322), DeliveryInfo(56687, 50174), DeliveryInfo(102644, 93854), DeliveryInfo(51160, 21629), DeliveryInfo(89837, 21123), DeliveryInfo(62814, 44061), DeliveryInfo(5485, 4477), DeliveryInfo(33647, 65506), DeliveryInfo(51160, 75466), DeliveryInfo(67195, 14706), DeliveryInfo(51160, 7843), DeliveryInfo(37995, 12665), DeliveryInfo(107053, 86249), DeliveryInfo(79455, 70851), DeliveryInfo(20490, 59387), DeliveryInfo(51160, 77914), DeliveryInfo(79455, 80541), DeliveryInfo(103954, 30634), DeliveryInfo(100065, 20408), DeliveryInfo(83317, 38615), DeliveryInfo(45974, 20321), DeliveryInfo(35313, 1899), DeliveryInfo(62038, 67929), DeliveryInfo(70710, 57250), DeliveryInfo(5225, 53901), DeliveryInfo(82993, 33449), DeliveryInfo(51160, 49192), DeliveryInfo(69411, 43920), DeliveryInfo(57530, 48581), DeliveryInfo(34213, 12965), DeliveryInfo(99394, 17430), DeliveryInfo(12367, 10485), DeliveryInfo(82993, 80188), DeliveryInfo(56284, 56652), DeliveryInfo(37630, 93854), DeliveryInfo(34213, 9757), DeliveryInfo(8831, 81670), DeliveryInfo(67383, 70107), DeliveryInfo(66198, 105775), DeliveryInfo(72195, 20160), DeliveryInfo(51160, 4165), DeliveryInfo(52135, 84657), DeliveryInfo(51724, 59909), DeliveryInfo(34213, 33857), DeliveryInfo(22254, 96331), DeliveryInfo(2996, 24081), DeliveryInfo(19118, 53115), DeliveryInfo(79455, 30052), DeliveryInfo(40790, 107605), DeliveryInfo(79455, 63670), DeliveryInfo(62255, 36331), DeliveryInfo(99394, 63670), DeliveryInfo(61183, 67293), DeliveryInfo(107158, 65506), DeliveryInfo(92100, 30634), DeliveryInfo(61789, 5972), DeliveryInfo(87640, 7843), DeliveryInfo(69768, 80701), DeliveryInfo(105434, 67172), DeliveryInfo(44077, 12413), DeliveryInfo(37630, 25767), DeliveryInfo(51160, 50383), DeliveryInfo(32528, 103466), DeliveryInfo(37995, 4701), DeliveryInfo(37995, 73392), DeliveryInfo(105174, 12665), DeliveryInfo(66884, 68455), DeliveryInfo(97034, 38752), DeliveryInfo(107158, 84254), DeliveryInfo(38421, 54525), DeliveryInfo(26272, 38594), DeliveryInfo(46996, 12665), DeliveryInfo(35313, 76583), DeliveryInfo(39295, 7843), DeliveryInfo(24117, 10485), DeliveryInfo(66930, 57489), DeliveryInfo(27823, 4165), DeliveryInfo(55810, 107360), DeliveryInfo(35313, 80701), DeliveryInfo(37995, 73198), DeliveryInfo(29102, 43920), DeliveryInfo(6709, 52971), DeliveryInfo(39078, 69259), DeliveryInfo(99149, 7843), DeliveryInfo(102632, 15005), DeliveryInfo(105744, 30061), DeliveryInfo(87027, 50174), DeliveryInfo(40801, 69027), DeliveryInfo(6709, 80541), DeliveryInfo(107158, 97563), DeliveryInfo(26214, 31702), DeliveryInfo(77961, 20408), DeliveryInfo(35313, 74120), DeliveryInfo(1181, 59203), DeliveryInfo(83990, 33468), DeliveryInfo(26750, 15356), DeliveryInfo(37310, 20307), DeliveryInfo(79455, 73392), DeliveryInfo(102613, 3112), DeliveryInfo(60258, 67985), DeliveryInfo(10679, 27776), DeliveryInfo(40080, 58035), DeliveryInfo(99394, 26570), DeliveryInfo(57530, 31568), DeliveryInfo(3153, 22846), DeliveryInfo(42725, 93247), DeliveryInfo(49298, 92319), DeliveryInfo(6345, 13222), DeliveryInfo(59248, 38615), DeliveryInfo(91580, 21629), DeliveryInfo(2032, 98262), DeliveryInfo(80474, 38685), DeliveryInfo(33647, 68676), DeliveryInfo(35313, 60835), DeliveryInfo(83317, 28756), DeliveryInfo(99394, 59203), DeliveryInfo(86012, 93854), DeliveryInfo(47947, 21629), DeliveryInfo(14774, 65506), DeliveryInfo(79455, 58035), DeliveryInfo(23741, 46990), DeliveryInfo(102613, 10998), DeliveryInfo(108063, 15726), DeliveryInfo(41604, 6224), DeliveryInfo(104009, 73603), DeliveryInfo(75284, 67344), DeliveryInfo(6709, 104642), DeliveryInfo(101640, 7182), DeliveryInfo(36071, 54387), DeliveryInfo(98893, 33156), DeliveryInfo(30885, 107331), DeliveryInfo(82993, 66932), DeliveryInfo(33647, 31763), DeliveryInfo(24490, 92319), DeliveryInfo(57530, 57345), DeliveryInfo(12689, 59352), DeliveryInfo(6709, 65506), DeliveryInfo(103954, 50150), DeliveryInfo(1959, 16171), DeliveryInfo(20490, 98262), DeliveryInfo(8166, 56081), DeliveryInfo(34213, 71797), DeliveryInfo(31305, 67172), DeliveryInfo(24244, 21629), DeliveryInfo(75089, 97563), DeliveryInfo(7169, 97563), DeliveryInfo(30301, 67539), DeliveryInfo(90791, 81955), DeliveryInfo(42959, 59203), DeliveryInfo(57530, 61808), DeliveryInfo(410, 92494), DeliveryInfo(92349, 40334), DeliveryInfo(91580, 18045), DeliveryInfo(64993, 82804), DeliveryInfo(65560, 15298), DeliveryInfo(107158, 48468), DeliveryInfo(79455, 20408), DeliveryInfo(42725, 8731), DeliveryInfo(42725, 81558), DeliveryInfo(27119, 99648), DeliveryInfo(37630, 59203), DeliveryInfo(51160, 65981), DeliveryInfo(102613, 32994), DeliveryInfo(34797, 12045), DeliveryInfo(2237, 80710), DeliveryInfo(91580, 5061), DeliveryInfo(34213, 91631), DeliveryInfo(105744, 80541), DeliveryInfo(6226, 38594), DeliveryInfo(7394, 73392), DeliveryInfo(102613, 38594), DeliveryInfo(82871, 15356), DeliveryInfo(102887, 20408), DeliveryInfo(105744, 24347), DeliveryInfo(63589, 7550), DeliveryInfo(102613, 59324), DeliveryInfo(99394, 8553), DeliveryInfo(103589, 21617), DeliveryInfo(51160, 59203), DeliveryInfo(51813, 60651), DeliveryInfo(90255, 351), DeliveryInfo(37630, 37233), DeliveryInfo(102613, 74991), DeliveryInfo(71605, 38594), DeliveryInfo(37995, 10985), DeliveryInfo(99394, 7971), DeliveryInfo(12087, 73392), DeliveryInfo(5446, 11574), DeliveryInfo(52117, 107386), DeliveryInfo(35313, 21303), DeliveryInfo(71021, 79876), DeliveryInfo(57530, 67172), DeliveryInfo(79455, 12665), DeliveryInfo(50874, 70919), DeliveryInfo(102613, 31763), DeliveryInfo(34014, 66982), DeliveryInfo(6372, 98642), DeliveryInfo(1958, 10485), DeliveryInfo(52135, 29121), DeliveryInfo(52135, 31763), DeliveryInfo(54404, 12767), DeliveryInfo(34213, 6543), DeliveryInfo(3238, 13116), DeliveryInfo(99394, 89087), DeliveryInfo(102613, 43626), DeliveryInfo(97803, 59091)};
    vector<unsigned> depots = {80658, 107155, 34524, 106290, 80008};
    double turn_penalty = 15;
    
    //test = testm3();
    test = traveling_courier(deliveries,depots,turn_penalty);
    
//    Window w = XCreateSimpleWindow(x11_state.display,x11_state.toplevel,100,100,200,200,3,1,0);
//    XMapWindow(x11_state.display,w);
    draw_text_input_field();
    
    
    /*VERY IMPORTANT COMMENT FOR FUTURE CREATION OF WINDOW CLASSES
     * THIS IS FOR WHEN U WANT TO MAKE AN OBJECT POSITION/SIZE INSIDE A WINDOW CALSS RELATIVE TO THE WINDOW INSTEAD OF THE MAP
     * Let mxw be posXMult of window, mxb by posXMult of object (relative to window), MWX be the widthDiv of the window, Sx is world width, MBX is relative object width to window
     * 
     * set mxb = 0.5-0.5(1/MBX) to center on x axis
     * 
     * posXMult(object) = mxw + (mxb/MWX);
     * widthDiv(object) = MBX*MWX;
     * 
     * for Y dim
     * 
     * Let myw be posYMult of window, myb by posYMult of object (relative to window), MWY be the heightDiv of the window, Sy is world height, MBY is relative object height to window
     *                                                                                                               
     * set myb = 0.5-0.5(1/MBY) for y axis center
     * 
     * posYMult(object) = myw + (myb/MWY)
     * heightDiv(object) = MBY*MWY
     */
    
    
    BGWindowSearch.initConditions(0.025,0.05,4.5,2.5,"Directions\n\n\nFrom                                 \n\n\n\n\n\nTo                                    ",15,false,true);
    swagButton.initConditions(0.92,0.3,15,20,10,true,true,"Directions" , 3,PINK,PASTEL_BLUE);
    findDirections.initConditions(0.025+(0.5-0.5*(1/5.0))/4.5, 0.05 + (0.8)/2.5,4.5*5,2.5*15,8,false,false,"GO!" , 3,WARM_GRAY,PASTEL_BLUE);
    //testButton.initConditions(0.025+(0.5-0.5*(1/2.0))/4.5, 0.05 + (0.5-0.5*(1/2.0))/2.5,4.5*2,2.5*2,8,true,true,"TEST ME" , 0,GREEN,GREEN);
    input1.initConditions(0.025+(0.5-0.5*(1/1.33))/4.5,0.05 + (0.2)/2.5,4.5*1.33,30,10,false,false,true,true,WARM_GRAY,PASTEL_BLUE, PINK,RED);
    input2.initConditions(0.025+(0.5-0.5*(1/1.33))/4.5,0.05 + (0.5)/2.5,4.5*1.33,30,10,false,false,true,true,WARM_GRAY,PASTEL_BLUE,PASTEL_GREEN,GREEN);
    swagButton.updateWorldConditions();
    
    helpTog.initConditions(0.92,0.5,15,20,10,true,true,"Help Me!" , 3,PINK,PASTEL_BLUE);
    clickPathTog.initConditions(0.92,0.6,15,20,7,true,true,"Click On Directions" , 3,PINK,PASTEL_BLUE);
    helpMe.initConditions(0.25,0.25,2,2,"HELP\n\nButtons Toggle things on or off\n\nChange theme changes theme\n\nDirections accepts either an intersection of POI and draws shortest path\n\nSearch on top right searches for features and blinks the user to them. The streets are also highlighted\n\nClicking on the map highlights nearest POI and intersection\n\nMiddle mouse button can be used to drag pan and zoom\n\nX closes all interfaces including this one\n\nTo input into text fields, please click on them and make sure they are highlighted\n\nTab can be use in the text fields to autocomplete the word using the gray text in the background\n\nclicking on results in the drop down menus will fill the input with the suggestions\n\nenter is used in the top right to engage a search, while directions uses the go button\n\nHave a nice day using EasyGIS :)", 12, false,true);
    colorSwitch.initConditions(0.92,0.4,15,20,8,true,true,"Change Theme" , 3,PINK,PASTEL_BLUE);
    
    /*setTheme Colors*/
    input1.highlightColor=current_palette->get_colors().textbox_highlight;
    input1.edgeColor1=current_palette->get_colors().building;
    input1.edgeColor2=current_palette->get_colors().building_shadow;
    input1.fieldColor=WHITE;

    input2.highlightColor=current_palette->get_colors().textbox_highlight;
    input2.edgeColor1=current_palette->get_colors().building;
    input2.edgeColor2=current_palette->get_colors().building_shadow;
    input2.fieldColor=WHITE;


    BGWindowSearch.edgeColor1=current_palette->get_colors().building;
    BGWindowSearch.edgeColor2=current_palette->get_colors().building_shadow;
    BGWindowSearch.fieldColor=current_palette->get_colors().textbox;
    BGWindowSearch.textColor=current_palette->get_colors().text;
    BGWindowSearch.xBoxColor=current_palette->get_colors().building;
    BGWindowSearch.xColor=current_palette->get_colors().building_shadow;

    helpMe.edgeColor1=current_palette->get_colors().building;
    helpMe.edgeColor2=current_palette->get_colors().building_shadow;
    helpMe.fieldColor=current_palette->get_colors().textbox;
    helpMe.textColor=current_palette->get_colors().text;
    helpMe.xBoxColor=current_palette->get_colors().building;
    helpMe.xColor=current_palette->get_colors().building_shadow;

    swagButton.inner=current_palette->get_colors().street_4;
    swagButton.outer=current_palette->get_colors().street_2;

    colorSwitch.inner=current_palette->get_colors().street_4;
    colorSwitch.outer=current_palette->get_colors().street_2;

    findDirections.inner=current_palette->get_colors().street_4;
    findDirections.outer=current_palette->get_colors().street_2;

    helpTog.inner=current_palette->get_colors().street_4;
    helpTog.outer=current_palette->get_colors().street_2;

    clickPathTog.inner=current_palette->get_colors().street_4;
    clickPathTog.outer=current_palette->get_colors().street_2;

    /**/
    event_loop(mouse_click,act_on_mousemove,key_press,draw_screen);
    
    close_graphics();
    
    ScreenManager::destroy_instance();
}

testStruct testm3(){
    
    //find_path_between_intersections(100269, 395237, 15);
    
    double penalty = 15;
    unsigned id1 = 12310;
    unsigned id2 = 26200;
    
    vector <unsigned> result;
    double t1;
    double t2;
    t1 = clock();
    result = find_path_between_intersections(id1,id2,penalty);
    t2 = clock();
    cout << "A* base runtime in ms: " << (t2 - t1)/(double)CLOCKS_PER_SEC*1000 << endl;
    result.clear();
    t1 = clock();
    result = find_path_between_intersections3(id1,id2,penalty);
    t2 = clock();
    cout << "A* modified runtime in ms: " << (t2 - t1)/(double)CLOCKS_PER_SEC*1000 << endl;
    double time = compute_path_travel_time(result,penalty);
    cout<<"cost: "<<time<<endl;
    testStruct finalResult;
    finalResult.road = result;
    finalResult.endpoints = make_pair(id1,id2);
    return finalResult;
}

void draw_screen(){
    
    Ix = get_visible_world().left();
    Iy = get_visible_world().bottom() + get_visible_world().get_height()/3;
    Iwidth = get_visible_world().get_width()/5;
    Iheight = get_visible_world().get_height()/3;     
    //cout << "toupper of [: " << toupper('[') <<endl;
    screen->check_oob();
    
    //draw all map elements
    clearscreen();

    draw_features(Park);
    draw_features(Greenspace);
    draw_features(Golfcourse);
    draw_features(Lake);
    draw_features(Island);
    draw_features(River);
    draw_features(Stream);
    draw_features(Beach);
    draw_features(Shoreline);
    
    draw_streets();

    draw_features(Building);
    
    //draw_street_names();
    
    if(intersection_toggle){
        draw_intersections();
    }
    
    if(poi_toggle){
        draw_pointsOfInterest();
    }
    
    Ix = get_visible_world().left();
    Iy = get_visible_world().bottom() + get_visible_world().get_height()/3;
    Iwidth = get_visible_world().get_width()/5;
    Iheight = get_visible_world().get_height()/3; 
    
   
    if (highlight){
        setcolor(BLUE);
        highlight_street(highlightID);
    }
    if (drawShortPath){
        cout <<"Shortest path active" <<endl;
        draw_street(shortestPath,3,BLUE);
    }
    draw_text_input_field();
    if (drawInfo && (autoTaken || suggestTaken))
        draw_info_box(get<0>(text_field_suggest),get<1>(text_field_suggest),get<3>(text_field_suggest), Ix,Iy,Iwidth,Iheight);
    
//    cout << test.endpoints.first << ", " << test.endpoints.second << endl;
//    setcolor(DARKGREEN);
//    draw_point(latLonIntersection[test.endpoints.first].lat(),latLonIntersection[test.endpoints.first].lon(),50);
//    draw_point(latLonIntersection[test.endpoints.second].lat(),latLonIntersection[test.endpoints.second].lon(),50);
//    cout << test.road.size() << endl;
//    draw_street(test.road);
//    setcolor(RED);
//    draw_point(latLonIntersection[80658].lat(),latLonIntersection[80658].lon(),300);
//    draw_point(latLonIntersection[107155].lat(),latLonIntersection[107155].lon(),300);
//    draw_point(latLonIntersection[34524].lat(),latLonIntersection[34524].lon(),300);
//    draw_point(latLonIntersection[106290].lat(),latLonIntersection[106290].lon(),300);
//    draw_point(latLonIntersection[80008].lat(),latLonIntersection[80008].lon(),300);
    draw_street(test);
//    setcolor(SADDLEBROWN);
//    setlinewidth(10);
//    draw_street_segment(test.front());
//    draw_street_segment(test.back());
    
    BGWindowSearch.updateWorldConditions();
    BGWindowSearch.drawOutputBox();
    input1.updateWorldConditions();
    input1.drawBox();
    input2.updateWorldConditions();
    input2.drawBox();

//    testButton.updateWorldConditions();
//    testButton.drawButton();
    
    findDirections.updateWorldConditions();
    findDirections.drawButton();
    swagButton.updateWorldConditions();
    swagButton.drawButton();
    
    helpTog.updateWorldConditions();
    helpTog.drawButton();
    
    clickPathTog.updateWorldConditions();
    clickPathTog.drawButton();
    
    colorSwitch.updateWorldConditions();
    colorSwitch.drawButton();
    if (clickTogOn){
        if (clickInter1 != -1 || clickInter2 != -1 ){
            settextattrs(13,0);
            if (clickInter1 != -1){
                setcolor(current_palette->get_colors().building);
                fillarc(longitude_to_cartesian(getIntersectionPosition(clickInter1).lon()),latitude_to_cartesian(getIntersectionPosition(clickInter1).lat()), Iwidth/30,0,360);
                setcolor(BLACK);
                drawtext(longitude_to_cartesian(getIntersectionPosition(clickInter1).lon())+Iwidth/30,latitude_to_cartesian(getIntersectionPosition(clickInter1).lat())-Iwidth/30, "Start", FLT_MAX, FLT_MAX,false);
            }
            if (clickInter2 != -1){
            setcolor(current_palette->get_colors().building_shadow);
            fillarc(longitude_to_cartesian(getIntersectionPosition(clickInter2).lon()),latitude_to_cartesian(getIntersectionPosition(clickInter2).lat()), Iwidth/30,0,360);
            
            setcolor(BLACK);
            drawtext(longitude_to_cartesian(getIntersectionPosition(clickInter2).lon())+Iwidth/30,latitude_to_cartesian(getIntersectionPosition(clickInter2).lat())-Iwidth/30, "Destination", FLT_MAX, FLT_MAX,false);
            }
            
        }
    }
    cout << " Start ID: " << clickInter1 << " End ID: " << clickInter2 << endl;
    helpMe.updateWorldConditions();
    helpMe.drawOutputBox() ;
    copy_off_screen_buffer_to_screen();
    
}


void key_press(char keyVal, int keysym){
    flushinput();
    if (highlight){
    highlight = false;
    draw_screen();
    }
    if (enable_text_input){

        if (keyVal >=32 && keyVal <= 126){
            autoTaken = false;
            suggestTaken = false;
            if (text_field.empty())
                text_field += toupper(keyVal);
            else 
                text_field += keyVal;
            
        if (!text_field.empty()){
            text_field_suggest = autoComplete(text_field);
            textSuggestions = textSuggest(text_field);
        }
        else
            get<0>(text_field_suggest) = "";
        }
        else{
            #ifdef X11
            switch(keysym){
                case XK_BackSpace:
                    autoTaken = false;
                    suggestTaken = false;
                    drawInfo = false;
                    if (text_field.empty()||text_field.length()==0 ){
                       
                        break;
                    }
                    else if (text_field.length() <= 1||text_field == "Search..."){
                        text_field = "";
                        draw_screen();
                    }
                    else {
                        text_field.erase(text_field.end()-1);
                        text_field_suggest = autoComplete(text_field);
                        textSuggestions = textSuggest(text_field);
                    }
                    
                    break;
                case XK_Tab:
                    
                    if (!(text_field.empty()||text_field.length()==0 )){
                        if (!get<0>(text_field_suggest).empty()){
                            text_field = get<0>(text_field_suggest);
                            autoTaken = true;
                            drawInfo = false;
                        }
                    }
                       //flushinput();
                    break;
                case XK_Return:
                    if(suggestTaken|| autoTaken){
                        drawInfo = true;
                        blinked = false;
                        draw_screen();
                    }
                    break;
                default:
                    break;
            }
            #endif
    }
    
}
    //Input1 Text input keyboard interactions
    if (keyVal >=32 && keyVal <= 126){
        if (keyVal == 115 && !enable_text_input && !input1.active && !input2.active){
            swagButton.setVisible(!swagButton.visible);
            draw_screen();
        swagButton.drawButton();
        }
    }
   // cout <<"before input1 weird character: " << keyVal << endl;
    if(input1.active){
        if (keyVal >=32 && keyVal <= 126){
             drawShortPath = false;
            if (input1.curText.empty()){
               // cout <<"before toupper: " << keyVal << endl;
                input1.inputTaken(toupper(keyVal));
                //cout << "Not toupper" <<endl;
            }
            else 
                input1.inputTaken(keyVal);
            if (!input1.curText.empty()){
                if(input1.autoActive){
                  //  cout << "autocomplete issue" << endl;
                    input1.loadAutocomplete(autoComplete(input1.curText,"I"));
                   // cout << " or not" << endl;
                }
                if(input1.suggestActive){
                    //cout << "suggestions issue" << endl;
                    input1.loadSuggestions(textSuggest(input1.curText,"I",2));   
                    //cout << " or not" << endl;
                }
            }
        }
        
        else{
            #ifdef X11
            switch(keysym){
                case XK_BackSpace:
                     drawShortPath = false;
                    if (input1.curText.empty()){
                        break;
                    }
                    else if (input1.curText.length() <= 1){
                        input1.curText = "";
                    }
                    else {
                        input1.curText.erase(input1.curText.end()-1);
                        if (input1.autoActive)
                            input1.loadAutocomplete(autoComplete(input1.curText,"I"));
                        if (input1.suggestActive)
                            input1.loadSuggestions(textSuggest(input1.curText,"I",2));
                    }
                    break;
                case XK_Tab:
                     drawShortPath = false;
                    if (!input1.curText.empty()&&input1.autoActive){
                        if (!get<0>(input1.autocomplete).empty()){
                            input1.curText = get<0>(input1.autocomplete);
                        }
                    }
                      // flushinput();
                    break;
                case XK_Return:
                    
                    break;
                default:
                    break;
            }
            #endif
        }
    }
    
    if(input2.active){
        if (keyVal >=32 && keyVal <= 126){
            drawShortPath = false;
            if (input2.curText.empty())
                input2.inputTaken(toupper(keyVal));
            else 
                input2.inputTaken(keyVal);
            if (!input2.curText.empty()){
                if(input2.autoActive)
                    input2.loadAutocomplete(autoComplete(input2.curText,"IP"));
                if(input2.suggestActive)
                    input2.loadSuggestions(textSuggest(input2.curText,"IP",2));   
            }
        }
        
        else{
            
            #ifdef X11
            switch(keysym){
                
                case XK_BackSpace:
                     drawShortPath = false;
                    
                    if (input2.curText.empty()){
                        break;
                    }
                    else if (input2.curText.length() <= 1){
                        input2.curText = "";
                    }
                    else {
                        input2.curText.erase(input2.curText.end()-1);
                        if (input2.autoActive)
                            input2.loadAutocomplete(autoComplete(input2.curText,"IP"));
                        if (input2.suggestActive)
                            input2.loadSuggestions(textSuggest(input2.curText,"IP",2));
                    }
                    break;
                case XK_Tab:
                     drawShortPath = false;
                    if (!input2.curText.empty()&&input2.autoActive){
                        if (!get<0>(input2.autocomplete).empty()){
                            input2.curText = get<0>(input2.autocomplete);
                        }
                    }
                       //flushinput();
                    break;
                case XK_Return:
                    
                    break;
                default:
                    break;
            }
            #endif
        }
    }
    //cout <<"Takes in weird character" << endl;
    input1.updateWorldConditions();
    input1.drawBox();
    input2.updateWorldConditions();
    input2.drawBox();
    draw_text_input_field();
    copy_off_screen_buffer_to_screen();
}

void mouse_click(float x,float y, t_event_buttonPressed event){
    double longitude = cartesian_to_lon(x);
    double latitude = cartesian_to_lat(y);
    bool notInTextOrInput = true;
   
    
    unsigned from,to;
    
    string toPOI;
    bool toPOIPath;
    bool fromValid = false;
    bool toValid = false;
    
    
    //draw_screen();
    flushinput();
    //see if text box is activated
    if((x >= Tx && x < Tx + Twidth &&y <= Ty && y > Ty-Theight*(1) )||(x >= Tx && x < Tx + Twidth &&y <= Ty && y > Ty-Theight*(textSuggestions.size()+1)&&enable_text_input) ){
        enable_text_input = true;
        //notInTextOrInput = false;
      //  cout << "In text box" << endl;
        for (unsigned i = 0 ; i < textSuggestions.size(); i ++){
            if (y < Ty-Theight*(i+1)&&y > Ty-Theight*(i+2)){
                fillrect(Tx ,Ty-Theight*(i+1), Tx + Twidth,Ty-Theight*(i+2));
              //  cout << get<0>(textSuggestions[i]) << endl;
                text_field = get<0>(textSuggestions[i]);
                text_field_suggest=textSuggestions[i];
                suggestTaken = true;
                drawInfo = false;
            }
        }
    }
    else{
        if(enable_text_input){
            enable_text_input = false;
            draw_screen();
        }
        enable_text_input = false;
    }
    
    
    
    if (x > Ix + 38*Iwidth/40 && x < Ix+Iwidth && y < Iy && y > Iy - 2*Iwidth/40){
        //cout << "clicked on x" << endl;
        drawInfo = false;
        highlight = false;
        draw_screen();
    }
    
    if (x > Ix && x < Ix+Iwidth && y < Iy && y > Iy - Iwidth){
        //notInTextOrInput = false;
    }
    
    if (helpTog.clicked(x,y)){
        helpMe.setVisible(!helpMe.visible);
    }
    if (helpMe.clickedClosed(x,y)){
        helpMe.setVisible(false);
    }
    if (colorSwitch.clicked(x,y)){
        themeNum++;
        current_palette = &(themes[themeNum%themes.size()]);
        input1.highlightColor=current_palette->get_colors().textbox_highlight;
        input1.edgeColor1=current_palette->get_colors().building;
        input1.edgeColor2=current_palette->get_colors().building_shadow;
        input1.fieldColor=WHITE;
       
        input2.highlightColor=current_palette->get_colors().textbox_highlight;
        input2.edgeColor1=current_palette->get_colors().building;
        input2.edgeColor2=current_palette->get_colors().building_shadow;
        input2.fieldColor=WHITE;
        
       
        BGWindowSearch.edgeColor1=current_palette->get_colors().building;
        BGWindowSearch.edgeColor2=current_palette->get_colors().building_shadow;
        BGWindowSearch.fieldColor=current_palette->get_colors().textbox;
        BGWindowSearch.textColor=current_palette->get_colors().text;
         BGWindowSearch.xBoxColor=current_palette->get_colors().building;
         BGWindowSearch.xColor=current_palette->get_colors().building_shadow;
        
        helpMe.edgeColor1=current_palette->get_colors().building;
        helpMe.edgeColor2=current_palette->get_colors().building_shadow;
        helpMe.fieldColor=current_palette->get_colors().textbox;
        helpMe.textColor=current_palette->get_colors().text;
        helpMe.xBoxColor=current_palette->get_colors().building;
         helpMe.xColor=current_palette->get_colors().building_shadow;
        
        swagButton.inner=current_palette->get_colors().street_4;
        swagButton.outer=current_palette->get_colors().street_2;
        
        colorSwitch.inner=current_palette->get_colors().street_4;
        colorSwitch.outer=current_palette->get_colors().street_2;
        
        findDirections.inner=current_palette->get_colors().street_4;
        findDirections.outer=current_palette->get_colors().street_2;
        
        helpTog.inner=current_palette->get_colors().street_4;
        helpTog.outer=current_palette->get_colors().street_2;
         clickPathTog.inner=current_palette->get_colors().street_4;
        clickPathTog.outer=current_palette->get_colors().street_2;
        
    }
    if (clickPathTog.clicked(x,y)){
        if (clickTogOn == false){
            clickInter1 = -1;
            clickInter2 = -1;
        }
        clickTogOn = !clickTogOn;
        if (clickTogOn){
            input1.setVisible(!clickTogOn);
            input2.setVisible(!clickTogOn);
            findDirections.setVisible(!clickTogOn);
            //findDirections.setActive(!findDirections.active);
            BGWindowSearch.setVisible(!clickTogOn);
        }
        if (BGWindowSearch.visible || !clickTogOn)
             drawShortPath = false;
        clickPathTog.isClicked = true;
        clickPathTog.updateWorldConditions();
        clickPathTog.drawButton();
        
//        input1.updateWorldConditions();
//        input1.drawBox();
        draw_screen();
        copy_off_screen_buffer_to_screen();
       // cout <<"beofre sleep" << endl;
        
        usleep(40000);
        //cout << "AfterSleep " << endl;
        clickPathTog.isClicked = false;
        clickPathTog.updateWorldConditions();
        clickPathTog.drawButton();  
        
        
    }
    
    if (clickTogOn && !clickPathTog.clicked(x,y) && !colorSwitch.clicked(x,y) && !swagButton.clicked(x,y) && !input1.active &&!helpMe.visible && !helpTog.clicked(x,y) && !((x >= Tx && x < Tx + Twidth &&y <= Ty && y > Ty-Theight*(1) )||(x >= Tx && x < Tx + Twidth &&y <= Ty && y > Ty-Theight*(textSuggestions.size()+1)&&enable_text_input)) ){
            LatLon click_pos(latitude,longitude);
          if (clickInter1 == -1){
              clickInter1 = find_closest_intersection(click_pos);
          }
          else if (clickInter2 == -1){
              clickInter2= find_closest_intersection(click_pos);
          }
          else{
              if(find_closest_intersection(click_pos) !=clickInter2){
          
              clickInter1= clickInter2;
              clickInter2=find_closest_intersection(click_pos);
              }
          }
            if(clickInter2 != -1 && clickInter1 != -1){
            drawShortPath = true;
            shortestPath = find_path_between_intersections(clickInter1,clickInter2,15);
            }
        }
        

    if (swagButton.clicked(x,y)||BGWindowSearch.clickedClosed(x,y)){
        
        input1.setVisible(!input1.visible);
        input2.setVisible(!input2.visible);
        findDirections.setVisible(!findDirections.visible);
        //findDirections.setActive(!findDirections.active);
        BGWindowSearch.setVisible(!BGWindowSearch.visible);
        if (input1.visible)
            clickTogOn = false;
        if (!clickTogOn ){
            drawShortPath = false;
            draw_screen();
        }
        if (!BGWindowSearch.visible)
             drawShortPath = false;
        //cout <<"Ouch you clicked the swagbutton" << endl;
        swagButton.isClicked = true;
        swagButton.updateWorldConditions();
        swagButton.drawButton();
        
//        input1.updateWorldConditions();
//        input1.drawBox();
        draw_screen();
        copy_off_screen_buffer_to_screen();
        //cout <<"beofre sleep" << endl;
        
        usleep(40000);
        cout << "AfterSleep " << endl;
        swagButton.isClicked = false;
        swagButton.updateWorldConditions();
        swagButton.drawButton();  
    }
   
    if (findDirections.clicked(x,y)){
        findDirections.isClicked = true;
        findDirections.updateWorldConditions();
        findDirections.drawButton();
        copy_off_screen_buffer_to_screen();
        
        usleep(40000);
         findDirections.isClicked = false;
        findDirections.updateWorldConditions();
        findDirections.drawButton();  
        
        if (input1.autoActive&& input1.suggestActive){
            if ((get<0>(input1.autocomplete)).size()!=0 && get<0>(input1.autocomplete)==input1.curText){
                //cout << "FROM AUTOCOMP: " << input1.curText
                
                from = get<3>(input1.autocomplete);
    
                fromValid = true;
            }
                
            else if(!input1.textSuggestions.empty()&&!input1.curText.empty()){
                //cout <<"FROM SUGGESTIONS: " << get<0>(input1.textSuggestions[0]) << endl;
                
                from = get<3>(input1.textSuggestions[0]);
                fromValid = true;
            }
            else{
                //cout <<"EMPTY FROM?!" << endl;
                fromValid = false;
            }
                        
        }
         if (input2.autoActive&& input2.suggestActive){
            if ((get<0>(input2.autocomplete)).size()!=0 && get<0>(input2.autocomplete)==input2.curText){
                //cout << "FROM AUTOCOMP: " << input1.curText;
                if (get<1>(input2.autocomplete) == "P"){
                    toPOIPath = true;
                    toPOI = get<0>(input2.autocomplete);
                }
                else{
                    toPOIPath = false;
                    to = get<3>(input2.autocomplete);
                }
                toValid = true;
            }
                
            else if(!input2.textSuggestions.empty()&&!input2.curText.empty()){
                //cout <<"FROM SUGGESTIONS: " << get<0>(input1.textSuggestions[0]) << endl;
                if (get<1>(input2.textSuggestions[0]) == "P"){
                    toPOIPath = true;
                    toPOI = get<0>(input2.textSuggestions[0]);
                }
                else{
                    toPOIPath = false;
                    to = get<3>(input2.textSuggestions[0]);
                }
                toValid = true;
            }
            else{
                //cout <<"EMPTY FROM?!" << endl;
                toValid = false;
            }              
        }
        if (toValid && fromValid){
            if(toPOIPath){
                //toPOI ->string
                //from ->int ID
                //Uncomment this
            drawShortPath = true;
            shortestPath = find_path_to_point_of_interest(from,toPOI,15);
            
//            settextattrs(13,0);
//           
//                setcolor(current_palette->get_colors().building);
//                fillarc(longitude_to_cartesian(getIntersectionPosition(shortestPath.front()).lon()),latitude_to_cartesian(getIntersectionPosition(shortestPath.front()).lat()), Iwidth/30,0,360);
//                setcolor(BLACK);
//                drawtext(longitude_to_cartesian(getIntersectionPosition(shortestPath.front()).lon())+Iwidth/30,latitude_to_cartesian(getIntersectionPosition(shortestPath.front()).lat())-Iwidth/30, "Start", FLT_MAX, FLT_MAX,false);
//            
//            
//            setcolor(current_palette->get_colors().building_shadow);
//            fillarc(longitude_to_cartesian(getIntersectionPosition(shortestPath.back()).lon()),latitude_to_cartesian(getIntersectionPosition(shortestPath.back()).lat()), Iwidth/30,0,360);
//            
//            setcolor(BLACK);
//            drawtext(longitude_to_cartesian(getIntersectionPosition(shortestPath.back()).lon())+Iwidth/30,latitude_to_cartesian(getIntersectionPosition(shortestPath.back()).lat())-Iwidth/30, "Destination", FLT_MAX, FLT_MAX,false);
            }
            
            else{
                //cout <<"Klappa" << endl;
                drawShortPath = true;
                shortestPath = find_path_between_intersections(from,to,15);
                
//                settextattrs(13,0);
//           
//                setcolor(current_palette->get_colors().building);
//                fillarc(longitude_to_cartesian(getIntersectionPosition(shortestPath.front()).lon()),latitude_to_cartesian(getIntersectionPosition(shortestPath.front()).lat()), Iwidth/30,0,360);
//                setcolor(BLACK);
//                drawtext(longitude_to_cartesian(getIntersectionPosition(shortestPath.front()).lon())+Iwidth/30,latitude_to_cartesian(getIntersectionPosition(shortestPath.front()).lat())-Iwidth/30, "Start", FLT_MAX, FLT_MAX,false);
//            
//            
//            setcolor(current_palette->get_colors().building_shadow);
//            fillarc(longitude_to_cartesian(getIntersectionPosition(shortestPath.back()).lon()),latitude_to_cartesian(getIntersectionPosition(shortestPath.back()).lat()), Iwidth/30,0,360);
//            
//            setcolor(BLACK);
//            drawtext(longitude_to_cartesian(getIntersectionPosition(shortestPath.back()).lon())+Iwidth/30,latitude_to_cartesian(getIntersectionPosition(shortestPath.back()).lat())-Iwidth/30, "Destination", FLT_MAX, FLT_MAX,false);
//            
            }
           
            //cout << "Inter from: " << getIntersectionName(from) << " to " << getIntersectionName(to) <<endl;
            
        }
        else{
            toPOIPath =false;
            drawShortPath = false;
            cout <<"INvalid Path!" <<endl;
        }
    }
    
  //focus set for input boxes
    if (input1.clickedOn(x,y) || (input1.active && input1.menuSelection(x,y) != -1)){
        cout <<"FIRSTONE" << endl;
        input1.setActive(true);
         if (input1.menuSelection(x,y) != -1){
            input1.curText = get<0>(input1.textSuggestions[input1.menuSelection(x,y)]);
            input1.autocomplete=input1.textSuggestions[input1.menuSelection(x,y)];
            input1.drawBox();
        }
        
    }
    else{
        input1.setActive(false);
       
    }
      
    if (input2.clickedOn(x,y) || (input2.active && input2.menuSelection(x,y) != -1)){
        
        input2.setActive(true);
         if (input2.menuSelection(x,y) != -1){
            input2.curText = get<0>(input2.textSuggestions[input2.menuSelection(x,y)]);
            input2.autocomplete=input2.textSuggestions[input2.menuSelection(x,y)];
            input2.drawBox();
        }
        
    }
    else{
        input2.setActive(false);
        
    }
     draw_screen();
   LatLon click_pos(latitude,longitude);
    stringstream ss1;
    stringstream    ss2;
    ss1 << latitude;
    string lat = (ss1.str());
    ss2<< longitude;
    string lon =(ss2.str());
    unsigned i = find_closest_intersection(click_pos);
    unsigned j = find_closest_point_of_interest(click_pos);
    click = click_pos;
    if (notInTextOrInput && event.button != 2){
        setcolor(RED);
        fillarc(longitude_to_cartesian(getIntersectionPosition(i).lon()),latitude_to_cartesian(getIntersectionPosition(i).lat()), Iwidth/40,0,360);
        drawtext(longitude_to_cartesian(getIntersectionPosition(i).lon())+Iwidth/40,latitude_to_cartesian(getIntersectionPosition(i).lat())-Iwidth/40, "I", FLT_MAX, FLT_MAX,false);
        setcolor(PURPLE);
        fillarc(longitude_to_cartesian(getPointOfInterestPosition(j).lon()),latitude_to_cartesian(getPointOfInterestPosition(j).lat()), Iwidth/40,0,360);
        drawtext(longitude_to_cartesian(getPointOfInterestPosition(j).lon())+Iwidth/40,latitude_to_cartesian(getPointOfInterestPosition(j).lat())-Iwidth/40, "P", FLT_MAX, FLT_MAX,false);
    
        std::cout << "Closest Intersection: " << getIntersectionName(i) << "\n" << endl;
        update_message("Closest Intersection: " + getIntersectionName(i) + " | Closest POI: " + getPointOfInterestName(j) + " | Mouse @ Lat: " + lat + " Lon: " + lon );
    }
    
    copy_off_screen_buffer_to_screen();

//    XEvent e;
//    XSelectInput(x11_state.display,x11_state.toplevel, ButtonReleaseMask);
//    XNextEvent(x11_state.display,&e);
//    if (e.type==KeyRelease && swagButton.clicked(x,y)){
//        swagButton.isClicked =false;
//        swagButton.updateWorldConditions();
//        swagButton.drawButton();
//    }
        
    
   
    input1.updateWorldConditions();
    input1.drawBox();
    input2.updateWorldConditions();
    input2.drawBox();
    draw_text_input_field();
    copy_off_screen_buffer_to_screen();
}

void act_on_mousemove(float x, float y){
    /* When the mouse is moved, x and y values are passed into the function
     * based on where the mouse is, it will change the color of the dots
     * that show up on the screen
     */
    
    /* WILL WORK ONCE RANGE SEARCH IS PROPERLY IMPLEMENTED FOR KD-TREE */
    
    // condition to check if POI flag is checked
    if(false){

        // update screen where mouse was previously to clear previously drawn markers before drawing new markers
        draw_in_radius(mouse_move_prev[0],mouse_move_prev[1],MIN_HIGHLIGHT_DISTANCE);

        // turn x,y position from mouse into lat,lon for use with KD-Tree
        double lat = cartesian_to_lat(y);
        double lon = cartesian_to_lon(x);

        // call findNearestInRange to find closest POI within radius of MIN_HIGHLIGHT_DISTANCE
        vector <KDNode*> nodes_within_range = pointsOfInterestMap.findNearestInRange(LatLon(lat,lon),MIN_HIGHLIGHT_DISTANCE);

        // populate LatLon vector using found KDNodes
        vector <LatLon> within_range;
        for(unsigned i = 0; i < nodes_within_range.size(); i++){
            within_range.push_back(LatLon(nodes_within_range[i]->getCoords()[1],nodes_within_range[i]->getCoords()[0]));
        }

        // find closest POI out of the ones that are within range
        double shortest_distance = 1; //shortest distance only has to be larger than MIN_HIGHLIGHT_DISTANCE
        LatLon closest;

        for(auto iter = within_range.begin(); iter != within_range.end(); iter++){
        //for(unsigned i = 0; i < numPOI; i++){

            double x_POI = longitude_to_cartesian(iter->lon());
            double y_POI = latitude_to_cartesian(iter->lat());
            double x_diff2 = x - x_POI;
            double y_diff2 = y - y_POI;

            double distance = sqrt(x_diff2*x_diff2 + y_diff2*y_diff2);

            if(distance < shortest_distance){
                shortest_distance = distance;
                closest = *iter;
            }
        }

        // start drawing results onto screen
        set_drawing_buffer(OFF_SCREEN);
        setlinewidth(2);

        if(within_range.size() != 0){

            // draw all nearby POI in ORANGE
            for(auto iter = within_range.begin(); iter != within_range.end(); iter++){
                double x_POI = longitude_to_cartesian(iter->lon());
                double y_POI = latitude_to_cartesian(iter->lat());
                setcolor(ORANGE);
                fillarc(x_POI,y_POI,0.0002,0,360);
                setcolor(SADDLEBROWN);
                drawarc(x_POI,y_POI,0.0002,0,360);
            }

            // draw POI closest to mouse in RED
            double x_POI = longitude_to_cartesian(closest.lon());
            double y_POI = latitude_to_cartesian(closest.lat());
            setcolor(RED);
            fillarc(x_POI,y_POI,0.0002,0,360);
            setcolor(SADDLEBROWN);
        }

        // copy graphics onto screen
        copy_off_screen_buffer_to_screen();
        
    }
    
    // update previous mouse location for next cycle
    mouse_move_prev[0] = x;
    mouse_move_prev[1] = y;
    
}

void draw_info_box(string name, string type, unsigned id, float Ix ,float Iy , float Iwidth ,float Iheight ){
    string nameData, typeData, featureType;
    string lat, lon;
    string oneChar = "X";
    float blinkX = 680;
    float blinkY = 444;
    stringstream ss1;
    stringstream ss2;
    setcolor(WHITE);
    fillrect(Ix,Iy,Ix+Iwidth,Iy-Iheight);
    setcolor(DARK_GRAY);
    drawrect(Ix,Iy,Ix+Iwidth,Iy-Iheight);
    setcolor(RED);
    drawrect(Ix + 38*Iwidth/40 ,Iy,Ix+Iwidth,Iy - 2*Iwidth/40);
    int textwidth, textheight;
    const char* text = oneChar.c_str();  
    int text_byte_length = strlen(text);

    font_ptr current_font = gl_state.font_info.get_font_info(
        gl_state.currentfontsize,
        gl_state.currentfontrotation);
    XGlyphInfo extents;
    XftTextExtentsUtf8(x11_state.display, current_font, reinterpret_cast<const FcChar8*>(text), text_byte_length, &extents);
    textwidth = extents.width;
    textheight = extents.height;
    //highlight = false;
    drawtext(Ix+Iwidth-textwidth*get_trans_coord().stow_xmult, Iy, "X",FLT_MAX,FLT_MAX,false);
    setcolor(BLACK);
    switch(type[0]){
        case 'S':
            typeData = "Street";
            nameData = name;
            setcolor(BLACK);
            highlight = true;
            highlightID = id;
            //settextattrs(10,0);
            drawtext(Ix, Iy+textheight*get_trans_coord().stow_ymult/2, nameData,FLT_MAX,FLT_MAX,false);
            drawtext(Ix, Iy-(Iheight/10)+textheight*get_trans_coord().stow_ymult/2, typeData,FLT_MAX,FLT_MAX,false);
            break;
        case 'P':
            nameData = name;
            typeData = "Point of Interest";
            ss1.clear();
            ss2.clear();
            ss1 << getPointOfInterestPosition(id).lat();
            lat = (ss1.str());
            ss2<< getPointOfInterestPosition(id).lon();
            lon =(ss2.str());
            setcolor(BLACK);
            //settextattrs(10,0);
            drawtext(Ix, Iy+textheight*get_trans_coord().stow_ymult/2, nameData,FLT_MAX,FLT_MAX,false);
            drawtext(Ix, Iy-(Iheight/10)+textheight*get_trans_coord().stow_ymult/2, typeData,FLT_MAX,FLT_MAX,false);
            drawtext(Ix, Iy-(2*Iheight/10)+textheight*get_trans_coord().stow_ymult/2, "LAT: " + lat,FLT_MAX,FLT_MAX,false);
            drawtext(Ix, Iy-(3*Iheight/10)+textheight*get_trans_coord().stow_ymult/2, "LON: " + lon,FLT_MAX,FLT_MAX,false);
            setcolor(RED);
            fillarc(longitude_to_cartesian(getPointOfInterestPosition(id).lon()),latitude_to_cartesian(getPointOfInterestPosition(id).lat()), Iwidth/25,0,360);
            setcolor(PINK);
            fillarc(longitude_to_cartesian(getPointOfInterestPosition(id).lon()),latitude_to_cartesian(getPointOfInterestPosition(id).lat()), Iwidth/50,0,360);
            if (!blinked){
                set_visible_world(longitude_to_cartesian(getPointOfInterestPosition(id).lon()) - blinkX,latitude_to_cartesian(getPointOfInterestPosition(id).lat())-blinkY,longitude_to_cartesian(getPointOfInterestPosition(id).lon()) + blinkX,latitude_to_cartesian(getPointOfInterestPosition(id).lat())+blinkY);
                blinked = true;
                draw_screen(); 
            }
            break;
        case 'I':
            nameData = name;
            typeData = "Intersection";
            ss1.clear();
            ss2.clear();
            ss1 << getIntersectionPosition(id).lat();
            lat = (ss1.str());
            ss2<< getIntersectionPosition(id).lon();
            lon =(ss2.str());
            if (text_field.find('&')){
                int i = 0;
                vector<string> tokens;
                tokens.empty();
                string name1,name2;
                stringstream ss(text_field);
                string item;
                char delim = '&';

                while(getline(ss,item,delim)){
                    tokens.push_back(item);
                    i++;
                }
                if (i >=2){
                    name1 = tokens[0].substr(0,tokens[0].length()-1);
                    name2 = tokens[1].substr(1,tokens[1].length()-1);
                    vector<unsigned> inter = find_intersection_ids_from_street_names(name1,name2);
                    for (unsigned i = 0; i < inter.size();i++){
                        setcolor(PINK);
                        fillarc(longitude_to_cartesian(getIntersectionPosition(inter[i]).lon()),latitude_to_cartesian(getIntersectionPosition(inter[i]).lat()), Iwidth/50,0,360);
                    }
                    
                }
            }
            
            setcolor(BLACK);
            drawtext(Ix, Iy+textheight*get_trans_coord().stow_ymult/2, nameData,FLT_MAX,FLT_MAX,false);
            drawtext(Ix, Iy-(Iheight/10)+textheight*get_trans_coord().stow_ymult/2, typeData,FLT_MAX,FLT_MAX,false);
            drawtext(Ix, Iy-(2*Iheight/10)+textheight*get_trans_coord().stow_ymult/2, "LAT: " + lat,FLT_MAX,FLT_MAX,false);
            drawtext(Ix, Iy-(3*Iheight/10)+textheight*get_trans_coord().stow_ymult/2, "LON: " + lon,FLT_MAX,FLT_MAX,false);
            setcolor(GREEN);
            fillarc(longitude_to_cartesian(getIntersectionPosition(id).lon()),latitude_to_cartesian(getIntersectionPosition(id).lat()), Iwidth/25,0,360);
            setcolor(PINK);
            fillarc(longitude_to_cartesian(getIntersectionPosition(id).lon()),latitude_to_cartesian(getIntersectionPosition(id).lat()), Iwidth/50,0,360);
            if (!blinked){
                set_visible_world(longitude_to_cartesian(getIntersectionPosition(id).lon()) - blinkX,latitude_to_cartesian(getIntersectionPosition(id).lat())-blinkY,longitude_to_cartesian(getIntersectionPosition(id).lon()) + blinkX,latitude_to_cartesian(getIntersectionPosition(id).lat())+blinkY);
                blinked = true;
                draw_screen(); 
            }
            break;
        case 'F':
            nameData = name;
            typeData = "Feature";
            featureType = featureDataTypeStr(getFeatureType(id));
            ss1.clear();
            ss2.clear();
            ss1 << getFeaturePoint(id,0).lat();
            lat = (ss1.str());
            ss2<< getFeaturePoint(id,0).lon();
            lon =(ss2.str());
            
            setcolor(BLACK);
            drawtext(Ix, Iy+textheight*get_trans_coord().stow_ymult/2, nameData,FLT_MAX,FLT_MAX,false);
            drawtext(Ix, Iy-(Iheight/10)+textheight*get_trans_coord().stow_ymult/2, typeData,FLT_MAX,FLT_MAX,false);
            drawtext(Ix, Iy-(2*Iheight/10)+textheight*get_trans_coord().stow_ymult/2, featureType,FLT_MAX,FLT_MAX,false);
            drawtext(Ix, Iy-(3*Iheight/10)+textheight*get_trans_coord().stow_ymult/2, "LAT: " + lat,FLT_MAX,FLT_MAX,false);
            drawtext(Ix, Iy-(4*Iheight/10)+textheight*get_trans_coord().stow_ymult/2, "LON: " + lon,FLT_MAX,FLT_MAX,false);
            setcolor(BLUE);
            fillarc(longitude_to_cartesian(getFeaturePoint(id,0).lon()),latitude_to_cartesian(getFeaturePoint(id,0).lat()), Iwidth/25,0,360);
            setcolor(DARKGREEN);
            fillarc(longitude_to_cartesian(getFeaturePoint(id,0).lon()),latitude_to_cartesian(getFeaturePoint(id,0).lat()), Iwidth/50,0,360);
            if (!blinked){
                set_visible_world(longitude_to_cartesian(getFeaturePoint(id,0).lon()) - blinkX,latitude_to_cartesian(getFeaturePoint(id,0).lat())-blinkY,longitude_to_cartesian(getFeaturePoint(id,0).lon()) + blinkX,latitude_to_cartesian(getFeaturePoint(id,0).lat())+blinkY);
                blinked = true;
                draw_screen();

            }      
    
            break;
        default:
            break;
            
    }
}

void draw_text_input_field(){
    // cout<<"Goes into function " << text_field << endl;
    Twidth = get_visible_world().get_width()/4;
    Theight = get_visible_world().get_height()/20 ;
    Tx = get_visible_world().right() - Twidth;
    Ty = get_visible_world().top();
    double size = 15;
    
    setcolor(gl_state.background_color);
    fillrect(Tx,Ty,Tx+Twidth,Ty-Theight);
    if (enable_text_input)
        setcolor(PASTEL_YELLOW);
    else 
        setcolor(gl_state.background_color);
    
    fillrect(Tx,Ty,Tx+Twidth,Ty-Theight);
//    setcolor(gl_state.background_color);
//    fillrect(Tx,Ty,Tx+Twidth,Ty-Theight);
    setcolor(WARM_GRAY);
    setlinewidth(2);
    drawline(Tx,Ty,Tx+Twidth,Ty);
    drawline(Tx,Ty,Tx,Ty-Theight);
    setcolor(DARK_GRAY);
    drawline(Tx,Ty-Theight,Tx+Twidth,Ty-Theight);
    drawline(Tx+Twidth,Ty,Tx+Twidth,Ty);
    
    settextattrs(size,0);
    if (text_field.length() >0){
        int textwidth, textheight;
        int cutoff=0;
        do{
            const char* text = text_field.substr(cutoff).c_str();  
            int text_byte_length = strlen(text);
            font_ptr current_font = gl_state.font_info.get_font_info(
                gl_state.currentfontsize,
                gl_state.currentfontrotation);
            XGlyphInfo extents;
            XftTextExtentsUtf8(x11_state.display, current_font, reinterpret_cast<const FcChar8*>(text), text_byte_length, &extents);
            textwidth = extents.width;
            textheight = extents.height;
            cutoff++;
        }
        
        while(textwidth*get_trans_coord().stow_xmult>Twidth && cutoff != text_field.length());
        setcolor(DARK_GRAY);
        drawtext(Tx, Ty -Theight/2 - textheight*get_trans_coord().stow_ymult/2, get<0>(text_field_suggest),FLT_MAX, FLT_MAX,false);
        setcolor(BLACK);
        drawtext(Tx, Ty -Theight/2- textheight*get_trans_coord().stow_ymult/2, (cutoff < 1)?0:text_field.substr(cutoff-1),FLT_MAX, FLT_MAX,false);
        if (!textSuggestions.empty()){
            for (unsigned i = 0; i < textSuggestions.size() && enable_text_input; i ++){

                setcolor(gl_state.background_color);
                fillrect(Tx,Ty - Theight*(i+1),Tx+Twidth,Ty-Theight*(i+2));
                setcolor(LIGHT_GRAY_TRANS);

                fillrect(Tx,Ty - Theight*(i+1),Tx+Twidth,Ty-Theight*(i+2));
                setcolor(WARM_GRAY);
                setlinewidth(2);
                drawline(Tx,Ty - Theight*(i+1),Tx+Twidth,Ty - Theight*(i+1));
                drawline(Tx,Ty - Theight*(i+1),Tx,Ty - Theight*(i+2));
                setcolor(DARK_GRAY);
                drawline(Tx,Ty - Theight*(i+1),Tx+Twidth,Ty - Theight*(i+1));
                drawline(Tx+Twidth,Ty-Theight*(i+2),Tx+Twidth,Ty - Theight*(i+2));

                setcolor(BLACK);
                drawtext(Tx, Ty -Theight/2 - textheight*get_trans_coord().stow_ymult/2 - Theight*(i+1), get<0>(textSuggestions[i]) + " " + get<1>(textSuggestions[i]) ,FLT_MAX, FLT_MAX,false);

            }
    }
        //drawline(x,0,x,100);
    }
}

//checks whether a point is on the screen
bool check_point_on_screen(LatLon point){
    double lat = point.lat();
    double lon = point.lon();
    double x = longitude_to_cartesian(lon);
    double y = latitude_to_cartesian(lat);
    t_bound_box dim = screen->get_screen_state_prev();
    if(x >= dim.left() && x <= dim.right() && y >= dim.bottom() && y <= dim.top()){
        return true;
    }
    //return false;
    return true;
}

bool check_point_on_screen(double x, double y){
    t_bound_box dim = screen->get_screen_state_prev();
    if(x >= dim.left() && x <= dim.right() && y >= dim.bottom() && y <= dim.top()){
        return true;
    }
    //return false;
    return true;
}

bool check_on_screen(vector<LatLon> points){
    bool on_screen;
    for(auto iter = points.begin(); iter != points.end(); iter++){
        on_screen = check_point_on_screen(*iter);
    }
    return on_screen;
}

bool check_on_screen(vector<t_point> points){
    bool on_screen;
    for(auto iter = points.begin(); iter != points.end(); iter++){
        on_screen = check_point_on_screen(iter->x,iter->y);
    }
    return on_screen;
}

string featureDataTypeStr(FeatureType f){
     switch (f){
            case 0:
                return "<unknown>";
                break;
            case Park:
                return "<Park>";
                break;
            case Greenspace:
                return "<Greenspace>";
                break;
            case Golfcourse:
                return "<Golfcourse>";
                break;
            case Island:
                return "<Island>";
                break;
            case Lake:
                return "<Lake>";
                break;
            case River:
                return "<River>";
                break;
            case Stream:
                return "<Stream>";
                break;
            case Beach:
                return "<Beach>";
                break;
            case Shoreline:
                return "<Shoreline>";
                break;
            case Building:
                return "<Building>";
                break;
            default:
                return "<unknown>";
                break;
        }
}

double check_street_size(double x){
    //set thickness/color of the line depending on the size/direction of the streets
    setfontsize(2);
    if(x > 0.7*EARTH_RADIUS_IN_METERS){
        return 0;
    }else if(x > 0.0025*EARTH_RADIUS_IN_METERS){
        return 1;
    }else if(x > 0.0015*EARTH_RADIUS_IN_METERS){
        return 2;
    }else if(x > 0.0005*EARTH_RADIUS_IN_METERS){
        return 3;
    }else if(x > 0.0003*EARTH_RADIUS_IN_METERS){
        return 6;
    }else if(x > 0.0002*EARTH_RADIUS_IN_METERS){
        return 10;
    }else if(x > 0.0001*EARTH_RADIUS_IN_METERS){
        return 15;
    }else if(x > 0.00005*EARTH_RADIUS_IN_METERS){
        return 25;
    }else if(x < 0.0001*EARTH_RADIUS_IN_METERS){
        return 35;
    }
    return 0;
}

float check_angle(unsigned x1,unsigned y1,unsigned x2,unsigned y2){
    float degree = atan2((y2-y1),(x2-x1))*180/PI;
    if(x2>x1){
        return degree;
    }else{
        return (degree+180); 
    }
}

//global variable that keep tracks of the total distance of one street
double totalDistance;

void draw_street_segment(unsigned street_seg_id){
    
    unsigned numCurvedPoints = latLonStreetSeg[street_seg_id].size();
    
    vector<LatLon> points;
    points.push_back(latLonIntersection[searchByStreetSeg[street_seg_id].from]);
    for(unsigned c = 0; c < numCurvedPoints; c++){
        points.push_back(latLonStreetSeg[street_seg_id][c]);
    }
    points.push_back(latLonIntersection[searchByStreetSeg[street_seg_id].to]);
    
//    bool oneway = searchByStreetSeg[street_seg_id].oneWay;
//    
//    if(oneway)
//        setcolor(DARKGREEN);
    
    if(check_on_screen(points)){
        
        //draw street for ones without curve points
        if(numCurvedPoints == 0){
            LatLon point1 = latLonIntersection[searchByStreetSeg[street_seg_id].from];
            LatLon point2 = latLonIntersection[searchByStreetSeg[street_seg_id].to];
            double x1 = longitude_to_cartesian(point1.lon());
            double y1 = latitude_to_cartesian(point1.lat());
            double x2 = longitude_to_cartesian(point2.lon());
            double y2 = latitude_to_cartesian(point2.lat());
            drawline(x1,y1,x2,y2);
        }
        //draw street for ones with curve points
        else{
            LatLon point1 = latLonIntersection[searchByStreetSeg[street_seg_id].from];
            LatLon point2 = latLonStreetSeg[street_seg_id][0];
            double x1 = longitude_to_cartesian(point1.lon());
            double y1 = latitude_to_cartesian(point1.lat());
            double x2 = longitude_to_cartesian(point2.lon());
            double y2 = latitude_to_cartesian(point2.lat());
            setlinestyle(0.1, ROUND);
            drawline(x1,y1,x2,y2);
            //set thickness/color of the line depending on the size/direction of the streets

            for(unsigned c = 0; (c+1) < numCurvedPoints; c++){
                point1 = latLonStreetSeg[street_seg_id][c];
                point2 = latLonStreetSeg[street_seg_id][c+1];
                x1 = longitude_to_cartesian(point1.lon());
                y1 = latitude_to_cartesian(point1.lat());
                x2 = longitude_to_cartesian(point2.lon());
                y2 = latitude_to_cartesian(point2.lat());
                setlinestyle(0.1, ROUND);
                drawline(x1,y1,x2,y2);
            }

            point1 = latLonStreetSeg[street_seg_id][numCurvedPoints-1];
            point2 = latLonIntersection[searchByStreetSeg[street_seg_id].to];
            x1 = longitude_to_cartesian(point1.lon());
            y1 = latitude_to_cartesian(point1.lat());
            x2 = longitude_to_cartesian(point2.lon());
            y2 = latitude_to_cartesian(point2.lat());
            setlinestyle(0.1, ROUND);
            drawline(x1,y1,x2,y2);
        }
    }
}

void draw_street_segment(unsigned street_seg_id, const t_color& seg_color){
    setcolor(seg_color);
    
    unsigned numCurvedPoints = latLonStreetSeg[street_seg_id].size();
    
//    bool oneway = searchByStreetSeg[street_seg_id].oneWay;
//    
//    if(oneway)
//        setcolor(DARKGREEN);
    
    vector<LatLon> points;
    points.push_back(latLonIntersection[searchByStreetSeg[street_seg_id].from]);
    for(unsigned c = 0; c < numCurvedPoints; c++){
        points.push_back(latLonStreetSeg[street_seg_id][c]);
    }
    points.push_back(latLonIntersection[searchByStreetSeg[street_seg_id].to]);
    
    if(check_on_screen(points)){
    
        //draw street for ones without curve points
        if(numCurvedPoints == 0){
            LatLon point1 = latLonIntersection[searchByStreetSeg[street_seg_id].from];
            LatLon point2 = latLonIntersection[searchByStreetSeg[street_seg_id].to];
            double x1 = longitude_to_cartesian(point1.lon());
            double y1 = latitude_to_cartesian(point1.lat());
            double x2 = longitude_to_cartesian(point2.lon());
            double y2 = latitude_to_cartesian(point2.lat());
            drawline(x1,y1,x2,y2);
        }
        //draw street for ones with curve points
        else{
            LatLon point1 = latLonIntersection[searchByStreetSeg[street_seg_id].from];
            LatLon point2 = latLonStreetSeg[street_seg_id][0];
            double x1 = longitude_to_cartesian(point1.lon());
            double y1 = latitude_to_cartesian(point1.lat());
            double x2 = longitude_to_cartesian(point2.lon());
            double y2 = latitude_to_cartesian(point2.lat());
            setlinestyle(0.1, ROUND);
            drawline(x1,y1,x2,y2);
            //set thickness/color of the line depending on the size/direction of the streets

            for(unsigned c = 0; (c+1) < numCurvedPoints; c++){
                point1 = latLonStreetSeg[street_seg_id][c];
                point2 = latLonStreetSeg[street_seg_id][c+1];
                x1 = longitude_to_cartesian(point1.lon());
                y1 = latitude_to_cartesian(point1.lat());
                x2 = longitude_to_cartesian(point2.lon());
                y2 = latitude_to_cartesian(point2.lat());
                setlinestyle(0.1, ROUND);
                drawline(x1,y1,x2,y2);
            }

            point1 = latLonStreetSeg[street_seg_id][numCurvedPoints-1];
            point2 = latLonIntersection[searchByStreetSeg[street_seg_id].to];
            x1 = longitude_to_cartesian(point1.lon());
            y1 = latitude_to_cartesian(point1.lat());
            x2 = longitude_to_cartesian(point2.lon());
            y2 = latitude_to_cartesian(point2.lat());
            setlinestyle(0.1, ROUND);
            drawline(x1,y1,x2,y2);
        }
    }
}

void draw_street_type(unsigned type, const t_color& color,double size){
    
    setlinewidth(size);
    
    if(type == MOTORWAY){
        for(unsigned i = 0; i < motorwayStreets.size(); i++){
            draw_street_segment(motorwayStreets[i],color);
        }
    }
    else if(type == TRUCK){
        for(unsigned i = 0; i < truckStreets.size(); i++){
            draw_street_segment(truckStreets[i],color);
        }
    }
    else if(type == PRIMARY){
        for(unsigned i = 0; i < primaryStreets.size(); i++){
            draw_street_segment(primaryStreets[i],color);
        }
    }
    else if(type == SECONDARY){
        for(unsigned i = 0; i < secondaryStreets.size(); i++){
            draw_street_segment(secondaryStreets[i],color);
        }
    }
    else if(type == TERTIARY){
        for(unsigned i = 0; i < tertiaryStreets.size(); i++){
            draw_street_segment(tertiaryStreets[i],color);
        }
    }
    else if(type == REGULAR){
        for(unsigned i = 0; i < regularStreets.size(); i++){
            draw_street_segment(regularStreets[i],color);
        }
    }
    else if(type == SMALL){
        for(unsigned i = 0; i < smallStreets.size(); i++){
            draw_street_segment(smallStreets[i],color);
        }
    }
}

void draw_street(vector<unsigned> street_segs, int width, const t_color& color){
    setlinewidth(width);
    for(unsigned i = 0; i < street_segs.size(); i++){
        draw_street_segment(street_segs[i],color);
    }
}

void draw_streets(){
    
    t_bound_box dim = get_visible_world();
    float x = dim.get_width();
    float y = dim.get_height();
    
    Colors color = current_palette->get_colors();

    double size = check_street_size(x);

    setlinestyle(0.1, ROUND);
        
    if(x > 0.0015*EARTH_RADIUS_IN_METERS){

        draw_street_type(SECONDARY,color.street_3,size);

        draw_street_type(PRIMARY,color.street_3,size);

        draw_street_type(TRUCK,color.street_2,size);

        draw_street_type(MOTORWAY,color.street_1,size);
            
    }else if(x > 0.0012*EARTH_RADIUS_IN_METERS){

        draw_street_type(REGULAR,color.street_highlight,size+2);
        draw_street_type(REGULAR,color.street_4,size);
        
        draw_street_type(TERTIARY,color.street_highlight,size+2);
        draw_street_type(TERTIARY,color.street_3,size);

        draw_street_type(SECONDARY,color.street_highlight,size+2);
        draw_street_type(SECONDARY,color.street_3,size);

        draw_street_type(PRIMARY,color.street_highlight,size+2);
        draw_street_type(PRIMARY,color.street_3,size);

        draw_street_type(TRUCK,color.street_highlight,size+2);
        draw_street_type(TRUCK,color.street_2,size);

        draw_street_type(MOTORWAY,color.street_1,size);
            
    }else{

        draw_street_type(SMALL,color.street_highlight,size+2.5);
        draw_street_type(SMALL,color.street_4,size);
        
        draw_street_type(REGULAR,color.street_highlight,size+2.5);
        draw_street_type(REGULAR,color.street_4,size);
        
        draw_street_type(TERTIARY,color.street_highlight,size+2.5);
        draw_street_type(TERTIARY,color.street_3,size);

        draw_street_type(SECONDARY,color.street_highlight,size+2.5);
        draw_street_type(SECONDARY,color.street_3,size);

        draw_street_type(PRIMARY,color.street_highlight,size+2.5);
        draw_street_type(PRIMARY,color.street_3,size);

        draw_street_type(TRUCK,color.street_highlight,size+2.5);
        draw_street_type(TRUCK,color.street_2,size);

        draw_street_type(MOTORWAY,color.street_1,size);
        
    }
}

void draw_street(const vector<unsigned>& street_seg_ids){
    for(unsigned i = 0; i < street_seg_ids.size(); i++){
        setlinewidth(10);
        draw_street_segment(street_seg_ids[i],MAGENTA);
    }
}

//check zoom levels and distance traveled down the street to decide whether to draw the street or not
bool check_street_names(unsigned distance,unsigned x){
    double firstZoomLevel = 0.00015*EARTH_RADIUS_IN_METERS;
    double secondZoomLevel = 0.0001*EARTH_RADIUS_IN_METERS;
    double thirdZoomLevel = 0.00005*EARTH_RADIUS_IN_METERS;
    double fourthZoomLevel = 0.00003*EARTH_RADIUS_IN_METERS;
    double fifthZoomLevel = 0.00001*EARTH_RADIUS_IN_METERS;
    
    if(totalDistance > 150 && x > firstZoomLevel
       ||totalDistance > 100 && x > secondZoomLevel
       ||totalDistance > 60 && x > thirdZoomLevel){
        if(distance < 100 && x > firstZoomLevel){
            totalDistance -= 150;
            return false;
        }else if(distance < 50 && x > secondZoomLevel){
            totalDistance -= 100;
            return false;
        }else if(distance < 45 && x > thirdZoomLevel){
            totalDistance -= 60;
            return false;
        }else if(distance < 35 && x > fourthZoomLevel){
            totalDistance -= 60;
            return false;
        }else if(distance < 20 && x > fifthZoomLevel){
            totalDistance -= 60;
            return false;
        }else if(distance < 5){
            totalDistance -= 60;
            return false;
        }else{
            totalDistance -= 60;
            return true;
        }
    }else
        return false;
    
    return false;
    
}

void draw_street_names(){
    
    t_bound_box dim = get_visible_world();
    float x = dim.get_width();
    float y = dim.get_height();
        
    if(x < 0.0004*EARTH_RADIUS_IN_METERS){
        setcolor(BLACK);      
        int count = 0;
        int scale = 4;
        
        for(unsigned i = 0;i<numStreet;i++){
            scale = 10;
            string name = streetNames[i]; 
            string original = name;

            totalDistance = 0;
            for(auto itr = (searchIntersectionsByStreetID[i]).begin();(itr+1)<(searchIntersectionsByStreetID[i]).end();itr++){
                name = original;
                LatLon point1 = latLonIntersection[*itr];
                LatLon point2 = latLonIntersection[*(itr+1)];
                float x1 = longitude_to_cartesian(point1.lon());
                float y1 = latitude_to_cartesian(point1.lat());
                float x2 = longitude_to_cartesian(point2.lon());
                float y2 = latitude_to_cartesian(point2.lat());
                float street_Seg_ID;

                //find sorted streetSegID to access the streetSegInfo
                unsigned index = itr-(searchIntersectionsByStreetID[i]).begin();
                if( index < searchStreetSegByStreetID_inorder[i].size() ){
                    street_Seg_ID = searchStreetSegByStreetID_inorder[i][index];
                }               

                unsigned numCurvedPoints = latLonStreetSeg[street_Seg_ID].size();
                bool oneWay = searchByStreetSeg[street_Seg_ID].oneWay;
                unsigned from = searchByStreetSeg[street_Seg_ID].from;
                unsigned to = searchByStreetSeg[street_Seg_ID].to;
                if(oneWay){
                    if(point1.lat() == latLonIntersection[from].lat()){
                        string temp_name = "->" + name + "->";
                        name = temp_name;
                    }else if(point1.lat() == latLonIntersection[to].lat()){
                        string temp_name = "<-" + name + "<-";
                        name = temp_name;
                    }
                }

                if(numCurvedPoints == 0){
                    double distance = find_distance_between_two_points(point1,point2);

                    if(oneWay){
                        if(/*closest1 < 10 &&*/ x > 0.00024*EARTH_RADIUS_IN_METERS){
                            continue;
                        }else
                            scale = 8;
                    }else if(distance < 50){
                        continue;
                    }else if(distance < 150 && x > 0.00015*EARTH_RADIUS_IN_METERS){
                        continue;
                    }else if(distance < 120 && x > 0.0001*EARTH_RADIUS_IN_METERS){
                        continue;
                    }else{
                        scale = 1;
                    }


                    float degree = atan2((y2-y1),(x2-x1))*180/PI;
                    if(x2>x1){
                        settextrotation(degree); 
                    }else{
                        settextrotation(180+degree); 
                    }
                    count++;
                    if(!check_point_on_screen(((x2+x1)/2),((y2+y1)/2)))
                        continue;
                    if(name != "<unknown>" && count%scale == 0){
                        drawtext(((x2+x1)/2),((y2+y1)/2),name); 
                        count++;
                    }
                }

                else{
                    LatLon point1 = latLonIntersection[searchByStreetSeg[street_Seg_ID].from];
                    LatLon point2 = latLonStreetSeg[street_Seg_ID][0];
                    x1 = longitude_to_cartesian(point1.lon());
                    y1 = latitude_to_cartesian(point1.lat());
                    x2 = longitude_to_cartesian(point2.lon());
                    y2 = latitude_to_cartesian(point2.lat());
                    double distance = find_distance_between_two_points(point1,point2);
                    totalDistance+=distance;
                    float degree = atan2((y2-y1),(x2-x1))*180/PI;
                    if(x2>x1){
                        settextrotation(degree); 
                    }else{
                        settextrotation(180+degree); 
                    }
                    bool street = check_street_names(distance,x);
                    if(!check_point_on_screen(((x2+x1)/2),((y2+y1)/2)))
                        continue;
                    if(name != "<unknown>" && street){
                        drawtext(((x2+x1)/2),((y2+y1)/2),name); 
                        count++;
                    }

                    for(unsigned c = 0; (c+1) < numCurvedPoints; c++){
                        point1 = latLonStreetSeg[street_Seg_ID][c];
                        point2 = latLonStreetSeg[street_Seg_ID][c+1];
                        x1 = longitude_to_cartesian(point1.lon());
                        y1 = latitude_to_cartesian(point1.lat());
                        x2 = longitude_to_cartesian(point2.lon());
                        y2 = latitude_to_cartesian(point2.lat());
                        distance = find_distance_between_two_points(point1,point2);
                        totalDistance+=distance;
                        degree = atan2((y2-y1),(x2-x1))*180/PI;
                        if(x2>x1){
                            settextrotation(degree); 
                        }else{
                            settextrotation(180+degree); 
                        }
                        street = check_street_names(distance,x);
                        if(!check_point_on_screen(((x2+x1)/2),((y2+y1)/2)))
                            continue;
                        if(name != "<unknown>" && street){
                            drawtext(((x2+x1)/2),((y2+y1)/2),name); 
                            count++;
                        }
                    }

                    point1 = latLonStreetSeg[street_Seg_ID][numCurvedPoints-1];
                    point2 = latLonIntersection[searchByStreetSeg[street_Seg_ID].to];
                    x1 = longitude_to_cartesian(point1.lon());
                    y1 = latitude_to_cartesian(point1.lat());
                    x2 = longitude_to_cartesian(point2.lon());
                    y2 = latitude_to_cartesian(point2.lat());
                    distance = find_distance_between_two_points(point1,point2);
                    totalDistance+=distance;

                    degree = atan2((y2-y1),(x2-x1))*180/PI;
                    if(x2>x1){
                        settextrotation(degree); 
                    }else{
                        settextrotation(180+degree); 
                    }

                    street = check_street_names(distance,x);
                    if(!check_point_on_screen(((x2+x1)/2),((y2+y1)/2)))
                        continue;
                    if(name != "<unknown>" && street){
                        drawtext(((x2+x1)/2),((y2+y1)/2),name); 
                        count++;
                    }
                }
            }
            name = "";
        }
    }
}

void highlight_street(unsigned street_id){
    setcolor(BLUE);
    t_bound_box dim = get_visible_world();
    double x = dim.get_width();
    double size = check_street_size(x);
    setlinewidth(size);
    setcolor(BLUE);
    if (highlight)
        cout << "Street is : " << getStreetName(street_id) << endl;
    vector<unsigned> allStreetSeg = searchStreetSegByStreetID[street_id];
    for(auto itr = allStreetSeg.begin();itr<allStreetSeg.end();itr++){
        cout << "Actual street name is " << streetNames[getStreetSegmentInfo(*itr).streetID] << endl;
        unsigned index = itr-allStreetSeg.begin();
        draw_street_segment(*itr);
    } 
}

void draw_features(FeatureType drawtype){
    
    for (auto iter = FeaturesVector.begin(); iter != FeaturesVector.end(); iter++){
        vector<t_point> vertexes;
        if ((*iter).type == drawtype){
        for (unsigned i = 0; i < (*iter).numFeaturePts; i++){
            float x = longitude_to_cartesian((*iter).featurePts[i].lon());
            float y = latitude_to_cartesian((*iter).featurePts[i].lat());
            t_point newPoint (x,y);
            vertexes.push_back(newPoint);
        }
        
        Colors color = current_palette->get_colors();
        
        if(!check_on_screen(vertexes))
            continue;
        
        switch ((*iter).type){
            case 0:
                setcolor(BLACK);
                break;
            case Park:
            case Greenspace:
            case Golfcourse:
            case Island:
                setcolor(color.greenery);
                break;
            case Lake:
            case River:
            case Stream:
                setcolor(color.water);
                break;
            case Beach:
            case Shoreline:
                setcolor(color.sand);
                break;
            case Building:
                setcolor(color.building_shadow);
                break;
            default:
                setcolor(color.building);
        }
        setfontsize(2);
          
        float x=get_visible_world().get_width();
        if(x > 0.7*EARTH_RADIUS_IN_METERS*DEG_TO_RAD){
            if(iter->type == Building)
                continue;
            setlinewidth(0);
        }else if(x > 0.25*EARTH_RADIUS_IN_METERS*DEG_TO_RAD){
            if(iter->type == Building)
                continue;
            setlinewidth(1);
        }else if(x > 0.15*EARTH_RADIUS_IN_METERS*DEG_TO_RAD){
            if(iter->type == Building)
                continue;
            setlinewidth(1);
        }else if(x > 0.05*EARTH_RADIUS_IN_METERS*DEG_TO_RAD){
            if(iter->type == Building)
                continue;
            setlinewidth(2);
            setfontsize(5);
        }else if(x > 0.03*EARTH_RADIUS_IN_METERS*DEG_TO_RAD){
            setlinewidth(3);
            setfontsize(6);
        }else if(x > 0.02*EARTH_RADIUS_IN_METERS*DEG_TO_RAD){
            setlinewidth(4);
            setfontsize(8);
        }else if(x > 0.01*EARTH_RADIUS_IN_METERS*DEG_TO_RAD){
            setlinewidth(5);
            setfontsize(10);
        }else if(x > 0.005*EARTH_RADIUS_IN_METERS*DEG_TO_RAD){
            setlinewidth(6);
            setfontsize(12);
        }else{
            setlinewidth(8);
            setfontsize(14);
        }
    
        //cout << "WHY IS IT SEGFAULTING" << endl;
        
        setlinestyle(0.1, ROUND);
        if (drawtype == (*iter).type){
                if (vertexes[0].x != vertexes[vertexes.size()-1].x || vertexes[0].y != vertexes[vertexes.size()-1].y){
                    for(unsigned i = 0; i < vertexes.size()-1;i++){
                        drawline(vertexes[i].x,vertexes[i].y,vertexes[i+1].x,vertexes[i+1].y);
                    }
                }
                else{
                    if(iter->type == Building){
                        fillpoly(&vertexes.front(), vertexes.size());
                        setcolor(color.building);
                        for(unsigned i = 0; i< vertexes.size(); i++){
                            vertexes[i].x -= 2;
                            vertexes[i].y += 2;
                        }
                        fillpoly(&vertexes.front(), vertexes.size());
                    }
                    else{
                        fillpoly(&vertexes.front(), vertexes.size());
                    }
                }
            }
            vertexes.clear();
        }
    }
}

void draw_test_point(double lat, double lon, const Surface& icon2, bool test){
    
    auto const x = longitude_to_cartesian(lon);
    auto const y = latitude_to_cartesian(lat);
    
    if(test){
        draw_surface(icon2,x,y);
    }
}

void draw_point(double lat, double lon, double radius){
    
    auto const x = longitude_to_cartesian(lon);
    auto const y = latitude_to_cartesian(lat);
    
    if(check_point_on_screen(x,y)){
        fillarc(x,y,radius,0,360);
    }
}

void draw_point(double lat, double lon, double radius, const t_color& color){
    
    setcolor(color);
    
    auto const x = longitude_to_cartesian(lon);
    auto const y = latitude_to_cartesian(lat);
        
    if(check_point_on_screen(x,y)){
        fillarc(x,y,radius,0,360);
    }
}

void draw_intersections(){
    
    for(auto iter = latLonIntersection.begin(); iter != latLonIntersection.end(); ++iter){
        draw_point(iter->lat(),iter->lon(),20,DARKGREY);
    }
}

void draw_pointsOfInterest(ElementType type){
        
    if(type == FOOD){
        for(unsigned i = 0; i < foodPOI.size(); ++i){
            if((i & 0x7) == 1 /*|| (i & 0x7) == 2*/){
                draw_point(latLonPointOfInterest[foodPOI[i]].lat(),latLonPointOfInterest[foodPOI[i]].lon(),20,RED);
            }
        }
    }
    else if(type == ENTERTAINMENT){
        for(unsigned i = 0; i < entertainmentPOI.size(); ++i){
            if((i & 0x3) == 1){
                draw_point(latLonPointOfInterest[entertainmentPOI[i]].lat(),latLonPointOfInterest[entertainmentPOI[i]].lon(),20,LIGHTSKYBLUE);
            }
        }
    }
    else if(type == RETAIL){
        for(unsigned i = 0; i < retailPOI.size(); ++i){
            draw_point(latLonPointOfInterest[retailPOI[i]].lat(),latLonPointOfInterest[retailPOI[i]].lon(),20,DARKKHAKI);
        }
    }
    else if(type == PUBLICSERVICE){
        for(unsigned i = 0; i < publicServicePOI.size(); ++i){
            if((i & 0x3) == 1){
                draw_point(latLonPointOfInterest[publicServicePOI[i]].lat(),latLonPointOfInterest[publicServicePOI[i]].lon(),20,LIMEGREEN);
            }
        }
    }
    else if(type == GOVERNMENT){
        for(unsigned i = 0; i < governmentPOI.size(); ++i){
            draw_point(latLonPointOfInterest[governmentPOI[i]].lat(),latLonPointOfInterest[governmentPOI[i]].lon(),20,SADDLEBROWN);
        }
    }
    else{
        for(unsigned i = 0; i < otherPOI.size(); ++i){
            draw_point(latLonPointOfInterest[otherPOI[i]].lat(),latLonPointOfInterest[otherPOI[i]].lon(),20,BLACK);
        }
    }
}

void draw_pointsOfInterest(){
    
    setcolor(RED);
    
    //food entertainment retail publicservice government
    
    for(unsigned i = 0; i < foodPOI.size(); ++i){
        if((i & 0x7) == 1 /*|| (i & 0x7) == 2*/){
            draw_test_point(latLonPointOfInterest[foodPOI[i]].lat(),latLonPointOfInterest[foodPOI[i]].lon(),icon,true);
            //draw_point(latLonPointOfInterest[foodPOI[i]].lat(),latLonPointOfInterest[foodPOI[i]].lon(),20,RED);
        }
    }
    for(unsigned i = 0; i < entertainmentPOI.size(); ++i){
        if((i & 0x3) == 1){
            draw_test_point(latLonPointOfInterest[foodPOI[i]].lat(),latLonPointOfInterest[foodPOI[i]].lon(),icon,true);
            //draw_point(latLonPointOfInterest[entertainmentPOI[i]].lat(),latLonPointOfInterest[entertainmentPOI[i]].lon(),20,LIGHTSKYBLUE);
        }
    }
    for(unsigned i = 0; i < retailPOI.size(); ++i){
        draw_test_point(latLonPointOfInterest[foodPOI[i]].lat(),latLonPointOfInterest[foodPOI[i]].lon(),icon,true);
        //draw_point(latLonPointOfInterest[retailPOI[i]].lat(),latLonPointOfInterest[retailPOI[i]].lon(),20,DARKKHAKI);
    }
    for(unsigned i = 0; i < publicServicePOI.size(); ++i){
        if((i & 0x3) == 1){
            draw_test_point(latLonPointOfInterest[foodPOI[i]].lat(),latLonPointOfInterest[foodPOI[i]].lon(),icon,true);
            //draw_point(latLonPointOfInterest[publicServicePOI[i]].lat(),latLonPointOfInterest[publicServicePOI[i]].lon(),20,LIMEGREEN);
        }
    }
    for(unsigned i = 0; i < governmentPOI.size(); ++i){
        draw_test_point(latLonPointOfInterest[foodPOI[i]].lat(),latLonPointOfInterest[foodPOI[i]].lon(),icon,true);
        //draw_point(latLonPointOfInterest[governmentPOI[i]].lat(),latLonPointOfInterest[governmentPOI[i]].lon(),20,SADDLEBROWN);
    }
}


//vector<pair<SearchReturnType,unsigned> search_in_radius(vector<string> keywords){
//    ;
//}

void draw_in_radius(double x, double y, double radius){
    
    // convert x,y to lat,lon for use with KD-Tree
    double lat = cartesian_to_lat(y);
    double lon = cartesian_to_lon(x);
    
    // prepare to draw
    set_drawing_buffer(OFF_SCREEN);
    
    // clear area before drawing
    setcolor(BACKGROUND_COLOR);
    fillarc(x,y,2*radius,0,360);
    
    // find closest intersections to position within specified radius
    vector<KDNode*> int_nodes;
    double scaling_factor = 50;
    int_nodes = intersectionMap.findNearestInRange(LatLon(lat,lon),scaling_factor*radius);
    scaling_factor = scaling_factor*2;
    cout << scaling_factor << endl;
    
    // if nothing found, continue until one is found;
    
    // debug msg
    cout << "Found nearby nodes in radius " << radius << ". Number of nodes is " << int_nodes.size() << "." << endl;
    
    setcolor(WARM_GRAY);
    
    // draw found intersections for debug
    for(unsigned i = 0; i < int_nodes.size(); i++){
        double x_pos = longitude_to_cartesian(int_nodes[i]->getCoords()[0]);
        double y_pos = latitude_to_cartesian(int_nodes[i]->getCoords()[1]);
        fillarc(x_pos,y_pos,0.0002,0,360);
    }
    
    t_color seg_color = PASTEL_YELLOW; 
    setlinewidth(5);
    
    // for every intersection within int_nodes
    for(unsigned i = 0; i < int_nodes.size(); i++){
        // get the id
        unsigned intersection_id = int_nodes[i]->getID();
        // find the ids of the street segs attached to this intersection
        vector<unsigned> street_seg_ids = searchByIntersection[intersection_id];
        // for every street segment, draw it
        for(unsigned j = 0; j < street_seg_ids.size(); j++){
            draw_street_segment(i,seg_color);
        }
    }
    
    copy_off_screen_buffer_to_screen();
}

double longitude_to_cartesian(double lon, double avg_lat){
    return (lon*DEG_TO_RAD * cos(avg_lat)*EARTH_RADIUS_IN_METERS);
}

double longitude_to_cartesian(double lon){
    return (lon*DEG_TO_RAD * cos(screen->get_global_properties().average_latitude)*EARTH_RADIUS_IN_METERS);
}

double latitude_to_cartesian(double lat){
    return lat*DEG_TO_RAD*EARTH_RADIUS_IN_METERS;
}
double cartesian_to_lon(double x, double avg_lat){
    return (x/(cos(avg_lat)*DEG_TO_RAD*EARTH_RADIUS_IN_METERS));
}

double cartesian_to_lon(double x){
    return (x/(cos(screen->get_global_properties().average_latitude)*DEG_TO_RAD*EARTH_RADIUS_IN_METERS));
}

double cartesian_to_lat(double y){
    return y/(EARTH_RADIUS_IN_METERS*DEG_TO_RAD);
}

//converts strings to uppercase to absolve case sensitive errors
string toUpperCase (string in){
	for (int i = 0; i < in.length(); i++){
		in[i] = toupper(in[i]);	
	}
	return in;
}

int levDistance(string query, string compare){
    // Levenshtein_distance, needs citation
    const int n = query.length();
    const int m = compare.length();
    if (n == 0) {
        return m;
    }
    if (m == 0) {
        return n;
    }
    vector< vector<int> > matrix(n+1);
    for (int i = 0; i <= n; i++) {
        matrix[i].resize(m+1);
    }

    for (int i = 0; i <= n; i++) {
        matrix[i][0]=i;
    }
    for (int j = 0; j <= m; j++) {
        matrix[0][j]=j;
    }
    for (int i = 1; i <= n; i++) {
        const char s_i = query[i-1];
        for (int j = 1; j <= m; j++) {
            const char t_j = compare[j-1];
            int cost;
            if (s_i == t_j) {
              cost = 0;
            }
            else {
              cost = 1;
            }
            const int above = matrix[i-1][j];
            const int left = matrix[i][j-1];
            const int diag = matrix[i-1][j-1];
            int cell = min( above + 1, min(left + 1, diag + cost));

            if (i>2 && j>2) {
              int trans=matrix[i-2][j-2]+1;
              if (query[i-2]!=t_j) trans++;
              if (s_i!= compare[j-2]) trans++;
              if (cell>trans) cell=trans;
            }
            matrix[i][j]=cell;
        }
    }
    return matrix[n][m];
}

int consecutive (string query, string compare){
    int count = 0;
    for (unsigned i = 0; i < min(query.length(),compare.length()); i++){
        if (query[i] != compare[i])
            return count;
        count ++;	
    }
    return count;
}
tuple<string,string,double,unsigned> autoComplete(string query, string filter){
  
    int highest = 0;
    char first = query[0];
    if (filter.size() >4)
        filter.erase(0,filter.size()-4);
    
    tuple<string,string,double,unsigned> autocomplete = make_tuple("", "N",0,0);
    
    if (filter.find('S') != string::npos){
 	for (unsigned i = 0; i < streetNames.size(); i ++){
            if (first == streetNames[i][0]){
		if (consecutive(toUpperCase(query),toUpperCase(streetNames[i]))>=highest){
                    autocomplete = make_tuple(streetNames[i], "S",1,i);
                    highest = consecutive(toUpperCase(query),toUpperCase(streetNames[i]));
                }
                if (highest < query.length())
                    autocomplete = make_tuple("", "S",0,0);
                if (toUpperCase(query) == toUpperCase(streetNames[i])){
                    return (make_tuple(streetNames[i],"S",1,i));
                }
            }
		
	}
    }
    if (filter.find('I') != string::npos){
        if (highest < query.size()){
            for(unsigned i = 0; i < numIntersections;i++){
                if (first == getIntersectionName(i)[0]){
                    if ( consecutive(toUpperCase(query),toUpperCase(getIntersectionName(i))) >=highest){
                        autocomplete = make_tuple(getIntersectionName(i), "I",1,i);
                        highest = consecutive(toUpperCase(query),toUpperCase(getIntersectionName(i)));
                    }
                    if (highest < query.length())
                        autocomplete = make_tuple("", "I",0,0);
                    if (toUpperCase(query) == toUpperCase(getIntersectionName(i))){
                        return (make_tuple(getIntersectionName(i),"I",1,i));
                    }
                }
            }
        }
    }
    
    if (filter.find('F') != string::npos){
        if (highest < query.size()){

            for(unsigned i = 0; i < numFeatures;i++){
                if (first == getFeatureName(i)[0]){
                    if (consecutive(toUpperCase(query),toUpperCase(getFeatureName(i)) )>=highest){
                        autocomplete = make_tuple(getFeatureName(i), "F",1,i);
                        highest = consecutive(toUpperCase(query),toUpperCase(getFeatureName(i)));
                    }
                    if (highest < query.length())
                        autocomplete = make_tuple("", "F",0,0);
                    if (toUpperCase(query) == toUpperCase(getFeatureName(i))){
                        return (make_tuple(getFeatureName(i),"F",1,i));
                    }
                }
            }

        } 
        }
        if (filter.find('P') != string::npos){
        if (highest < query.size()){
            for(unsigned i = 0; i < numPOI;i++){
                if (first == getPointOfInterestName(i)[0]){
                    if (consecutive(toUpperCase(query),toUpperCase(getPointOfInterestName(i)))>=highest){
                        autocomplete = make_tuple(getPointOfInterestName(i), "P",1,i);
                        highest = consecutive(toUpperCase(query),toUpperCase(getPointOfInterestName(i)));
                    }
                    if (highest < query.length())
                        autocomplete = make_tuple("", "P",0,0);
                    if (toUpperCase(query) == toUpperCase(getPointOfInterestName(i))){
                        return (make_tuple(getPointOfInterestName(i),"P",1,i));
                    }
                }

            }
        }
    }
    return autocomplete;
}

//Vector of suggested texts, inclusive of perfect matches
vector< tuple<string,string, double,unsigned> > textSuggest(string query, string filter, int maxNum){
    
	vector< tuple<string,string, double,unsigned> > suggestions;
	if (filter.size()>4)
            filter=filter.substr(4);
        
        //Finds all similar Strings
        if (filter.find('S') != string::npos){
            cout << "Streets Loaded" << endl;
            for (unsigned i = 0; i < streetNames.size(); i ++){
                    if(toupper(query[0]) == toupper((streetNames[i])[0])) {
                    //weights the consecutive letters that are the same in the beginning
                    //also does a pseudo% calculcation for similarity
                    int levDis = levDistance(toUpperCase(query),toUpperCase(streetNames[i]));		
                    int larger = max(query.length(), (streetNames[i]).length());
                    double percentage = (larger-levDis)/(double)larger;
                    percentage = percentage * percentage;
                    int inARow = consecutive(toUpperCase(query),toUpperCase(streetNames[i]));
                    if ( inARow != 0 ){
                        percentage *=1.1 + inARow*0.5;
                    }

                    suggestions.push_back(make_tuple(streetNames[i], "S", percentage,i));
                }
            }
        }
        if (filter.find('I') != string::npos){
             cout << "Inters Loaded" << endl;
            for(unsigned i = 0; i < numIntersections;i++){
                if(toupper(query[0]) == toupper(getIntersectionName(i)[0])) {
                    int levDis = levDistance(toUpperCase(query),toUpperCase(getIntersectionName(i)));		
                    int larger = max(query.length(), getIntersectionName(i).length());
                    double percentage = (larger-levDis)/(double)larger;
                    percentage = percentage * percentage;
                    int inARow = consecutive(toUpperCase(query),toUpperCase(getIntersectionName(i)));
                    if ( inARow != 0 ){
                        percentage *=1.1 + inARow*0.5;

                    }

                    suggestions.push_back(make_tuple(getIntersectionName(i), "I", percentage,i));
                }
            }
        }
        if (filter.find('F') != string::npos){
             cout << "Features Loaded" << endl;
            for(unsigned i = 0; i < numFeatures;i++){
                if(toupper(query[0]) == toupper(getFeatureName(i)[0])) {
                    int levDis = levDistance(toUpperCase(query),toUpperCase(getFeatureName(i)));		
                    int larger = max(query.length(), getFeatureName(i).length());
                    double percentage = (larger-levDis)/(double)larger;
                    percentage = percentage * percentage;
                    int inARow = consecutive(toUpperCase(query),toUpperCase(getFeatureName(i)));
                    if ( inARow != 0 ){
                        percentage *=1.1 + inARow*0.5;

                    }

                    suggestions.push_back(make_tuple(getFeatureName(i), "F", percentage,i));
                }
            }
        }
        
        if (filter.find('P') != string::npos){
             cout << "POIS Loaded" << endl;
            for(unsigned i = 0; i < numPOI;i++){
                if(toupper(query[0]) == toupper(getPointOfInterestName(i)[0])) { 
                int levDis = levDistance(toUpperCase(query),toUpperCase(getPointOfInterestName(i)));		
                    int larger = max(query.length(), getPointOfInterestName(i).length());
                    double percentage = (larger-levDis)/(double)larger;
                    percentage = percentage * percentage;
                    int inARow = consecutive(toUpperCase(query),toUpperCase(getPointOfInterestName(i)));
                    if ( inARow != 0 ){
                        percentage *=1.1 + inARow*0.5;

                    }

                    suggestions.push_back(make_tuple(getPointOfInterestName(i), "P", percentage,i));
                }
            }
        }
        
      if (!suggestions.empty()){
                //cout << "empty vector, segfaults at sort" <<endl;//sort by percentages, gives top 5 or size of vector
              sort(suggestions.begin(), suggestions.end(),[](tuple<string, string, double,unsigned> const &t1, tuple<string, string, double,unsigned> const &t2) {
              return get<2>(t1) > get<2>(t2); // comparator
          });
          suggestions.erase(unique(suggestions.begin(), suggestions.end(),[](tuple<string, string, double,unsigned> const &t1, tuple<string, string, double,unsigned> const &t2) {
              return (get<0>(t1) == get<0>(t2) && get<1>(t1) == get<1>(t2)); // comparator
          }),suggestions.end());
              //returns top 5 results
          suggestions.erase(suggestions.begin() + (suggestions.size() < maxNum ? (suggestions.size()-1):maxNum) , suggestions.end());
          //cout << "or not " << endl;
      }
  return suggestions;
}
