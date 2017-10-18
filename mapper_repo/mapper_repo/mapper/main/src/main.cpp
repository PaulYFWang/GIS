#include <iostream>
#include <string>
#include <tuple>
#include <cmath>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xos.h>
#include <X11/Xatom.h>

#include "m1.h"
#include "m2.h"

#include "StreetsDatabaseAPI.h"

using namespace std;

int main(int argc, char** argv) {

    std::string map_path_toronto;
    std::string map_path_beijing;
    std::string map_path_cairo;
    std::string map_path_capetown;
    std::string map_path_goldenhorseshoe;
    std::string map_path_hamilton;
    std::string map_path_hongkong;
    std::string map_path_iceland;
    std::string map_path_london;
    std::string map_path_moscow;
    std::string map_path_newdelhi;
    std::string map_path_newyork;
    std::string map_path_riodejaneiro;
    std::string map_path_sainthelena;
    std::string map_path_singapore;
    std::string map_path_sydney;
    std::string map_path_tehran;
    std::string map_path_tokyo;
    std::string map_path;
    
    if(argc == 1){
        //Use a default map
        map_path_toronto = "/cad2/ece297s/public/maps/toronto_canada.streets.bin";
        map_path_beijing = "/cad2/ece297s/public/maps/beijing_china.streets.bin";
        map_path_cairo = "/cad2/ece297s/public/maps/cairo_egypt.streets.bin";
        map_path_capetown = "/cad2/ece297s/public/maps/cape-town_south-africa.streets.bin";
        map_path_goldenhorseshoe = "/cad2/ece297s/public/maps/golden-horseshoe_canada.streets.bin";
        map_path_hamilton = "/cad2/ece297s/public/maps/hamilton_canada.streets.bin";
        map_path_hongkong = "/cad2/ece297s/public/maps/hong-kong_china.streets.bin";
        map_path_iceland = "/cad2/ece297s/public/maps/iceland.streets.bin";
        map_path_london = "/cad2/ece297s/public/maps/london_england.streets.bin";
        map_path_moscow = "/cad2/ece297s/public/maps/moscow_russia.streets.bin";
        map_path_newdelhi = "/cad2/ece297s/public/maps/new-delhi_india.streets.bin";
        map_path_newyork = "/cad2/ece297s/public/maps/new-york_usa.streets.bin";
        map_path_riodejaneiro = "/cad2/ece297s/public/maps/rio-de-janeiro_brazil.streets.bin";
        map_path_sainthelena = "/cad2/ece297s/public/maps/saint-helena.streets.bin";
        map_path_singapore = "/cad2/ece297s/public/maps/singapore.streets.bin";
        map_path_sydney = "/cad2/ece297s/public/maps/sydney_australia.streets.bin";
        map_path_tehran = "/cad2/ece297s/public/maps/tahran_iran.streets.bin";
        map_path_tokyo = "/cad2/ece297s/public/maps/tokyo_japan.streets.bin";
        
        string userinput;
        cout<<"City Please:"<<endl;
        cin>>userinput;
        
        if(userinput == "toronto"){
            map_path = map_path_toronto;
        }else if(userinput == "beijing"){
            map_path = map_path_beijing;
        }else if(userinput == "cairo"){
            map_path = map_path_cairo;
        }else if(userinput == "capetown"){
            map_path = map_path_capetown;
        }else if(userinput == "goldenhorseshoe"){
            map_path = map_path_goldenhorseshoe;
        }else if(userinput == "hamilton"){
            map_path = map_path_hamilton;
        }else if(userinput == "hongkong"){
            map_path = map_path_hongkong;
        }else if(userinput == "iceland"){
            map_path = map_path_iceland;
        }else if(userinput == "london"){
            map_path = map_path_london;
        }else if(userinput == "moscow"){
            map_path = map_path_moscow;
        }else if(userinput == "newdelhi"){
            map_path = map_path_newdelhi;
        }else if(userinput == "newyork"){
            map_path = map_path_newyork;
        }else if(userinput == "riodejaneiro"){
            map_path = map_path_riodejaneiro;
        }else if(userinput == "sainthelena"){
            map_path = map_path_sainthelena;
        }else if(userinput == "singapore"){
            map_path = map_path_singapore;
        }else if(userinput == "sydney"){
            map_path = map_path_sydney;
        }else if(userinput == "tehran"){
            map_path = map_path_tehran;
        }else if(userinput == "tokyo"){
            map_path = map_path_tokyo;
        }else{
            map_path = map_path_toronto;
        }
        
    } else if (argc == 2) {
        //Get the map from the command line
        map_path = argv[1];
    } else {
        //Invalid arguments
        std::cerr << "Usage: " << argv[0] << " [map_file_path]\n";
        std::cerr << "  If no map_file_path is provided a default map is loaded.\n";
        return 1;
    }
        
    //Load the map and related data structures
    bool load_success = load_map(map_path);
    if(!load_success) {
        std::cerr << "Failed to load map '" << map_path << "'\n";
        return 2;
    }
	string searchy = "vctora prk ";

            
    std::cout << "Successfully loaded map '" << map_path << "'\n";
    //You can now do something with the map
    draw_map();
    
    //Clean-up the map related data structures
    std::cout << "Closing map\n";
    close_map();

    return 0;
    
}