#include "m1.h"
#include "m3.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "../inc/m1_structures.h"
#include "../inc/KDMap.h"
#include "../inc/OSMStructures.h"
#include "../inc/path_finding_struct.h"
#include <vector>
#include <map>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <utility>
#include <iomanip>
#include <string>
#include <sstream>
#include <cstdlib>
#include <ctime>

/*
 * Michael's Notes:
 * -Update draw streets to include _links, draw at lower zoom levels
 * -Layer one does not provide any coordinates, can only pull more info for
 *  streets available from layer 2
 * -Look into if there are other useful properties ie railways?
 * 
 */

using namespace std;

unsigned numStreetSegs;
unsigned numIntersections;
unsigned numStreet;
unsigned numFeatures;
unsigned numPOI;

vector <StreetSegmentInfo> searchByStreetSeg;
vector <vector<unsigned>> searchByIntersection;
vector <vector<LatLon>> latLonStreetSeg;
vector <LatLon> latLonIntersection;
vector <LatLon> latLonPointOfInterest;
vector <string> streetNames;
vector <vector<unsigned>>  searchStreetSegByStreetID;
vector <double> streetSegTravelTime;
unordered_multimap <string, unsigned> searchByStreetName;
KDMap intersectionMap;
KDMap pointsOfInterestMap;
// an array of vectors written in terms of street_ids contain array of vector that contains all of the intersection in order on the street
vector <vector<unsigned>> searchIntersectionsByStreetID;
vector <vector<unsigned>> searchStreetSegByStreetID_inorder;

vector <FeatureData> FeaturesVector;
vector <unsigned> featuresGreenery;
vector <unsigned> featuresWater;
vector <unsigned> featuresSand;
vector <unsigned> featuresBuilding;
vector <unsigned> featuresOther;

unordered_map <OSMID,const OSMWay*> searchStreetByOSMID;
vector<unsigned> motorwayStreets;
vector<unsigned> truckStreets;
vector<unsigned> primaryStreets;
vector<unsigned> secondaryStreets;
vector<unsigned> tertiaryStreets;
vector<unsigned> regularStreets;
vector<unsigned> smallStreets;
vector<string> streetSegTags;
vector<bool> streetLoop;

unordered_map <OSMID,const OSMNode*> searchPOIByOSMID;
vector<unsigned> foodPOI;
vector<unsigned> educationPOI;
vector<unsigned> transportationPOI;
vector<unsigned> financialPOI;
vector<unsigned> healthcarePOI;
vector<unsigned> entertainmentPOI;
vector<unsigned> retailPOI;
vector<unsigned> publicServicePOI;
vector<unsigned> governmentPOI;
vector<unsigned> religiousPOI;
vector<unsigned> otherPOI;
vector<string> POITags;
unordered_multimap <string,unsigned> POINames;

bool load_map(std::string map_path) {
    
    //Load your map related data structures here
    bool success = loadStreetsDatabaseBIN(map_path);
    if(!success)
        return false;
    
    numStreetSegs = getNumberOfStreetSegments();
    numIntersections = getNumberOfIntersections();
    numStreet = getNumberOfStreets();
    numPOI = getNumberOfPointsOfInterest();
    numFeatures = getNumberOfFeatures();
    
    FeaturesVector.clear();
    
    vector <StreetSegmentInfo> searchByStreetSeg1;
    vector <vector<unsigned>> searchByIntersection1(numIntersections);
    vector <vector<LatLon>> latLonStreetSeg1(numStreetSegs);
    vector <LatLon> latLonIntersection1;
    vector <string> streetNames1;
    vector <vector<unsigned>> searchStreetSegByStreetID1(numStreet);
    vector <double> streetSegTravelTime1;
    unordered_multimap <string, unsigned> searchByStreetName1;
    vector <vector<unsigned>> searchIntersectionsByStreetID1(numStreet);
    vector <vector<unsigned>>  searchStreetSegByStreetID_inorder1(numStreet);

    for (unsigned i=0; i < numFeatures; i++){
        vector<LatLon> pts;
        for (unsigned j = 0; j < getFeaturePointCount(i); j++){
            pts.push_back(getFeaturePoint(i,j));
        }
        FeatureData newFeatureData(getFeatureName(i), getFeatureType(i), getFeatureOSMID(i),getFeaturePointCount(i) , pts );
        FeaturesVector.push_back(newFeatureData);
        
        //place Feature's index in corresponding vector
//        FeatureType type = getFeatureType(i);
//        
//        if(type == Greenspace || type == Golfcourse || type == Island){
//            featuresGreenery.push_back(i);
//        }
//        else if(type == Lake || type == River || type == Stream){
//            featuresWater.push_back(i);
//        }
//        else if(type == Beach || type == Shoreline){
//            featuresSand.push_back(i);
//        }
//        else if(type == Building){
//            featuresBuilding.push_back(i);
//        }
//        else{ //other
//            featuresOther.push_back(i);
//        }
        
        pts.clear();
    }
    
    //populate latLonStreetSeg and searchByStreetSeg
    for(unsigned i = 0; i < numStreetSegs;i++){
        for(unsigned j = 0; j < getStreetSegmentInfo(i).curvePointCount; j++){
            latLonStreetSeg1[i].push_back(getStreetSegmentCurvePoint(i, j));
        }
        searchByStreetSeg1.push_back(getStreetSegmentInfo(i));
    }
    latLonStreetSeg = latLonStreetSeg1;
    searchByStreetSeg = searchByStreetSeg1;
    
    //populate latLonPointOfInterest and kd map for POI
    for(unsigned i = 0; i < numPOI;i++){
        latLonPointOfInterest.push_back(getPointOfInterestPosition(i));
        pointsOfInterestMap.insertNode(getPointOfInterestPosition(i),i);
    }

    //populate searchByIntersection and latLonIntersection and intersectionMap
    for(unsigned i = 0; i < numIntersections;i++){
        for(unsigned j = 0; j < getIntersectionStreetSegmentCount(i); j++){
	  searchByIntersection1[i].push_back(getIntersectionStreetSegment(i, j));
        }
        latLonIntersection1.push_back(getIntersectionPosition(i));
	intersectionMap.insertNode(getIntersectionPosition(i),i);
    }
    searchByIntersection = searchByIntersection1;
    latLonIntersection = latLonIntersection1;

    //populate streetNames as well as multimap searchByStreetName
    for(unsigned i = 0; i< numStreet; i++){
        streetNames1.push_back(getStreetName(i));
	searchByStreetName1.insert(make_pair(getStreetName(i),i));
    }
    streetNames = streetNames1;
    searchByStreetName=searchByStreetName1;

    //populate searchStreetSegByStreetID and streetSegTravelTime
    for(unsigned i = 0; i < numStreetSegs; i++){
        searchStreetSegByStreetID1[searchByStreetSeg1[i].streetID].push_back(i);
        streetSegTravelTime1.push_back(find_street_segment_length(i)*3.6/searchByStreetSeg[i].speedLimit);
    }
    searchStreetSegByStreetID = searchStreetSegByStreetID1;
    streetSegTravelTime = streetSegTravelTime1;
    
    for(unsigned i = 0; i < streetNames.size(); i++){
        if(streetNames[i]!="<unknown>"){
            vector <unsigned> temp = find_all_street_intersections(i);
            vector <unsigned> streetSeg = find_street_street_segments(i);
            vector <unsigned> streetSegStore;
            vector <unsigned> intersections;

            unsigned current = streetSeg.front();
            streetSegStore.push_back(current);
            unsigned from = current;
            unsigned to = current;

            for(unsigned j = 0;j<streetSeg.size();j++){

                for(auto itr = streetSeg.begin();itr<streetSeg.end();itr++){
                    if(searchByStreetSeg[from].to == searchByStreetSeg[*itr].from){
                        from = *itr;
                        streetSegStore.push_back(from);
                    }
                }

                for(auto itr = streetSeg.begin();itr<streetSeg.end();itr++){
                    if(searchByStreetSeg[to].from == searchByStreetSeg[*itr].to){
                        to = *itr;
                        streetSegStore.insert(streetSegStore.begin(),to);
                    }
                }
            }
            
            intersections.push_back(searchByStreetSeg[streetSegStore.front()].from);
            for(auto itr = streetSegStore.begin();itr<streetSegStore.end();itr++){
                if(searchByStreetSeg[*itr].to != searchByStreetSeg[*itr].from){
                    intersections.push_back(searchByStreetSeg[*itr].to); 
                }
            }

            searchIntersectionsByStreetID1[i] = intersections;
            searchStreetSegByStreetID_inorder1[i] = streetSegStore;
            
        }        
    }
    searchIntersectionsByStreetID = searchIntersectionsByStreetID1;
    searchStreetSegByStreetID_inorder = searchStreetSegByStreetID_inorder1;
    
    populatePathFind();
    
    // layer-1 stuff
    string map_path_osm;
    
    vector<string> tokens;
    
    stringstream ss(map_path);
    string item;
    char delim = '.';
    
    while(getline(ss,item,delim)){
        tokens.push_back(item);
    }
    
    map_path_osm = tokens.front() + ".osm.bin";
    
    loadOSMDatabaseBIN(map_path_osm);
        
    loadOSMData();
    
    return true;
}

void close_map() {

    //Clean-up your map related data structures here
    closeStreetDatabase();
    closeOSMDatabase();

}


//Returns street id(s) for the given street name
//If no street with this name exists, returns a 0-length vector.
std::vector<unsigned> find_street_ids_from_name(std::string street_name){
    vector<unsigned> found_sid;
    
    //range is a pair of iterators of type multimap<string,unsigned>
    //find range of values with same key
    auto range = searchByStreetName.equal_range(street_name);
    for(auto sid_iter = range.first; sid_iter != range.second; sid_iter++){
        found_sid.push_back(sid_iter->second);
    }
    
    return found_sid;
}

//Returns the street segments for the given intersection
//return a NULL if can not find street segment
std::vector<unsigned> find_intersection_street_segments(unsigned intersection_id){

    //check bound to make more the street segment exists
    if(intersection_id >= numIntersections){
       	vector<unsigned> fail;
        return fail;
    }

    return searchByIntersection[intersection_id];
}

//Returns the street names at the given intersection (includes duplicate street names in returned vector)
//return a NULL if can not find street segment
std::vector<std::string> find_intersection_street_names(unsigned intersection_id){
    vector<string> found_sn;

    unsigned index = 0;
    string name;
    for(auto ssid_iter = searchByIntersection[intersection_id].begin();
        ssid_iter != searchByIntersection[intersection_id].end(); ssid_iter++){
        name = streetNames[searchByStreetSeg[searchByIntersection[intersection_id][index]].streetID];
        found_sn.push_back(name);
        index++;
    }

    return found_sn;
}

//Returns true if you can get from intersection1 to intersection2 using a single street segment (hint: check for 1-way streets too)
//the function is written under the assumption that there would not be a pair of street segments that begin and end at the same points
//return a false if can not find street segment
bool are_directly_connected(unsigned intersection_id1, unsigned intersection_id2){

    if(intersection_id1 == intersection_id2){
        return true;
    }
    
    unsigned index = 0;
    for(auto ssid_iter = searchByIntersection[intersection_id1].begin();
        ssid_iter != searchByIntersection[intersection_id1].end(); ssid_iter++){
            if((searchByStreetSeg[searchByIntersection[intersection_id1][index]].from == intersection_id1 &&
                searchByStreetSeg[searchByIntersection[intersection_id1][index]].to == intersection_id2) ||
               (searchByStreetSeg[searchByIntersection[intersection_id1][index]].to == intersection_id1 &&
                searchByStreetSeg[searchByIntersection[intersection_id1][index]].from == intersection_id2 &&
                searchByStreetSeg[searchByIntersection[intersection_id1][index]].oneWay == false)){
                return true;
            }
            index++;
    }

    return false;
}

//Returns all intersections reachable by traveling down one street segment
//from given intersection (hint: you can't travel the wrong way on a 1-way street)
//the returned vector should NOT contain duplicate intersections
vector<unsigned> find_adjacent_intersections(unsigned intersection_id){
    vector <unsigned> found_iid;

    unsigned index = 0;
    //again, consider using iterators
    for(auto ssid_iter = searchByIntersection[intersection_id].begin();
        ssid_iter != searchByIntersection[intersection_id].end(); ssid_iter++){ //loop through vector of connected street segs
        //check if the street is a one way street
        if(searchByStreetSeg[searchByIntersection[intersection_id][index]].oneWay == true){ //if true check direction is correct
            if(searchByStreetSeg[searchByIntersection[intersection_id][index]].from == intersection_id && intersection_id != searchByStreetSeg[searchByIntersection[intersection_id][index]].to){
                found_iid.push_back(searchByStreetSeg[searchByIntersection[intersection_id][index]].to);
            }
        }
        //not one way street
        //take the other id and place into vector
        else if(searchByStreetSeg[searchByIntersection[intersection_id][index]].from == intersection_id && intersection_id != searchByStreetSeg[searchByIntersection[intersection_id][index]].to){
            found_iid.push_back(searchByStreetSeg[searchByIntersection[intersection_id][index]].to); //push .to id
        }
        else if(intersection_id != searchByStreetSeg[searchByIntersection[intersection_id][index]].from){
            found_iid.push_back(searchByStreetSeg[searchByIntersection[intersection_id][index]].from); //push .from id
        }
        index++;
    }//the function is written under the assumption that there would not be a pair of intersections that begin and end at the same points

    sort(found_iid.begin(), found_iid.end()); //sort vector
    found_iid.erase(unique(found_iid.begin(), found_iid.end()),found_iid.end()); //delete repeated values
    
    return found_iid;
}

//Returns all street segments for the given street
//return a NULL if can not find street_id
vector<unsigned> find_street_street_segments(unsigned street_id){
    
    if(street_id >= numStreet){
       	vector<unsigned> fail;
        return fail;
    }
 
    return searchStreetSegByStreetID[street_id];
}

//Returns all intersections(in vector) along the a given street
vector<unsigned> find_all_street_intersections(unsigned street_id){
    
    //going to implement this here but might be better to move it into load_map()
    vector<unsigned> found_iid;
    vector<unsigned> found_ssid = find_street_street_segments(street_id); //returns vector<unsigned> of street_ids, unordered
    for(auto ssid_iter = found_ssid.begin(); ssid_iter != found_ssid.end(); ssid_iter++){
        found_iid.push_back(searchByStreetSeg[*ssid_iter].from);
        found_iid.push_back(searchByStreetSeg[*ssid_iter].to);
    }
    //finished populating, full of repeats
    sort(found_iid.begin(), found_iid.end()); //sort vector
    found_iid.erase(unique(found_iid.begin(), found_iid.end()),found_iid.end()); //delete repeated values
    
    
    return found_iid;
}

//Return all intersection ids for two intersecting streets
//This function will typically return one intersection id.
//However street names are not guaranteed to be unique, so more than 1 intersection id may exist
vector<unsigned> find_intersection_ids_from_street_names(std::string street_name1, std::string street_name2){
    vector <unsigned> street_intersection1;
    vector <unsigned> street_id1 = find_street_ids_from_name(street_name1);

    for(auto sIter1 = street_id1.begin(); sIter1 != street_id1.end(); sIter1++){
        vector <unsigned> temp_street_int = find_all_street_intersections(*sIter1);
        street_intersection1.insert(street_intersection1.end(), temp_street_int.begin(), temp_street_int.end());
    }
    
    vector <unsigned> street_intersection2;
    vector <unsigned> street_id2 = find_street_ids_from_name(street_name2);

    for(auto sIter2 = street_id2.begin(); sIter2 != street_id2.end(); sIter2++){
        vector <unsigned> temp_street_int = find_all_street_intersections(*sIter2);
        street_intersection2.insert(street_intersection2.end(), temp_street_int.begin(), temp_street_int.end());
    }
    
    vector <unsigned> intersection_ids;
    for(auto iter1 = street_intersection1.begin();iter1!= street_intersection1.end();iter1++){
        for(auto iter2 = street_intersection2.begin();iter2!= street_intersection2.end();iter2++){
            if(*iter1 == *iter2){
                intersection_ids.push_back(*iter1);
            }
        }
    }
    
    return intersection_ids;
}


//Returns the distance between two coordinates in meters
double find_distance_between_two_points(LatLon point1, LatLon point2){
    
    //load in latitude and longitudes of both points
    double lat1 = point1.lat()*DEG_TO_RAD;
    double lon1 = point1.lon()*DEG_TO_RAD;
    double lat2 = point2.lat()*DEG_TO_RAD;
    double lon2 = point2.lon()*DEG_TO_RAD;
    
    //convert lat/long to x,y positions
    double lonFactor = cos((lat1+lat2)/2);
    double x1 = lon1*lonFactor;
    double y1 = lat1;
    double x2 = lon2*lonFactor;
    double y2 = lat2;
    
    //calculate distance
    double magnitudex = (x2-x1)*(x2-x1);
    double magnitudey = (y2-y1)*(y2-y1);
    double d = EARTH_RADIUS_IN_METERS*sqrt(magnitudex+magnitudey);
    
    return d;
}

//Returns the length of the given street segment in meters
double find_street_segment_length(unsigned street_segment_id){
    
    double totalDistance = 0;
    unsigned numCurvedPoints = searchByStreetSeg[street_segment_id].curvePointCount;
    if(numCurvedPoints == 0){
        totalDistance = find_distance_between_two_points(latLonIntersection[searchByStreetSeg[street_segment_id].from],
                                                latLonIntersection[searchByStreetSeg[street_segment_id].to]);
    }
    //initialize total distance value
    else{    
        totalDistance = 0;
        totalDistance += find_distance_between_two_points(latLonIntersection[searchByStreetSeg[street_segment_id].from],latLonStreetSeg[street_segment_id][0]);
        for(unsigned i = 1; i < numCurvedPoints; i++){
            totalDistance += find_distance_between_two_points(latLonStreetSeg[street_segment_id][i-1],latLonStreetSeg[street_segment_id][i]);
        }
        totalDistance += find_distance_between_two_points(latLonStreetSeg[street_segment_id][numCurvedPoints-1],latLonIntersection[searchByStreetSeg[street_segment_id].to]);
    }
    
    //return streetSegLengthByStreetSeg[street_segment_id];
    return totalDistance;
}

//Returns the length of the specified street in meters
double find_street_length(unsigned street_id){
    
    double totalDistance = 0;
    
    for(auto ssid_iter = searchStreetSegByStreetID[street_id].begin();
        ssid_iter != searchStreetSegByStreetID[street_id].end(); ssid_iter++){
        totalDistance += find_street_segment_length(*ssid_iter);
    }
    
    return totalDistance;
}

//Returns the travel time to drive aPOITa street segment in seconds (time = distance/speed_limit)
double find_street_segment_travel_time(unsigned street_segment_id){
    
    return streetSegTravelTime[street_segment_id];
}

//Returns the nearest point of interest to the given position
unsigned find_closest_point_of_interest(LatLon my_position){
    
    KDNode* ans =  pointsOfInterestMap.findNearestPoint(my_position);

    if(ans!=NULL)
        return ans->getID();
    else
        return 0;
}

//Returns the the nearest intersection to the given position
unsigned find_closest_intersection(LatLon my_position){

    KDNode* ans =  intersectionMap.findNearestPoint(my_position);
  
    if(ans!=NULL)
        return ans->getID();
    else
        return 0;
}

void testOSM(){
    
    //srand(time(NULL));
    
//    cout << "Number of nodes: " << getNumberOfNodes() << endl;
//    cout << "Number of POI: " << numPOI << endl;
//    
//    for(unsigned j = 0; j < 5000; j++){
//        const OSMNode* ptr = searchPOIByOSMID.find(getPointOfInterestOSMNodeID(j))->second;
//        cout << "test" << endl;
//        unsigned count = getTagCount(ptr);
//        for(unsigned i = 0; i < count; i++){libstreetmap/src/m1.cpp:604:26: error: request for member 'insert' in 'POINames', which is of non-class type 'int'
//            string key, value;
//            tie(key,value) = getTagPair(ptr,i);
//            cout << "Key: " << key << endl;
//            cout << "Value: " << value << endl;
//        }
//        cout << endl;
//    }
    
//    for(unsigned j = 0; j < 10; j++){
//        const OSMWay* ptr = getWayByIndex(rand()%379588);
//        unsigned count = getTagCount(ptr);
//        for(unsigned i = 0; i < count; i++){
//            string key, value;
//            tie(key,value) = getTagPair(ptr,i);
//            cout << "Key: " << key << endl;
//            cout << "Value: " << value << endl;
//        }
//        cout << endl;
//    }
}

void loadOSMData(){
    populatePOIByOSMID();
    populatePOITags();
    populateStreetByOSMID();
    populateStreetTags();
}

void populatePOIByOSMID(){
    unsigned numNodes = getNumberOfNodes();
    for(unsigned i = 0; i < numNodes; i++){
        const OSMNode* ptr = getNodeByIndex(i);
        searchPOIByOSMID.insert(make_pair(ptr->id(),ptr));
    }
}
//POITa

void populatePOITags(){
    
    for(unsigned i = 0; i < numPOI; i++){
        const OSMNode* morePOIData = searchPOIByOSMID.find(getPointOfInterestOSMNodeID(i))->second;
        
        unsigned count = getTagCount(morePOIData);
        for(unsigned j = 0; j < count; j++){
            string key,value;
            tie(key,value) = getTagPair(morePOIData,j);
            
            if(key == "name"){
                POINames.insert(make_pair(value,i));
            }
            
            if(key == "amenity" || key == "shop" || key == "public_transport"){
                if(value == "bar" || value == "biergarten" || value == "cafe" ||
                   value == "fast_food" || value == "food_court" || value == "ice_cream" ||
                   value == "pub" || value == "restaurant"){
                    foodPOI.push_back(i);
                    //POITags.push_back(value);
                }
                else if(value == "college" || value == "kindergarten" || value == "school" ||
                        value == "music_school" || value == "driving_school" ||
                        value == "language_school" || value == "university"){
                    educationPOI.push_back(i);
                    //POITags.push_back(value);
                }
                else if(value == "bicycle_rental" || value == "car_rental" ||
                        value == "charging_station" || value == "ferry_terminal" ||
                        value == "fuel" || value == "taxi" || value == "station"){
                    transportationPOI.push_back(i);
                    //POITags.push_back(value);
                }
                else if(value == "atm" || value == "bank"){
                    financialPOI.push_back(i);
                    //POITags.push_back(value);
                }
                else if(value == "clinic" || value == "dentist" || value == "doctors" ||
                        value == "hospital" || value == "nursing_home" || value == "pharmacy" ||
                        value == "veterinary"){
                    healthcarePOI.push_back(i);
                    //POITags.push_back(value);
                }
                else if(value == "arts_center" || value == "casino" || value == "cinema" ||
                        value == "gambling" || value == "nightclub" || value == "planetarium" ||
                        value == "studio" || value == "theatre"){
                    entertainmentPOI.push_back(i);
                    //POITags.push_back(value);
                }
                else if(key == "shop" || value == "marketplace" || value == "internet_cafe"){
                    retailPOI.push_back(i);
                    //POITags.push_back(value);
                }
                else if(value == "courthouse" || value == "fire_station" || 
                        value == "post_office" || value == "prison" || value == "police" ||
                        value == "library"){
                    publicServicePOI.push_back(i);
                    //POITags.push_back(value);
                }
                else if(value == "embassy" || value == "town_hall"){
                    governmentPOI.push_back(i);
                    //POITags.push_back(value);
                }
                else if(value == "place_of_worship"){
                    religiousPOI.push_back(i);
                    //POITags.push_back(value);
                }
                else{
                    otherPOI.push_back(i);
                    //POITags.push_back(value);
                }
            }
        }
    }
}


void populateStreetByOSMID(){
    unsigned numWay = getNumberOfWays();
    for(unsigned i = 0;i<numWay;i++){
        const OSMWay* ptr = getWayByIndex(i);
        searchStreetByOSMID.insert(make_pair(ptr->id(),ptr));
    }
}

void populateStreetTags(){
    for(unsigned i = 0;i<numStreetSegs;i++){
        const OSMWay* moreStData = searchStreetByOSMID.find(searchByStreetSeg[i].wayOSMID)->second;
        unsigned count = getTagCount(moreStData);
        for(unsigned j = 0;j<count;j++){
            string key,value;
            tie(key,value) = getTagPair(moreStData,j);
            if(key == "highway"){
                if(value == "motorway" /*|| value == "motorway_link"*/){
                    streetSegTags.push_back("motorway");
                    motorwayStreets.push_back(i);
                }else if(value == "truck" /*|| value == "truck_link"*/){
                    streetSegTags.push_back("truck");
                    truckStreets.push_back(i);
                }else if(value == "primary" /*|| value == "primary_link"*/){
                    streetSegTags.push_back("primary");
                    primaryStreets.push_back(i);
                }else if(value == "secondary" /*|| value == "secondary_link"*/){
                    streetSegTags.push_back("secondary");
                    secondaryStreets.push_back(i);
                }else if(value == "tertiary" /*|| value == "tertiary_link"*/){
                    streetSegTags.push_back("tertiary");
                    tertiaryStreets.push_back(i);
                }else if(value == "unclassified" /*||value == "residential"*/){
                    streetSegTags.push_back("regular");
                    regularStreets.push_back(i);
                }else if(value == "service"){
                    streetSegTags.push_back("small");
                    smallStreets.push_back(i);
                }else{
                    streetSegTags.push_back("regular");
                    regularStreets.push_back(i);
                }
            }
        }
    }
}
