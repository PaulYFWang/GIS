#pragma once

#include <vector>
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "m1.h"
#include "graphics.h"
#include "../inc/m1_structures.h"
#include "../inc/KDMap.h"

using namespace std;

class LatLon;

//enum POITypes{
//    FOOD = 0,
//    EDUCATION,
//    TRANSPORTATION,
//    FINANCIAL,
//    HEALTHCARE,
//    ENTERTAINMENT,
//    RETAIL,
//    PUBLIC_SERVICE,
//    GOVERNMENT,
//    RELIGIOUS,
//    OTHER
//};

enum streetTypes{
    MOTORWAY = 0,
    TRUCK,
    PRIMARY,
    SECONDARY,
    TERTIARY,
    REGULAR,
    SMALL
};

extern unsigned numStreetSegs;
extern unsigned numIntersections;
extern unsigned numStreet;
extern unsigned numFeatures;
extern unsigned numPOI;

extern vector <StreetSegmentInfo> searchByStreetSeg;
extern vector <vector<unsigned>> searchByIntersection;
extern vector <vector<LatLon>> latLonStreetSeg;
extern vector <LatLon> latLonIntersection;
extern vector <LatLon> latLonPointOfInterest;
extern vector <string> streetNames;
extern vector <vector<unsigned>>  searchStreetSegByStreetID;
extern vector <double> streetSegTravelTime;
extern unordered_multimap <string, unsigned> searchByStreetName;
extern vector <FeatureData> FeaturesVector;
extern KDMap intersectionMap;
extern KDMap pointsOfInterestMap;
// an array of vectors written in terms of street_ids contain array of vector that contains all of the intersection in order on the street
extern vector <vector<unsigned>> searchIntersectionsByStreetID;
extern vector <vector<unsigned>> searchStreetSegByStreetID_inorder;

extern unordered_map <OSMID,const OSMWay*> searchStreetByOSMID;
extern vector<unsigned> motorwayStreets;
extern vector<unsigned> truckStreets;
extern vector<unsigned> primaryStreets;
extern vector<unsigned> secondaryStreets;
extern vector<unsigned> tertiaryStreets;
extern vector<unsigned> regularStreets;
extern vector<unsigned> smallStreets;
extern vector<string> streetSegTags;
extern vector<bool> streetLoop;

extern unordered_map <OSMID,const OSMNode*> searchPOIByOSMID;
extern vector<unsigned> foodPOI;
extern vector<unsigned> educationPOI;
extern vector<unsigned> transportationPOI;
extern vector<unsigned> financialPOI;
extern vector<unsigned> healthcarePOI;
extern vector<unsigned> entertainmentPOI;
extern vector<unsigned> retailPOI;
extern vector<unsigned> publicServicePOI;
extern vector<unsigned> governmentPOI;
extern vector<unsigned> religiousPOI;
extern vector<unsigned> otherPOI;
extern vector<string> POITags;