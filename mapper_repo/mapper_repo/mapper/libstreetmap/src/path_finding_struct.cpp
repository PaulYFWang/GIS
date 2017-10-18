/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "m1.h"
#include "m3.h"
#include "../inc/path_finding_struct.h"
#include "StreetsDatabaseAPI.h"
#include <climits>
#include <cfloat>
#include <limits>


using namespace std;
extern unsigned numStreetSegs;
extern unsigned numIntersections;
extern vector <LatLon> latLonIntersection;
extern vector <vector<unsigned>> searchByIntersection;
extern vector <StreetSegmentInfo> searchByStreetSeg;
extern vector<double>streetSegTravelTime;

//this data structure is indexed by intersection id and stores distance value 
//that is going to be used in pathfinding algorithm as well as all of the 
//adjacent intersections
//more data might be added later for the purpose of the algorithm
vector < vector <int_cost> > searchDataStruct;

void populatePathFind(){

    searchDataStruct.resize(numIntersections);
    
    for(unsigned i = 0;i<numIntersections;i++){
        
        vector<unsigned> adj_int = find_adjacent_intersections(i);
        for(auto itr = adj_int.begin();itr != adj_int.end();itr++){
            int_cost newCost;
            
            vector <unsigned> intersect;
            intersect.push_back(i);
            intersect.push_back(*itr);
            
            vector <unsigned> segments = return_streetSegID_from_intIDs(intersect);
            
            if(segments.size() == 0){
                continue;
            }
            
            for(auto itr2 = segments.begin(); itr2 != segments.end(); itr2++){
                newCost.intersectionid = *itr;
                newCost.streetsegid = *itr2;
                newCost.cost = streetSegTravelTime[*itr2];
                searchDataStruct[i].push_back(newCost);
            }
        }
    }
}

bool check_all_street_segs_connected(const vector<unsigned>& street_segs){
    for(auto itr = street_segs.begin(); itr != street_segs.end()-1; itr++){
        StreetSegmentInfo seg1 = searchByStreetSeg[*itr];
        StreetSegmentInfo seg2 = searchByStreetSeg[*(itr+1)];
        if(seg1.oneWay){
            if(seg2.oneWay){
                if(seg1.to == seg2.from)
                    continue;
            }
            else{
                if(seg1.to == seg2.to || seg1.to == seg2.from)
                    continue;
            }
            cout << "mismatched oneway streets" << endl;
            return false;
        }
        else if(seg2.oneWay){
            if(seg2.from == seg1.to || seg2.from == seg1.from)
                continue;
            cout << "mismatched oneway streets" << endl;
            return false;
        }
        else{
            if(seg1.to == seg2.to || seg1.to == seg2.from || seg1.from == seg2.to || seg1.from == seg2.from)
                continue;
        }
        return false;
    }
    return true;
}

vector<unsigned> return_streetSegID_from_intIDs(const vector<unsigned>& intersectionids){
    unsigned intersection_id1,intersection_id2;
    vector<unsigned> streetSegIDs;
//    cout << intersectionids.size() << endl;
    for(auto itr = intersectionids.begin();itr<intersectionids.end()-1;itr++){
        
        intersection_id1 = *itr;
        intersection_id2 = *(itr+1);
        
        if(intersection_id1 == intersection_id2){
            cout<<"cul-de-sac near by"<<endl;
            continue;
        }else{
            //vector<unsigned> temp;
            unsigned index = 0;
            for(auto ssid_iter = searchByIntersection[intersection_id1].begin();
                ssid_iter != searchByIntersection[intersection_id1].end(); ssid_iter++){
                
                if((searchByStreetSeg[*ssid_iter].from == intersection_id1 &&
                    searchByStreetSeg[*ssid_iter].to == intersection_id2) ||
                   (searchByStreetSeg[*ssid_iter].to == intersection_id1 &&
                    searchByStreetSeg[*ssid_iter].from == intersection_id2 &&
                    searchByStreetSeg[*ssid_iter].oneWay == false)){
                    //temp.push_back(searchByIntersection[intersection_id1][index]);
                    streetSegIDs.push_back(searchByIntersection[intersection_id1][index]);
                }
                
                index++;
            }
        }
    }
    if(streetSegIDs.size()==0){
        cout<<"streetSegIDs empty, no connecting segments" << endl;;
    }
    
    return streetSegIDs;
}