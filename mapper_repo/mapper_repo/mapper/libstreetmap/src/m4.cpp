#include "m4.h"
#include "m3.h"
#include "m1.h"
#include "../inc/path_finding_struct.h"
#include "../inc/m1_globals.h"
#include "StreetsDatabaseAPI.h"
#include "../inc/genetic.h"

#include <chrono>
#include <climits>
#include <cfloat>
#include <tuple>
#include <unordered_map>
#include <vector>

#define TIME_LIMIT 30

enum DNTYPE{
    PICKUP = 0,
    DROPOFF
};

const int considered_starts = 4;

using namespace std;

extern vector<LatLon> latLonIntersection;

struct comp{
    bool operator()(const pair<unsigned,DNTYPE>& lhs, const pair<unsigned,DNTYPE>& rhs){
        return lhs.first < rhs.first;
    }
};

struct comp2{
    bool operator()(const pair<unsigned,double>& lhs, const pair<unsigned,double>& rhs){
        return lhs.second < rhs.second;
    }
};

std::vector<unsigned> traveling_courier(const std::vector<DeliveryInfo>& deliveries, 
                                        const std::vector<unsigned>& depots, 
                                        const float turn_penalty){
    
    auto start_time = chrono::high_resolution_clock::now();
    bool timeout = false;
    
    //make unordered_map for quick access to pickup dropoffs
    unordered_multimap<unsigned,unsigned> delivery_info;
    
    //make open set
    set<pair<unsigned,DNTYPE>,comp> default_open;
    
    //populate unordered map and open
    for(auto itr = deliveries.begin(); itr != deliveries.end(); itr++){
        delivery_info.insert(make_pair(itr->pickUp,itr->dropOff));
        default_open.insert(make_pair(itr->pickUp,PICKUP));
    }

    //stuff for 2opt
    vector <unsigned> pickUp;
    vector <unsigned> dropOff;
    unordered_multimap <unsigned,unsigned> itemByPickUp;
    unordered_multimap <unsigned,unsigned> itemByDropOff;
    srand(time(NULL));
    for (unsigned i = 0; i < deliveries.size(); i++){
        pickUp.push_back(deliveries[i].pickUp);
        dropOff.push_back(deliveries[i].dropOff);
        itemByPickUp.insert(make_pair(deliveries[i].pickUp,i));
        itemByDropOff.insert(make_pair(deliveries[i].dropOff,i));
    }
    
    vector<unsigned> final_result_intersections;
    vector<unsigned> final_result;
    
    double total_cost = DBL_MAX;
    
    #pragma omp parallel for
    for(unsigned i = 0; i < depots.size(); i++){
        
        set<pair<unsigned,DNTYPE>,comp> open = default_open;
        
//        unsigned best_node;
//        double best_cost = DBL_MAX;
//        for(auto itr = open.begin(); itr != open.end(); itr++){
//            vector<unsigned> path = find_path_between_intersections_fast(depots[i],itr->first,turn_penalty);
//            double cost;
//            if(path.size() == 0)
//                continue;
//            else
//                cost = compute_path_travel_time(path,turn_penalty);
//            if(cost < best_cost){
//                best_node = itr->first;
//                best_cost = cost;
//            }
//        }
//        if(best_cost == DBL_MAX)
//            continue;
        
        
        set<pair<unsigned,double>,comp2> sorted_starts;
        
        for(auto itr = open.begin(); itr != open.end(); itr++){
            vector<unsigned> path = find_path_between_intersections_fast(depots[i],itr->first,turn_penalty);
            double cost;
            if(path.size() == 0)
                continue;
            else
                cost = compute_path_travel_time(path,turn_penalty);
            sorted_starts.insert(make_pair(itr->first,cost));
        }
        
        vector<unsigned> start_pickups;
        
//        if(sorted_starts.size() == 0)
//            continue;
//        if(sorted_starts.size() > 1){
//            start_pickups.push_back(sorted_starts.begin()->first);
//            sorted_starts.erase(sorted_starts.begin());
//            start_pickups.push_back(sorted_starts.begin()->first);
//        }
//        else{
//            start_pickups.push_back(sorted_starts.begin()->first);
//        }
        
        if(sorted_starts.size() >= considered_starts){
            for(int i = 0; i < considered_starts; i++){
                start_pickups.push_back(sorted_starts.begin()->first);
                sorted_starts.erase(sorted_starts.begin());
            }
        }
        else{
            for(int i = 0; i < sorted_starts.size(); i++){
                start_pickups.push_back(sorted_starts.begin()->first);
                sorted_starts.erase(sorted_starts.begin());
            }
        }
        
        for(auto starts = start_pickups.begin(); starts != start_pickups.end(); starts++){
            
            vector<unsigned> result_intersections;
            
            auto node = open.find(make_pair(*starts,PICKUP));
            DNTYPE type = node->second;
            open.erase(node);
            if(type == PICKUP){
                auto thing = delivery_info.equal_range(*starts);
                for(auto itr = thing.first; itr != thing.second; itr++){
                    open.insert(make_pair(itr->second,DROPOFF));
                }
            }

            result_intersections.push_back(depots[i]);
            result_intersections.push_back(*starts);

            unsigned prev_node = *starts;
            unsigned best_node;
            double best_cost;
            
            while(!open.empty()){

                best_cost = DBL_MAX;
                for(auto itr = open.begin(); itr != open.end(); itr++){
                    LatLon p1 = latLonIntersection[prev_node];
                    LatLon p2 = latLonIntersection[itr->first];
                    double cost = find_distance_between_two_points(p1,p2);
                    if(cost < best_cost){
                        best_node = itr->first;
                        best_cost = cost;
                    }
                }

                auto node = open.find(make_pair(best_node,PICKUP));
                DNTYPE type = node->second;
                open.erase(node);
                if(type == PICKUP){
                    auto thing = delivery_info.equal_range(best_node);
                    for(auto itr = thing.first; itr != thing.second; itr++){
                        open.insert(make_pair(itr->second,DROPOFF));
                    }
                }

                result_intersections.push_back(best_node);
                prev_node = best_node;

            }
            
            open = default_open;

            best_cost = DBL_MAX;
            for(auto itr = depots.begin(); itr != depots.end(); itr++){
                vector<unsigned> path = find_path_between_intersections_fast(prev_node,*itr,turn_penalty);
                double cost;
                if(path.size() == 0)
                    continue;
                else
                    cost = compute_path_travel_time(path,turn_penalty);
                if(cost < best_cost){
                    best_node = *itr;
                    best_cost = cost;
                }
            }

            result_intersections.push_back(best_node);
            vector<unsigned> result;
            for(auto itr = result_intersections.begin(); itr != result_intersections.end()-1; itr++){
                vector<unsigned> interm_path = find_path_between_intersections(*itr,*(itr+1),turn_penalty);
                result.insert(result.end(),interm_path.begin(),interm_path.end());
            }
            
            double cost = compute_path_travel_time(result,turn_penalty);
            if(cost < total_cost){
                final_result_intersections = result_intersections;
                total_cost = cost;
            }
            
        }
        
    }
    
    while(!timeout){
        
        unsigned depot1,depot2;
        depot1 = final_result_intersections.front();
        depot2 = final_result_intersections.back();
        final_result_intersections.erase(final_result_intersections.begin());
        final_result_intersections.pop_back();

        opt2(pickUp,itemByDropOff,final_result_intersections,turn_penalty);

        final_result_intersections.insert(final_result_intersections.begin(),depot1);
        final_result_intersections.push_back(depot2);
        
        auto current_time = chrono::high_resolution_clock::now();
        auto wall_clock = chrono::duration_cast<chrono::duration<double>> (current_time - start_time);
        
        if(wall_clock.count() > 0.9 * TIME_LIMIT)
            timeout = true;
        
    }
    
    //generate complete path from intersections
    for(auto itr = final_result_intersections.begin(); itr != final_result_intersections.end()-1; itr++){
        vector<unsigned> interm_path = find_path_between_intersections(*itr,*(itr+1),turn_penalty);
        final_result.insert(final_result.end(),interm_path.begin(),interm_path.end());
    }
        
    return final_result;
}
