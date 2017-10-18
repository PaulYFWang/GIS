#ifndef PATH_FINDING_STRUCT_H
#define PATH_FINDING_STRUCT_H

#include <vector>
#include <algorithm>
#include <utility>
#include <iostream>
#include <climits>
#include <cfloat>
    
using namespace std;

struct int_cost{
    double cost;
    unsigned streetsegid;
    unsigned intersectionid;
};

void populatePathFind();

double heuristic1(unsigned id1,unsigned id2);

double heuristic2(unsigned id1,unsigned id2);

vector<unsigned> return_streetSegID_from_intIDs(const vector<unsigned>& intersectionids);

bool check_all_street_segs_connected(const vector<unsigned>& street_segs);

bool check_turn_penalty(unsigned id1,unsigned id2);

vector<unsigned> find_path_between_intersections_fast(const unsigned intersect_id_start,
                                                 const unsigned intersect_id_end,
                                                 const double turn_penalty);

vector<unsigned> find_path_between_intersections3(const unsigned intersect_id_start,
                                                 const unsigned intersect_id_end,
                                                 const double turn_penalty);

#endif /* PATH_FINDING_STRUCT_H */

