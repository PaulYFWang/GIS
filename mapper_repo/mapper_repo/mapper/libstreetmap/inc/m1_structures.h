#pragma once

#include <vector>
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"

using namespace std;

struct FeatureData {
    FeatureData(string _name, FeatureType _type, TypedOSMID _OSMID, unsigned _numFeaturePts, vector<LatLon> _featurePts){
        name = _name;
        type=_type;
        OSMID = _OSMID;
        numFeaturePts = _numFeaturePts;
        featurePts = _featurePts;
    }
    string name;
    FeatureType type;
    TypedOSMID OSMID;
    unsigned numFeaturePts;
    vector <LatLon> featurePts;
};
