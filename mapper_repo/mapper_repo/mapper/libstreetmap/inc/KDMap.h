#pragma once

#include "StreetsDatabaseAPI.h"
#include <cmath>
#include <vector>

using namespace std;

const double EARTH_RADIUS_IN_METERS_1 = 6372797.560856;
const double DEG_TO_RAD_1 = 0.017453292519943295769236907684886;
const int numDimension = 2;

class KDNode {

    private:
        vector<double> coords;

        unsigned id;
        KDNode* left;
        KDNode* right;

    public:

        KDNode(LatLon l, unsigned _id);
        
        ~KDNode();

        vector<double> getCoords();
        
        unsigned getID();

        void setLeft(KDNode* newLeft);

        // sets the right child of the KDNode
        void setRight(KDNode* newRight);

        // gets the left child of the KDNode.
        KDNode* getLeft() const;

        // gets the right child of the KDNode
        KDNode* getRight() const;

};

class TagsNode : public KDNode {
private:
    vector <string> tags;
    
public:
    TagsNode(LatLon l, unsigned _id, vector<string> _tags);
    
    
};
    
class KDMap {

    private:

        KDNode* root;
        KDNode* best_node;
        vector<KDNode*> nodes_in_range;
        double best_dist;
        int visited;

    public:

        KDMap();
        
        ~KDMap();
        
        void deleteAll();

        bool insertNode(LatLon l, unsigned _id);

        void insertNodeRecursive(KDNode* entry, KDNode* curNode, int curDim);
        
        KDNode* findNearestPoint(LatLon _query);
        
        vector<KDNode*> findNearestInRange(LatLon _query, double range);

        void recursiveFind(vector<double> query, KDNode* curNode, int curDim);
        
        void recursiveFindInRange(vector<double> query, KDNode* curNode, int curDim, double range);

        vector<double> diffs2(LatLon point1, LatLon point2);

        double distanceXY2(LatLon point1, LatLon point2);
        
        double distance2 (vector<double> point1, vector<double> point2 );
        
        void deleteMap(KDNode* n);

};