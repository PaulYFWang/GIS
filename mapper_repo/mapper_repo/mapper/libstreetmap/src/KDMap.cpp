#include "../inc/KDMap.h"

KDNode::KDNode(LatLon l, unsigned _id){

    //treats lon as x and lat as y in context of k-d map, 
    //ignore case of lon wrap around transition from 180 deg to -180 deg
    //as no point of interest exists there
    coords.reserve(numDimension);

    coords.push_back(l.lon());
    coords.push_back(l.lat());

    id = _id;
    right = NULL;
    left = NULL;
}

KDNode::~KDNode() {}

vector<double> KDNode::getCoords(){
    return coords;
}
unsigned KDNode::getID(){
    return id;
}

void KDNode::setLeft(KDNode* newLeft){
    left = newLeft;
}

// sets the right child of the KDNode
void KDNode::setRight(KDNode* newRight){
    right = newRight;
}

// gets the left child of the KDNode.
KDNode* KDNode::getLeft() const{
    return left;
}

// gets the right child of the KDNode
KDNode* KDNode::getRight() const{
    return right;
}

 TagsNode::TagsNode(LatLon l, unsigned _id, vector<string> _tags):KDNode(l,_id){
    tags = _tags;
}

KDMap::KDMap(){
    root = NULL;
    best_node = NULL;
    best_dist = 100000000000;
}
        
KDMap::~KDMap(){
    deleteMap(root);
}

void KDMap::deleteAll(){
    deleteMap(root);
}
        
bool KDMap::insertNode(LatLon l, unsigned _id){
    if(root == NULL)
        root = new KDNode(l,_id);
    else{
        KDNode* entry = new KDNode(l,_id);
        insertNodeRecursive(entry,root,0);
    }

    return true;
}
  
void KDMap::insertNodeRecursive(KDNode* entry, KDNode* curNode, int curDim){
    //right insertions
    if(curNode->getCoords()[curDim % numDimension]<= entry->getCoords()[curDim % numDimension]){
        if(curNode->getRight() == NULL){
            curNode->setRight(entry);
            // cout << "RIGHT" <<endl;
        }
        else 
            insertNodeRecursive(entry, curNode->getRight(), curDim+1);
    }
    // insertion is to the left
    else {
        if(curNode->getLeft()==NULL){
            curNode->setLeft(entry);
            // cout << "LEFT" <<endl;
        }
        else 
            insertNodeRecursive(entry, curNode->getLeft(), curDim+1);  
    }   
}
        
KDNode* KDMap::findNearestPoint(LatLon _query){
    vector<double> query(numDimension);
    query[0] = _query.lon();
    query[1] = _query.lat();
    visited = 0;
    best_node = NULL;
    //cout << best_node->getID() << endl;
    // best_node = root;
    best_dist = 100000000000;
    if (root==NULL)
        return NULL;
    else{
        recursiveFind(query,root,0);
        //	cout << "Visited " << visited <<  endl;
        //	cout << "LatLon of best is " << best_node->getCoords()[0] << "," << best_node->getCoords()[1] << endl;
        //	cout << "best distance is " << best_dist <<  endl;
        return best_node;


    }
}

vector<KDNode*> KDMap::findNearestInRange(LatLon _query, double range){
    
    vector<double> query(numDimension);
    query[0] = _query.lon();
    query[1] = _query.lat();
    
    nodes_in_range.clear();

    recursiveFindInRange(query,root,0,range);
    
    cout << "visited: " << visited << endl;
    
    visited = 0;
    
    return nodes_in_range;
}

void KDMap::recursiveFind(vector<double> query, KDNode* curNode, int curDim){
    double d, dx, dx2;
    if (curNode == NULL)
        return;
    d = distanceXY2(LatLon(curNode->getCoords()[1],curNode->getCoords()[0]), LatLon(query[1],query[0]));
    dx = curNode->getCoords()[curDim%numDimension] - query[curDim%numDimension];
    dx2 = diffs2(LatLon(curNode->getCoords()[1],curNode->getCoords()[0]),LatLon(query[1],query[0]))[curDim%numDimension];

    visited ++;

    if(best_node == NULL || d < best_dist){
        best_dist = d;
        //cout << "New distance is " << d << endl;
        //cout <<std::setprecision(10)<< "Swapped Coords is :" << curNode->getCoords()[0] << "," << curNode->getCoords()[1] << " ID " << curNode->getID() << endl;
        best_node = curNode;	
    }
    recursiveFind(query,dx > 0? curNode->getLeft():curNode->getRight(), curDim +1);
    if (dx2 >=  best_dist) 
        return;

    recursiveFind(query,dx > 0? curNode->getRight():curNode->getLeft(), curDim +1);

}

void KDMap::recursiveFindInRange(vector<double> query, KDNode* curNode, int curDim, double range){
    double d, dx, dx2;
    if (curNode == NULL)
        return;
    d = distanceXY2(LatLon(curNode->getCoords()[1],curNode->getCoords()[0]), LatLon(query[1],query[0]));
    dx = curNode->getCoords()[curDim%numDimension] - query[curDim%numDimension];
    dx2 = diffs2(LatLon(curNode->getCoords()[1],curNode->getCoords()[0]),LatLon(query[1],query[0]))[curDim%numDimension];

    visited ++;
    
    cout << d << endl;

    if(d <= range){
        cout << "found node in range" << endl;
        //cout <<std::setprecision(10)<< "Swapped Coords is :" << curNode->getCoords()[0] << "," << curNode->getCoords()[1] << " ID " << curNode->getID() << endl;
        nodes_in_range.push_back(curNode);	
    }
    recursiveFindInRange(query,dx > 0? curNode->getLeft():curNode->getRight(), curDim +1, range);
    if (dx2 >= range) 
        return;

    recursiveFindInRange(query,dx > 0? curNode->getRight():curNode->getLeft(), curDim +1, range);

}

vector<double> KDMap::diffs2(LatLon point1, LatLon point2){

    //load in latitude and longitudes of both points
    double lat1 = point1.lat()*DEG_TO_RAD_1;
    double lon1 = point1.lon()*DEG_TO_RAD_1;
    double lat2 = point2.lat()*DEG_TO_RAD_1;
    double lon2 = point2.lon()*DEG_TO_RAD_1;

    //convert lat/long to x,y positions
    double lonFactor = cos((lat1+lat2)/2);
    double x1 = lon1*lonFactor;
    double y1 = lat1;
    double x2 = lon2*lonFactor;
    double y2 = lat2;

    //calculate distance
    vector <double> diffs;
    diffs.push_back(EARTH_RADIUS_IN_METERS_1*(x2-x1)*(x2-x1));
    diffs.push_back(EARTH_RADIUS_IN_METERS_1*(y2-y1)*(y2-y1));

    return diffs;
}
   
double KDMap::distanceXY2(LatLon point1, LatLon point2){
    
    //load in latitude and longitudes of both points
    double lat1 = point1.lat()*DEG_TO_RAD_1;
    double lon1 = point1.lon()*DEG_TO_RAD_1;
    double lat2 = point2.lat()*DEG_TO_RAD_1;
    double lon2 = point2.lon()*DEG_TO_RAD_1;

    //convert lat/long to x,y positions
    double lonFactor = cos((lat1+lat2)/2);
    double x1 = lon1*lonFactor;
    double y1 = lat1;
    double x2 = lon2*lonFactor;
    double y2 = lat2;

    //calculate distance
    double magnitudex = (x2-x1)*(x2-x1);
    double magnitudey = (y2-y1)*(y2-y1);
    double d2 = EARTH_RADIUS_IN_METERS_1*(magnitudex+magnitudey);

    return d2;
}

double KDMap::distance2 (vector<double> point1, vector<double> point2 ){
    //load in latitude and longitudes of both points
    double lat1 = point1[1];
    double lon1 = point1[0];
    double lat2 = point2[1];
    double lon2 = point2[0];

    //calculate distance
    double magnitudex = (lon2-lon1)*(lon2-lon1);
    double magnitudey = (lat2-lat1)*(lat2-lat1);
    double d2 = (magnitudex+magnitudey);

    return d2;
}

void KDMap::deleteMap(KDNode* n){
    if(n == NULL)
        return;
    else{
        deleteMap(n->getRight());
        deleteMap(n->getLeft());
        delete n;
    }
}