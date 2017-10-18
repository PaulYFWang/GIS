#include "../inc/genetic.h"
#include "m3.h"
#include "m4.h"
#include "m1.h"
#include <ctime>
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "../inc/m1_structures.h"
#include "../inc/KDMap.h"
#include "../inc/OSMStructures.h"
#include "../inc/path_finding_struct.h"
/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */


using namespace std;
extern vector <LatLon> latLonIntersection;
bool checkValidity(vector <unsigned> pickUp, unordered_multimap <unsigned,unsigned> itemByDropOff, vector <unsigned> path){
    
    for ( auto itr = path.begin(); itr != path.end(); itr++){
        
        for (auto itr2 = itemByDropOff.equal_range(*itr).first ; itr2 !=itemByDropOff.equal_range(*itr).second; itr2++){
            
            if (find(path.begin(),itr,pickUp[itr2->second]) ==itr)
                return false;
        }
    }
    
    return true;
}

void enforceValidity(vector <unsigned> pickUp, unordered_multimap <unsigned,unsigned> itemByDropOff, vector <unsigned> &path){
    
    for ( auto itr = path.begin(); itr != path.end(); itr++){
        
        
        for (auto itr2 = itemByDropOff.equal_range(*itr).first ; itr2 !=itemByDropOff.equal_range(*itr).second; itr2++){
            
            if (find(itr,path.end(),pickUp[itr2->second]) !=path.end()){
                iter_swap(itr,find(itr,path.end(),pickUp[itr2->second]));
                break;
            }
                
        }
    }
           
}


bool crossover1 (vector <unsigned> path1, vector <unsigned> path2, vector< pair < vector<unsigned> , double> >& population, unsigned popSize,float turn_penalty){
    
    //takes substring from parent 1 / parent 2 to cutoff point. repeats in parent substrings force swaps in parent 2 
    /*
     *  parent1 = 2 1 6 4|5 3
     * parent 2 = 2 6 4 5|3 1
     * 
     * cutoff = 3 (index)
     * 
     *   parent1 = 1 2 6 4|5 3
     * parent 2 = 1 2 6 4|3 5
     * child1 = 1 2 3 4 | 3 5
     * 
     *      
     
     */
    vector <unsigned> child1;
    vector <unsigned> child2;
    vector <unsigned> path1Copy = path1;
     vector <unsigned> path2Copy = path2;
     double chance = 0.01/100;
 
    unsigned cutoff = rand()%(path1.size());
    
    for (unsigned i = 0; i < cutoff ; i++){
        child1.push_back(path1[i]);
        auto it = find(path2Copy.begin(),path2Copy.end(),path1[i]);
        unsigned dist = distance(path2Copy.begin(), it);
        unsigned temp = path2Copy[i];
        path2Copy[i] = path2Copy[dist];
        path2Copy[dist] = temp;
        
        //child2.push_back(path2[i]);
    }
    for (unsigned i = cutoff; i < path2.size() ; i++){
        child1.push_back(path2[i]);
    }
    
    mutation1(child1, chance);
    
    for (unsigned i = 0; i < cutoff ; i++){
        child2.push_back(path2[i]);
        auto it = find(path1Copy.begin(),path1Copy.end(),path1[i]);
        unsigned dist = distance(path1Copy.begin(), it);
        unsigned temp = path1Copy[i];
        path1Copy[i] = path1Copy[dist];
        path1Copy[dist] = temp;
        
        //child2.push_back(path2[i]);
    }
    for (unsigned i = cutoff; i < path1.size() ; i++){
        child2.push_back(path1[i]);
    }
    
     mutation1(child2, chance);
     
     if (population.size() <popSize){
         population.push_back(make_pair(child1, fitness(child1,turn_penalty)));
         if (population.size() <popSize)
             population.push_back(make_pair(child2, fitness(child2,turn_penalty)));
         else
             return false;
         return true;
     }
     return false;
     
}

bool crossover2(vector <unsigned> pickUp, unordered_multimap <unsigned,unsigned> itemByDropOff,vector <unsigned> path1, vector <unsigned> path2, vector< pair < vector<unsigned> , double> >& population,unsigned popSize,float turn_penalty){
    vector <unsigned> child1;
    vector <unsigned> child2;
    vector <unsigned> path1Copy = path1;
    vector <unsigned> path2Copy = path2;
     double chance = 0.15-(population[0].second-population[1].second)/(2*(population[0].second+population[1].second));
    
    unsigned cutoff = rand()%path1.size();
    
    //bool ensureValid = checkValidity(pickUp, itemByDropOff, path1) && checkValidity(pickUp, itemByDropOff, path2) ;
    
    
    for (unsigned i =0; i < cutoff; i++){
        child1.push_back(path1[i]);
        path2Copy.erase(remove(path2Copy.begin(),path2Copy.end(),path1[i]),path2Copy.end());
        child2.push_back(path2[i]);
        path1Copy.erase(remove(path1Copy.begin(),path1Copy.end(),path2[i]),path1Copy.end());
    }
//    if (path1Copy.size()!= path2Copy.size())
//        cout <<"YA FUCKED UP " << endl;
    
    for (unsigned i =0; i < path2Copy.size(); i++){
        child1.push_back(path2Copy[i]);
        child2.push_back(path1Copy[i]);
    }
    mutation1(child1,chance);
    mutation1(child2,chance);
//    cout <<"CHILD ONE: ";
//    for (auto itr = child1.begin(); itr != child1.end(); itr ++){
//        cout << (*itr) << " ";
//    }
//    cout << endl;
    
    //cout << "c1: " <<  child1[0] << " " << child1[1] << "p1" << path1[0] << " " << path1[1] << endl;
     
     if (population.size() <popSize){
//         if (!ensureValid)
//         enforceValidity(pickUp, itemByDropOff, child1);
         //population.push_back(make_pair(child1, fitness(child1,turn_penalty)));
         population.push_back(make_pair(child1, -1));
         if (population.size() <popSize){
//             if (!ensureValid)
//             enforceValidity(pickUp, itemByDropOff, child2);
             //population.push_back(make_pair(child2, fitness(child2,turn_penalty)));
             population.push_back(make_pair(child2, -1));
         }
         else
             return false;
         return true;
     }
     return false;
}

void mutation1(vector <unsigned>& path, double chance){ //chance is a decimal < 1 and > 0 
    //swap node mutation
    //cout << "mutation" << endl;
    if (chance*10000 > rand()%10000){

        unsigned s1,s2;
        s1 = rand()%(path.size());
        s2 = rand()%(path.size());
        unsigned temp;
        temp = path[s1];
        path[s1] = path[s2];
        path[s2] = temp;
    }
    else
        return;
}


void repopulation(vector <unsigned> pickUp, unordered_multimap <unsigned,unsigned> itemByDropOff,vector< pair < vector<unsigned> , double> >& population, unsigned popSize, float turn_penalty){
    bool notDone;
    unsigned originalPop = population.size();
    
    unsigned parent1 = rand()%(originalPop);
    unsigned parent2 = rand()%(originalPop);
    
    do{
        //cout << "BREED SIZE IS" << originalPop  << " cur pop  is " << population.size()<< endl;
    
        parent1 = rand()%(originalPop);

        parent2 = rand()%(originalPop);
        while(parent1 == parent2)
            parent2 = rand()%(originalPop);
        //cout << rand() << endl;
        //cout << "P1 and p2 is :" << parent1 << " " << parent2 << endl;
        notDone = crossover2(pickUp, itemByDropOff,population[parent1].first, population[parent2].first,population, popSize,turn_penalty);
    }while(notDone);
    
}


void depopulation (vector <unsigned> pickUp, unordered_multimap <unsigned,unsigned> itemByDropOff,vector< pair < vector<unsigned> , double> >& population, unsigned rate, float turn_penalty){
    for (auto itr = (population.begin()+population.size()/rate); itr != population.end(); itr ++){
//        if (!checkValidity(pickUp, itemByDropOff, itr->first)){
//            //opt2(pickUp, itemByDropOff, itr->first, turn_penalty);
//            itr->second = 0;
//        }
//        else
        
        //enforceValidity(pickUp, itemByDropOff,itr->first);
        if (itr->second =-1){
            //opt2(pickUp, itemByDropOff, itr->first, turn_penalty);
            if(checkValidity( pickUp, itemByDropOff, itr->first))
            itr->second = fitness(itr->first, turn_penalty);
            else
                itr->second = 0;
        }
        //enforceValidity(pickUp, itemByDropOff, itr->first);
//         if (itr->second!=0)
//                itr->second = fitness(itr->first, turn_penalty);
//        
    }
    
      sort(population.begin(), population.end(),[](pair < vector<unsigned> , double>  const &t1, pair < vector<unsigned> , double> const &t2) {
              return t1.second > t2.second; // comparator
          });
          population.erase(population.begin()+population.size()/rate, population.end());
//          for (auto itr = population.begin(); itr != population.end(); itr++){
//              cout << "Rank " << itr-population.begin() << " Has fitness  "  << itr->second << endl;
//          }
          //cout <<"finish depop" << endl;
}
   
void opt2 (vector <unsigned> pickUp, unordered_multimap <unsigned,unsigned> itemByDropOff, vector <unsigned> &path, float turn_penalty){
    int counter = 0;
    vector <unsigned> potential;
    potential = path;
    for (unsigned i =1;i< potential.size()-2 ; i++){
        //cout << "In opt2 " <<  "size " << potential.size() <<  " "  << counter << endl;
        for (unsigned j = i+1; j< potential.size()-1 ;j++){
            
            vector<unsigned> reverseTemp;
            //cout << i <<" , " << j <<endl;
            double sumABCD = compute_path_travel_time(find_path_between_intersections_fast(potential[j+1],potential[j],turn_penalty),turn_penalty)+compute_path_travel_time(find_path_between_intersections_fast(potential[i-1],potential[i],turn_penalty),turn_penalty);
            double sumACBD = compute_path_travel_time(find_path_between_intersections_fast(potential[i-1],potential[j],turn_penalty),turn_penalty)+compute_path_travel_time(find_path_between_intersections_fast(potential[i],potential[j+1],turn_penalty),turn_penalty);
//            LatLon a,b,c,d;
//            a = latLonIntersection[potential[i-1]];
//            b = latLonIntersection[potential[i]];
//            c = latLonIntersection[potential[j]];
//            d = latLonIntersection[potential[j+1]];
//            
//            double sumABCD = find_distance_between_two_points(a,b) + find_distance_between_two_points(c,d);
//            double sumACBD = find_distance_between_two_points(a,c) + find_distance_between_two_points(b,d);
//            
            if (sumABCD > sumACBD){
                for (unsigned k = i+1; k <=j-1 ; k++){
                    reverseTemp.push_back(potential[k]);
                }
                reverseTemp.push_back(potential[i]);
                reverseTemp.insert(reverseTemp.begin(),potential[j]);
                if (checkValidity(pickUp,itemByDropOff,reverseTemp)){
                    unsigned temp = potential[j];
                    potential[j] = potential[i];
                    potential[i] = temp;
                    counter++;
                }
                
                
            }
            reverseTemp.clear();
        }
    }
    if (fitness(potential,turn_penalty) > fitness(path,turn_penalty))
        path = potential;
}

double fitness(vector <unsigned> path, float turn_penalty){
    double sum = 0;
#pragma omp parallel for reduction ( + : sum )
   
        for(unsigned j = 0; j< path.size()-1; j++){
            sum+= compute_path_travel_time(find_path_between_intersections(path[j],path[j+1],turn_penalty),turn_penalty);
        
    }
    //cout <<"Fitness is this" << 1/sum << endl;
    
    return (1/sum);
}

