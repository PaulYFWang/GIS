/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   genetic.h
 * Author: wangy384
 *
 * Created on April 2, 2017, 11:59 PM
 */
#include "m4.h"
#include "OSMStructures.h"
#include <vector>
#include <queue>
#include <set>
#include <unordered_map>
#include "../inc/path_finding_struct.h"

#ifndef GENETIC_H
#define GENETIC_H
bool checkValidity(vector <unsigned> pickUp, unordered_multimap <unsigned,unsigned> itemByDropOff, vector <unsigned> path);

bool crossover1 (vector <unsigned> path1, vector <unsigned> path2, vector< pair < vector<unsigned> , double> >& population,unsigned popSize,float turn_penalty);

bool crossover2 (vector <unsigned> pickUp, unordered_multimap <unsigned,unsigned> itemByDropOff,vector <unsigned> path1, vector <unsigned> path2, vector< pair < vector<unsigned> , double> >& population,unsigned popSize,float turn_penalty);

void mutation1(vector <unsigned>& path, double chance);


void depopulation (vector <unsigned> pickUp, unordered_multimap <unsigned,unsigned> itemByDropOff,vector< pair < vector<unsigned> , double> >& population, unsigned rate, float turn_penalty);

void repopulation(vector <unsigned> pickUp, unordered_multimap <unsigned,unsigned> itemByDropOff,vector< pair < vector<unsigned> , double> >& population, unsigned popSize, float turn_penalty);

double fitness(vector <unsigned> path, float turn_penalty);

void enforceValidity(vector <unsigned> pickUp, unordered_multimap <unsigned,unsigned> itemByDropOff, vector <unsigned> &path);

void opt2 (vector <unsigned> pickUp, unordered_multimap <unsigned,unsigned> itemByDropOff, vector <unsigned> &path, float turn_penalty);
#endif /* GENETIC_H */

