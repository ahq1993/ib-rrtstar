/*! 
 * \file rrts.hpp 
 */ 

#ifndef __RRTS_HPP_
#define __RRTS_HPP_

#include <iostream>
#include <cfloat>
#include <cmath>
#include <algorithm>


#include "rrts.h"

using namespace std;

template<class State, class Trajectory, class System>
RRTstar::Vertex<State, Trajectory, System>
::Vertex () {
    
    state = NULL;
    parent = NULL;
    trajFromParent = NULL;
    
    costFromParent = 0.0;
    costFromRoot = 0.0;
}


template<class State, class Trajectory, class System>
RRTstar::Vertex<State, Trajectory, System>
::~Vertex () {
    
    if (state)
        delete state;
    parent = NULL;
    if (trajFromParent)
        delete trajFromParent;
    children.clear();
}


template<class State, class Trajectory, class System>
RRTstar::Vertex<State, Trajectory, System>
::Vertex(const Vertex<State, Trajectory, System>& vertexIn) {
    
    if (vertexIn.state)
        state = new State (vertexIn.getState());
    else 
        state = NULL;
    parent = vertexIn.parent;
    for (typename std::set< Vertex<State,Trajectory,System> * >::const_iterator iter = vertexIn.children.begin(); iter != vertexIn.children.end(); iter++)
        children.insert (*iter);
    costFromParent = vertexIn.costFromParent;
    costFromRoot = vertexIn.costFromRoot;
    if (vertexIn.trajFromParent)
        trajFromParent = new Trajectory (*(vertexIn.trajFromParent));
    else 
        trajFromParent = NULL;
}




template<class State, class Trajectory, class System>
RRTstar::Planner<State, Trajectory, System>
::Planner () {
    
    gamma = 1.0;
    
    lowerBoundCost = DBL_MAX;
    lowerBoundVertex = NULL;
    
    
    lowerBoundCost2 = DBL_MAX;
    lowerBoundVertex2 = NULL;
    
    endtoend_Cost= DBL_MAX;
    lowerBoundVertexA = NULL;
    lowerBoundVertexB = NULL;
    kdtree = NULL;
    kdtree2 = NULL;
    
    root = NULL;
    root2 = NULL;
    numVertices = 0;
    numVertices2 = 0;
    system = NULL;
}


template<class State, class Trajectory, class System>
RRTstar::Planner<State, Trajectory, System>
::~Planner () {
    
    // Delete the kdtree structure
    if (kdtree) {
        kd_clear (kdtree);
        kd_free (kdtree);
    }
    if (kdtree2) {
        kd_clear (kdtree2);
        kd_free (kdtree2);
    }
    // Delete all the vertices
    for (typename std::list<Vertex <State,Trajectory,System> * >::iterator iter = listVertices.begin(); iter != listVertices.end(); iter++) 
        delete *iter;
    for (typename std::list<Vertex <State,Trajectory,System> * >::iterator iter = listVertices2.begin(); iter != listVertices2.end(); iter++) 
        delete *iter;
}


template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::insertIntoKdtree (Vertex<State,Trajectory,System>& vertexIn,int a) {
    
    double *stateKey = new double[numDimensions];
    system->getStateKey ( *(vertexIn.state), stateKey);
    if(a==1)
    kd_insert (kdtree, stateKey, &vertexIn);
    else
    kd_insert (kdtree2, stateKey, &vertexIn);    
    delete [] stateKey;
    
    return 1;
}


template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State,Trajectory,System>
::getNearestVertex (State& stateIn, Vertex<State,Trajectory,System>*& vertexPointerOut, int a) {
    
    // Get the state key for the query state
    double *stateKey = new double[numDimensions];
    system->getStateKey (stateIn, stateKey);
    KdRes *kdres;
    // Search the kdtree for the nearest vertex
    if(a==1){
    kdres = kd_nearest (kdtree, stateKey);
    }
    else
    {
    kdres = kd_nearest (kdtree2, stateKey);
    }
    if (kd_res_end (kdres))  
        vertexPointerOut = NULL;
    vertexPointerOut = (Vertex<State,Trajectory,System>*) kd_res_item_data (kdres);
    
    // Clear up the memory
    delete [] stateKey;
    kd_res_free (kdres);
    
    // Return a non-positive number if any errors
    if (vertexPointerOut == NULL)
        return 0;
    
    return 1;
}

template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State,Trajectory,System>
::getNearestVertex2 (State& stateIn, Vertex<State,Trajectory,System>*& vertexPointerOut, int a) {
    
    // Get the state key for the query state
    double *stateKey = new double[numDimensions];
    system->getStateKey (stateIn, stateKey);
    KdRes *kdres;
   
    // Search the kdtree for the nearest vertex
    if(a==1){
    kdres = kd_nearest (kdtree2, stateKey);
    
    }
    else
    {
    kdres = kd_nearest (kdtree, stateKey);
    
    }
    if (kd_res_end (kdres))  
        vertexPointerOut = NULL;
    vertexPointerOut = (Vertex<State,Trajectory,System>*) kd_res_item_data (kdres);
    
    // Clear up the memory
    delete [] stateKey;
    kd_res_free (kdres);
    
    // Return a non-positive number if any errors
    if (vertexPointerOut == NULL)
        return 0;
    
    return 1;
}


template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::getNearVertices (State& stateIn, std::vector< Vertex<State,Trajectory,System>* >& vectorNearVerticesOut,std::vector< Vertex<State,Trajectory,System>* >& vectorNearVerticesOut2) {
    
    // Get the state key for the query state
    double *stateKey = new double[numDimensions];
    system->getStateKey (stateIn, stateKey);


	// ball radius for tree 1 and 2	   
    double ballRadius = gamma * pow( log((double)(numVertices + 1.0))/((double)(numVertices + 1.0)), 1.0/((double)numDimensions) );
         KdRes *kdres = kd_nearest_range (kdtree, stateKey, ballRadius);
    
    double ballRadius2 = gamma * pow( log((double)(numVertices2 + 1.0))/((double)(numVertices2 + 1.0)), 1.0/((double)numDimensions) );
         KdRes *kdres2 = kd_nearest_range (kdtree2, stateKey, ballRadius2);
   
    delete [] stateKey;
    
    // Create the vector data structure for storing the results
    int numNearVertices = kd_res_size (kdres);
    int numNearVertices2 = kd_res_size (kdres2);
	  int  i = 0;
	  vectorNearVerticesOut.resize(numNearVertices);  
	  vectorNearVerticesOut2.resize(numNearVertices2);
	  kd_res_rewind (kdres);
	 while (!kd_res_end(kdres)) {
	  Vertex<State,Trajectory,System> *vertexCurr = (Vertex<State,Trajectory,System> *) kd_res_item_data (kdres);
	  vectorNearVerticesOut[i] = vertexCurr;
	  kd_res_next (kdres);
	  i++;
	 }
	  kd_res_free (kdres);
	  i = 0;
	  kd_res_rewind (kdres2);
	 while (!kd_res_end(kdres2)) {
	  Vertex<State,Trajectory,System> *vertexCurr = (Vertex<State,Trajectory,System> *) kd_res_item_data (kdres2);
	  vectorNearVerticesOut2[i] = vertexCurr;
	  kd_res_next (kdres2);
	  i++;
	 }
	 kd_res_free (kdres2); 
	 return 1;
   
}


template<class State, class Trajectory, class System>
int
RRTstar::Planner<State, Trajectory, System>
::checkUpdateBestVertex (Vertex<State,Trajectory,System>& vertexIn, int a) {
    if (a==1)
    {
        if (system->isReachingTarget(vertexIn.getState(),a)){
        
        
        double costCurr = vertexIn.getCost();
        if ( (lowerBoundVertex == NULL) || ( (lowerBoundVertex != NULL) && (costCurr < lowerBoundCost)) ) {

            lowerBoundVertex = &vertexIn;
            lowerBoundCost = costCurr;
        }
        }
    }
    else
    {
        if (system->isReachingTarget(vertexIn.getState(),a)){
        
        
        double costCurr = vertexIn.getCost();
        if ( (lowerBoundVertex2 == NULL) || ( (lowerBoundVertex2 != NULL) && (costCurr < lowerBoundCost2)) ) {

            lowerBoundVertex2 = &vertexIn;
            lowerBoundCost2 = costCurr;
        }
        }
    }
    
    return 1;
}
template<class State, class Trajectory, class System>
int
RRTstar::Planner<State, Trajectory, System>
::checkUpdateBestVertex2(Vertex<State,Trajectory,System>& vertexIn_T1,Vertex<State,Trajectory,System>& vertexIn_T2,double curr_endtoend_cost) {
    

if(curr_endtoend_cost< endtoend_Cost)
{
        endtoend_Cost=curr_endtoend_cost;
        lowerBoundVertexA=&vertexIn_T1;
        lowerBoundVertexB=&vertexIn_T2;
   
}

    
    return 1;
}

template<class State, class Trajectory, class System>
RRTstar::Vertex<State,Trajectory,System>*
RRTstar::Planner<State, Trajectory, System>
::insertTrajectory (Vertex<State,Trajectory,System>& vertexStartIn, Trajectory& trajectoryIn, int a) {
    
    // Check for admissible cost-to-go
    if(a==1)
    {
    if (lowerBoundVertex != NULL) {
        double costToGo = system->evaluateCostToGo (trajectoryIn.getEndState(),a);
        if (costToGo >= 0.0) 
            if (lowerBoundCost < vertexStartIn.getCost() + costToGo) 
                return NULL;
    }
    }
    else
    {
        if (lowerBoundVertex2 != NULL) {
        double costToGo = system->evaluateCostToGo (trajectoryIn.getEndState(),a);
        if (costToGo >= 0.0) 
            if (lowerBoundCost2 < vertexStartIn.getCost() + costToGo) 
                return NULL;
        }
    }
    
    // Create a new end vertex
    Vertex<State,Trajectory,System>* vertexNew = new Vertex<State,Trajectory,System>;
    vertexNew->state = new State;
    vertexNew->parent = NULL;
    vertexNew->getState() = trajectoryIn.getEndState();
    insertIntoKdtree (*vertexNew,a);
    if(a==1)
    {
        this->listVertices.push_front (vertexNew);
        this->numVertices++;
    }
    else
    {
        this->listVertices2.push_front (vertexNew);
        this->numVertices2++;
    }
    // Insert the trajectory between the start and end vertices
    insertTrajectory (vertexStartIn, trajectoryIn, *vertexNew,a);
    
    return vertexNew;
}


template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::insertTrajectory (Vertex<State,Trajectory,System>& vertexStartIn, Trajectory& trajectoryIn, Vertex<State,Trajectory,System>& vertexEndIn,int a) {

    // Update the costs
    vertexEndIn.costFromParent = trajectoryIn.evaluateCost();
    vertexEndIn.costFromRoot = vertexStartIn.costFromRoot + vertexEndIn.costFromParent;
    checkUpdateBestVertex (vertexEndIn,a);
    
    // Update the trajectory between the two vertices
    if (vertexEndIn.trajFromParent)
        delete vertexEndIn.trajFromParent;
    vertexEndIn.trajFromParent = new Trajectory (trajectoryIn);
    // Update the parent to the end vertex
    if (vertexEndIn.parent)
        vertexEndIn.parent->children.erase (&vertexEndIn);
    vertexEndIn.parent = &vertexStartIn;
    
    // Add the end vertex to the set of chilren
    vertexStartIn.children.insert (&vertexEndIn);
    
    return 1;
}


template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::setSystem (System& systemIn) {
    
    if (system)
        delete system;
    
    system = &systemIn;
    
    numDimensions = system->getNumDimensions ();
    
    // Delete all the vertices
    for (typename std::list< Vertex<State,Trajectory,System>* >::iterator iter = listVertices.begin(); iter != listVertices.end(); iter++)
        delete *iter;
    for (typename std::list< Vertex<State,Trajectory,System>* >::iterator iter = listVertices2.begin(); iter != listVertices2.end(); iter++)
        delete *iter;
    numVertices = 0;
    numVertices2=0;
    lowerBoundCost = DBL_MAX;
    lowerBoundVertex = NULL;
    lowerBoundCost2 = DBL_MAX;
    lowerBoundVertex2 = NULL;
    
    // Clear the kdtree
    if (kdtree) {
        kd_clear (kdtree);
        kd_free (kdtree);
    }
    if (kdtree2) {
        kd_clear (kdtree2);
        kd_free (kdtree2);
    }
    kdtree = kd_create (numDimensions);
    kdtree2 = kd_create (numDimensions);
    // Initialize the root vertex
    root = new Vertex<State,Trajectory,System>;
    root->state = new State (system->getRootState());
    root->costFromParent = 0.0;
    root->costFromRoot = 0.0;
    root->trajFromParent = NULL;
    
    root2 = new Vertex<State,Trajectory,System>;
    root2->state = new State (system->getRootState2());
    root2->costFromParent = 0.0;
    root2->costFromRoot = 0.0;
    root2->trajFromParent = NULL;
    
    return 1;
}



template<class State, class Trajectory, class System>
RRTstar::Vertex<State, Trajectory, System>& 
RRTstar::Planner<State, Trajectory, System>
::getRootVertex () {
    
    return *root;
}
template<class State, class Trajectory, class System>
RRTstar::Vertex<State, Trajectory, System>& 
RRTstar::Planner<State, Trajectory, System>
::getRootVertex2() {
    
    return *root2;
}



template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::initialize () {
    
    // If there is no system, then return failure
    if (!system)
        return 0;
    
    // Backup the root
    Vertex<State,Trajectory,System> *rootBackup = NULL;
    Vertex<State,Trajectory,System> *rootBackup2 = NULL;
    if (root)
        rootBackup = new Vertex<State,Trajectory,System> (*root);
    if (root2)
        rootBackup2 = new Vertex<State,Trajectory,System> (*root2);

    
    // Delete all the vertices
    for (typename std::list< Vertex<State,Trajectory,System>* >::iterator iter = listVertices.begin(); iter != listVertices.end(); iter++)
        delete *iter;
    listVertices.clear();
    for (typename std::list< Vertex<State,Trajectory,System>* >::iterator iter = listVertices2.begin(); iter != listVertices2.end(); iter++)
        delete *iter;
    listVertices2.clear();
    numVertices = 0;
    numVertices2 = 0;
    lowerBoundCost = DBL_MAX;
    lowerBoundCost2 = DBL_MAX;
    endtoend_Cost=DBL_MAX;
    lowerBoundVertex = NULL;
    lowerBoundVertex2 = NULL;
    lowerBoundVertexA = NULL;
    lowerBoundVertexB = NULL;
    
    // Clear the kdtree
    if (kdtree) {
        kd_clear (kdtree);
        kd_free (kdtree);
    }
    if (kdtree2) {
        kd_clear (kdtree2);
        kd_free (kdtree2);
    }
    kdtree = kd_create (system->getNumDimensions());
    kdtree2 = kd_create (system->getNumDimensions());
    // Initialize the variables
    numDimensions = system->getNumDimensions();
    root = rootBackup;
    root2= rootBackup2;
    if (root){
        listVertices.push_back(root);
        insertIntoKdtree (*root,1);
        numVertices++;
    }
    if (root2){
        listVertices2.push_back(root2);
        insertIntoKdtree (*root2,2);
        numVertices++;
    }
    lowerBoundCost = DBL_MAX;
    lowerBoundVertex = NULL;
    
    return 1;
}


template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::setGamma (double gammaIn) {
    
    if (gammaIn < 0.0)
        return 0;
    
    gamma = gammaIn;
    
    return 1;
}




template <class State,class Trajectory, class System>
int compareVertexCostPairs (std::pair<RRTstar::Vertex<State,Trajectory,System>*,double> i, std::pair<RRTstar::Vertex<State,Trajectory,System>*,double> j) {
    
    return (i.second < j.second);
}


template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::findBestParent (State& stateIn, std::vector< Vertex<State,Trajectory,System>* >& vectorNearVerticesIn, Vertex<State,Trajectory,System>*& vertexBest, Trajectory& trajectoryOut, bool& exactConnection) {
    
    
    // Compute the cost of extension for each near vertex
    int numNearVertices = vectorNearVerticesIn.size();

    std::vector< std::pair<Vertex<State,Trajectory,System>*,double> > vectorVertexCostPairs(numNearVertices);
    
    int i = 0;
    for (typename std::vector< Vertex<State,Trajectory,System>* >::iterator iter = vectorNearVerticesIn.begin(); iter != vectorNearVerticesIn.end(); iter++) {
        
        vectorVertexCostPairs[i].first = *iter;
        exactConnection = false;
        double trajCost = system->evaluateExtensionCost ( *((*iter)->state), stateIn, exactConnection);
        //if(trajCost>=0)
        vectorVertexCostPairs[i].second = (*iter)->costFromRoot + trajCost;
        i++;
    }

    // Sort vertices according to cost
    std::sort (vectorVertexCostPairs.begin(), vectorVertexCostPairs.end(), compareVertexCostPairs<State,Trajectory,System>);
    
    // Try out each extension according to increasing cost
    i = 0;
    bool connectionEstablished = false;
    for (typename std::vector< std::pair<Vertex<State,Trajectory,System>*,double> >::iterator iter = vectorVertexCostPairs.begin(); 
         iter != vectorVertexCostPairs.end(); iter++) {
        
        Vertex<State,Trajectory,System>* vertexCurr = iter->first;

        // Extend the current vertex towards stateIn (and this time check for collision with obstacles)
        exactConnection = false;
        if (system->extendTo(*(vertexCurr->state), stateIn, trajectoryOut, exactConnection) > 0) {
            vertexBest = vertexCurr;
            connectionEstablished = true;
            //cout<<"Yes"<<endl;
            break;

        }

    }

    // Return success if a connection was established
    if (connectionEstablished)
        return 1;

    // If the connection could not be established then return zero
    return 0;
}


template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::updateBranchCost (Vertex<State,Trajectory,System>& vertexIn, int depth ,int a) {
    
    
    // Update the cost for each children
    for (typename std::set< Vertex<State,Trajectory,System>* >::iterator iter = vertexIn.children.begin(); iter != vertexIn.children.end(); iter++) {
        
        Vertex<State,Trajectory,System>& vertex = **iter;
        
        vertex.costFromRoot = vertexIn.costFromRoot + vertex.costFromParent;

        checkUpdateBestVertex (vertex,a);
        
        updateBranchCost (vertex, depth + 1,a);
    }
    
    
    
    return 1;
}


template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::rewireVertices (Vertex<State,Trajectory,System>& vertexNew, std::vector< Vertex<State,Trajectory,System>* >& vectorNearVertices,int a) {
    

    // Repeat for all vertices in the set of near vertices
    for (typename std::vector< Vertex<State,Trajectory,System>* >::iterator iter = vectorNearVertices.begin(); iter != vectorNearVertices.end(); iter++) {
        
        Vertex<State,Trajectory,System>& vertexCurr = **iter; 
        
        // Check whether the extension results in an exact connection
        bool exactConnection = false;
        double costCurr = system->evaluateExtensionCost (*(vertexNew.state), *(vertexCurr.state), exactConnection);
        if ( (exactConnection == false) || (costCurr < 0) )
            continue;
        
        // Check whether the cost of the extension is smaller than current cost
        double totalCost = vertexNew.costFromRoot + costCurr;

        if (totalCost < vertexCurr.costFromRoot - 0.001) {
            
            // Compute the extension (checking for collision)
            Trajectory trajectory;
            
            if (system->extendTo (*(vertexNew.state), *(vertexCurr.state), trajectory, exactConnection) <= 0 ) 
                continue;
            
            // Insert the new trajectory to the tree by rewiring
            
            insertTrajectory (vertexNew, trajectory, vertexCurr,a);
            // Update the cost of all vertices in the rewired branch
            updateBranchCost (vertexCurr, 0,a);
        }
    }
    
    return 1;
}




template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::iteration () {
    
int chexk;
    
    // 1. Sample a new state

    State stateRandom;
    system->sampleState (stateRandom);

    	// 2. Compute the set of all near vertices
    std::vector< Vertex<State,Trajectory,System>* > vectorNearVertices;
    std::vector< Vertex<State,Trajectory,System>* > vectorNearVertices2;
    getNearVertices (stateRandom, vectorNearVertices,vectorNearVertices2);
    // 3. Find the best parent and extend from that parent
    Vertex<State,Trajectory,System>* vertexParent  = NULL;
    Vertex<State,Trajectory,System>* vertexParent2 = NULL;
    Trajectory trajectory,trajectory2;

    bool exactConnection = false;
    bool exactConnection2 = false;
    bool flagA = false;
    bool flagB = false;
    
    //tree a
    if (vectorNearVertices.size() == 0) {
        // 3.a Extend the nearest
        if (getNearestVertex (stateRandom, vertexParent,1) <= 0)
           {flagA=true;goto x;}       
        if (system->extendTo(vertexParent->getState(), stateRandom, trajectory, exactConnection) <= 0)
        	{flagA=true;goto x;}               
    }
    else
    {
        // 3.b Extend the best parent within the near vertices
        
        if (findBestParent (stateRandom, vectorNearVertices, vertexParent, trajectory, exactConnection) <= 0)         
            {flagA=true;goto x;}
    }
    // tree b
    x:
     if (vectorNearVertices2.size() == 0) {
        // 3.a Extend the nearest
        if (getNearestVertex (stateRandom, vertexParent2,2) <= 0)
        {flagB=true;goto y;}
                  
        if (system->extendTo(vertexParent2->getState(), stateRandom, trajectory2, exactConnection2) <= 0)
        {flagB=true;goto y;}	
                           
    }
    else
    {
        // 3.b Extend the best parent within the near vertices
        
        if (findBestParent (stateRandom, vectorNearVertices2, vertexParent2, trajectory2, exactConnection2) <= 0)         
            {flagB=true;goto y;}//return 0;
    }
    //
    y:
    if(flagA==true && flagB==true)
        return 0;
    else if(flagA)
    {
        //insert into treeB
        Vertex<State,Trajectory,System>* vertexNew = insertTrajectory (*vertexParent2, trajectory2,2);
         if (vertexNew == NULL) 
         return 0;
         if (vectorNearVertices2.size() > 0) 
        rewireVertices (*vertexNew, vectorNearVertices2,2);
    }
    else if (flagB)
    {
        //insert into tree A
         Vertex<State,Trajectory,System>* vertexNew = insertTrajectory (*vertexParent, trajectory,1);
         if (vertexNew == NULL) 
         return 0;
         if (vectorNearVertices.size() > 0) 
        rewireVertices (*vertexNew, vectorNearVertices,1);
        
    }
    else{
    double costA;
    double costB;
    int cA,cB;
    costA=vertexParent->costFromRoot + trajectory.evaluateCost();
    costB=vertexParent2->costFromRoot + trajectory2.evaluateCost();
    cA=(int)floor(costA);
    cB=(int)floor(costB);
    
    if(cA==cB)
    {
        Vertex<State,Trajectory,System>* vertexNew = insertTrajectory (*vertexParent, trajectory,1);
         if (vertexNew != NULL)
         if (vectorNearVertices.size() > 0) 
        rewireVertices (*vertexNew, vectorNearVertices,1);
        Vertex<State,Trajectory,System>* vertexNew2 = insertTrajectory (*vertexParent2, trajectory2,2);
         if (vertexNew2 != NULL)
         if (vectorNearVertices2.size() > 0) 
        rewireVertices (*vertexNew2, vectorNearVertices2,2);
        
        
         double costend=costA +costB;
         checkUpdateBestVertex2(*vertexNew,*vertexNew2,costend);
    }
         
    if(cA<=cB)
    {
        //insert into tree A
         Vertex<State,Trajectory,System>* vertexNew = insertTrajectory (*vertexParent, trajectory,1);
         if (vertexNew == NULL) 
         return 0;
         if (vectorNearVertices.size() > 0) 
        rewireVertices (*vertexNew, vectorNearVertices,1);
         double costend=costA +costB;
         checkUpdateBestVertex2(*vertexNew,*vertexParent2,costend);

    
    }
    else 
    {
        //insert into tree B
        Vertex<State,Trajectory,System>* vertexNew = insertTrajectory (*vertexParent2, trajectory2,2);
         if (vertexNew == NULL) 
         return 0;
         if (vectorNearVertices2.size() > 0) 
        rewireVertices (*vertexNew, vectorNearVertices2,2);
         double costend=costA +costB;
         checkUpdateBestVertex2(*vertexParent,*vertexNew,costend);

    }
    
    }
       
   
}


template<class State, class Trajectory, class System>
int 
RRTstar::Planner<State, Trajectory, System>
::getBestTrajectory (std::list<double*>& trajectoryOut, int a) {
    
    Vertex<State,Trajectory,System>* vertexCurr;
    Vertex<State,Trajectory,System>* vertexCurr2;
    if(a==1){
    if (lowerBoundVertex == NULL)
        return 0;
    
      vertexCurr = lowerBoundVertex;
    }
    else if(a==2){
    if (lowerBoundVertex2 == NULL)
        return 0;
    
      vertexCurr = lowerBoundVertex2;
    }
    else if (a==3)
    {
      if (lowerBoundVertexA == NULL || lowerBoundVertexB == NULL)
        return 0;
    
      vertexCurr = lowerBoundVertexA;
      vertexCurr2 = lowerBoundVertexB;
    
    }
    else
        return 0;
        
    
    while (vertexCurr) {
        
        State& stateCurr = vertexCurr->getState();
        
        double *stateArrCurr = new double[2]; 
        stateArrCurr[0] = stateCurr[0];
        stateArrCurr[1] = stateCurr[1];
        
        trajectoryOut.push_front (stateArrCurr);
        
        Vertex<State,Trajectory,System>& vertexParent = vertexCurr->getParent(); 
        
        if (&vertexParent != NULL) {
            
            State& stateParent = vertexParent.getState();
            
            std::list<double*> trajectory;
            system->getTrajectory (stateParent, stateCurr, trajectory);

            trajectory.reverse ();
            for (std::list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++) {
                
                double *stateArrFromParentCurr = *iter;
                
                stateArrCurr = new double[2];
                stateArrCurr[0] = stateArrFromParentCurr[0];
                stateArrCurr[1] = stateArrFromParentCurr[1];
                
                trajectoryOut.push_front (stateArrCurr);
                
                delete [] stateArrFromParentCurr;
            }
        }
        
        vertexCurr = &vertexParent;
    }
    if(a==3)
    {
        trajectoryOut.reverse();
         while (vertexCurr2) {
        
        State& stateCurr = vertexCurr2->getState();
        
        double *stateArrCurr = new double[2]; 
        stateArrCurr[0] = stateCurr[0];
        stateArrCurr[1] = stateCurr[1];
        
        trajectoryOut.push_front (stateArrCurr);
        
        Vertex<State,Trajectory,System>& vertexParent = vertexCurr2->getParent(); 
        
        if (&vertexParent != NULL) {
            
            State& stateParent = vertexParent.getState();
            
            std::list<double*> trajectory;
            system->getTrajectory (stateParent, stateCurr, trajectory);

            trajectory.reverse ();
            for (std::list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++) {
                
                double *stateArrFromParentCurr = *iter;
                
                stateArrCurr = new double[2];
                stateArrCurr[0] = stateArrFromParentCurr[0];
                stateArrCurr[1] = stateArrFromParentCurr[1];
                
                trajectoryOut.push_front (stateArrCurr);
                
                delete [] stateArrFromParentCurr;
            }
        }
        
        vertexCurr2 = &vertexParent;
    }
    }


    return 1;
}


#endif
