// The code by Sertac Karaman (https://svn.csail.mit.edu/rrtstar) has been modified and adapted for the implementation of Inteliigent Bidirectional-RRT*


#define LIBBOT_PRESENT 0

#include <iostream>
#include <ctime>

#include <bot_core/bot_core.h>

#include <lcm/lcm.h>

#include <lcmtypes/lcmtypes.h>

#include "rrts.hpp"
#include "system_single_integrator.h"


using namespace RRTstar;
using namespace SingleIntegrator;

using namespace std;



typedef Planner<State,Trajectory,System> planner_t;
typedef Vertex<State,Trajectory,System> vertex_t;


int publishTree (lcm_t *lcm, planner_t& planner, System& system);
int publishTraj (lcm_t *lcm, planner_t& planner, System& system);
int publishEnvironment (lcm_t *lcm, region& regionOperating, region& regionGoal, list<region*>& obstacles);


int main () {
    
    
    planner_t rrts;
    
    cout << "RRTstar is alive" << endl;
    
    
    // Get lcm
    lcm_t *lcm = bot_lcm_get_global (NULL);
    
    
    // Create the dynamical system
    System system;
    
    // Three dimensional configuration space
    system.setNumDimensions (2);
    
    // Define the operating region
    system.regionOperating.setNumDimensions(2);
    system.regionOperating.center[0] = 0.0;
    system.regionOperating.center[1] = 0.0;
    system.regionOperating.center[2] = 0.0;
    system.regionOperating.size[0] = 200.0;
    system.regionOperating.size[1] = 200.0;
    system.regionOperating.size[2] = 0.0;
    
    // Define the goal region
    system.regionGoal.setNumDimensions(2);
    system.regionGoal.center[0] = 25.0;
    system.regionGoal.center[1] = -75.0;
    system.regionGoal.center[2] = 0.0;
    system.regionGoal.size[0] = 2.0;
    system.regionGoal.size[1] = 2.0;
    system.regionGoal.size[2] = 0.0;
    
    system.regionGoal2.setNumDimensions(2);
    system.regionGoal2.center[0] = -15.0;
    system.regionGoal2.center[1] = -60.0;
    system.regionGoal2.center[2] = 0.0;
    system.regionGoal2.size[0] = 2.0;
    system.regionGoal2.size[1] = 2.0;
    system.regionGoal2.size[2] = 0.0;
    
    // Define the obstacle region
    region *obstacle,*obstacle2,*obstacle3,*obstacle4,*obstacle5,*obstacle6,*obstacle7,*obstacle8,*obstacle9,*obstacle10,*obstacle11;
    
	 obstacle = new region;
	 obstacle2= new region;
	 obstacle3=new region;
	 obstacle4=new region;
	 obstacle5=new region;
	 obstacle6=new region;
	 obstacle7=new region;
	 obstacle8=new region;
	 obstacle9=new region;
	 obstacle10=new region;
	 obstacle11=new region;
  
    obstacle->setNumDimensions(2);
    obstacle->center[0] =31.0;
    obstacle->center[1] = -67.5;
    obstacle->center[2] = 0.0;
    obstacle->size[0] = 2.0;
    obstacle->size[1] = 27.0;
    obstacle->size[2] = 0.0;


    obstacle2->setNumDimensions(2);
    obstacle2->center[0] = 10.0;
    obstacle2->center[1] = -80.0;
    obstacle2->center[2] = 0.0;
    obstacle2->size[0] = 43.0;
    obstacle2->size[1] = 2.0; 
    obstacle2->size[2] = 0.0;
    
    
    obstacle3->setNumDimensions(2);
    obstacle3->center[0] = 10.0;
    obstacle3->center[1] = -55.0;
    obstacle3->center[2] = 0.0;
    obstacle3->size[0] = 43.0;
    obstacle3->size[1] = 2.0;
    obstacle3->size[2] = 0.0;
    
    
    obstacle4->setNumDimensions(2);
    obstacle4->center[0] = 20.0;
    obstacle4->center[1] = -65.0;
    obstacle4->center[2] = 0.0;
    obstacle4->size[0] = 3.0;
    obstacle4->size[1] = 10.0;
    obstacle4->size[2] = 0.0;
    
    obstacle5->setNumDimensions(2);
    obstacle5->center[0] = 20.0;
    obstacle5->center[1] = -75.0;
    obstacle5->center[2] = 0.0;
    obstacle5->size[0] = 3.0;
    obstacle5->size[1] = 10.0;
    obstacle5->size[2] = 0.0;
    
    obstacle6->setNumDimensions(2);
    obstacle6->center[0] = 10.0;
    obstacle6->center[1] = -60.0;
    obstacle6->center[2] = 0.0;
    obstacle6->size[0] = 3.0;
    obstacle6->size[1] = 10.0;
    obstacle6->size[2] = 0.0;
    
    obstacle7->setNumDimensions(2);
    obstacle7->center[0] = 10.0;
    obstacle7->center[1] = -70.0;
    obstacle7->center[2] = 0.0;
    obstacle7->size[0] = 3.0;
    obstacle7->size[1] = 10.0;
    obstacle7->size[2] = 0.0;
    
    obstacle8->setNumDimensions(2);
    obstacle8->center[0] = 0.0;
    obstacle8->center[1] = -65.0;
    obstacle8->center[2] = 0.0;
    obstacle8->size[0] = 3.0;
    obstacle8->size[1] = 10.0;
    obstacle8->size[2] = 0.0;
    
    obstacle9->setNumDimensions(2);
    obstacle9->center[0] = 0.0;
    obstacle9->center[1] = -75.0;
    obstacle9->center[2] = 0.0;
    obstacle9->size[0] = 3.0;
    obstacle9->size[1] = 10.0;
    obstacle9->size[2] = 0.0;
    
    obstacle10->setNumDimensions(2);
    obstacle10->center[0] = -10.0;
    obstacle10->center[1] = -60.0;
    obstacle10->center[2] = 0.0;
    obstacle10->size[0] = 3.0;
    obstacle10->size[1] = 10.0; 
    obstacle10->size[2] = 0.0;
 
    obstacle11->setNumDimensions(2);
    obstacle11->center[0] = -10.0;
    obstacle11->center[1] = -70.0;
    obstacle11->center[2] = 0.0;
    obstacle11->size[0] = 3.0;
    obstacle11->size[1] = 10.0;
    obstacle11->size[2] = 0.0;
    
    system.obstacles.push_front(obstacle);  
    system.obstacles.push_front(obstacle2);  
    system.obstacles.push_front(obstacle3);  
    system.obstacles.push_front(obstacle4);  
    system.obstacles.push_front(obstacle5);  
    system.obstacles.push_front(obstacle6);  
    system.obstacles.push_front(obstacle7);  
    system.obstacles.push_front(obstacle8);  
    system.obstacles.push_front(obstacle9);  
    system.obstacles.push_front(obstacle10); 
    system.obstacles.push_front(obstacle11); 

    
    publishEnvironment (lcm, system.regionOperating, system.regionGoal, system.obstacles);

    

    // Add the system to the planner
    rrts.setSystem (system);
    
    // Set up the root vertex
    vertex_t &root = rrts.getRootVertex(); //tree 1  
	 State &rootState = root.getState();    

    vertex_t &root2 = rrts.getRootVertex2(); //tree 2     
    State &rootState2 = root2.getState();

    rootState[0] = -15.0;
    rootState[1] = -60.0;
    rootState[2] = 0.0;
    
    rootState2[0] = 25.0;
    rootState2[1] = -75.0;
    rootState2[2] = 0.0;
    
    
    // Initialize the planner
    rrts.initialize ();
    
    // This parameter should be larger than 1.5 for asymptotic 
    //   optimality. Larger values will weigh on optimization 
    //   rather than exploration in the RRT* algorithm. Lower 
    //   values, such as 0.1, should recover the RRT.
    rrts.setGamma (1.0);

    
    
    clock_t start = clock();
    int a=2;
    // Run the algorithm for 10000 iteartions
    /*for (int i = 0; i < 10000; i++) {
        rrts.iteration (a);
        a=~a;
    }*/
    int i=0;
    while(i<1000000){
   
        rrts.iteration();
               
        i++;
    }
    cout<<"iterations "<<i<<endl;
    
    clock_t finish = clock();
    cout << "Time : " << ((double)(finish-start))/CLOCKS_PER_SEC << endl;

    
    publishTree (lcm, rrts, system);    
   publishTraj (lcm, rrts, system);
    
    return 1;
}



int publishEnvironment (lcm_t *lcm, region& regionOperating, region& regionGoal, list<region*>& obstacles) {
    
    // Publish the environment
    lcmtypes_environment_t *environment = (lcmtypes_environment_t*) malloc (sizeof(lcmtypes_environment_t));
    
    environment->operating.center[0] = regionOperating.center[0];
    environment->operating.center[1] = regionOperating.center[1];
    environment->operating.center[2] = regionOperating.center[2];
    environment->operating.size[0] = regionOperating.size[0];
    environment->operating.size[1] = regionOperating.size[1];
    environment->operating.size[2] = regionOperating.size[2];

    environment->goal.center[0] = regionGoal.center[0];
    environment->goal.center[1] = regionGoal.center[1];
    environment->goal.center[2] = regionGoal.center[2];
    environment->goal.size[0] = regionGoal.size[0];
    environment->goal.size[1] = regionGoal.size[1];
    environment->goal.size[2] = regionGoal.size[2];
    
    
    environment->num_obstacles = obstacles.size();
    
    if (environment->num_obstacles > 0) 
        environment->obstacles = (lcmtypes_region_3d_t *) malloc (sizeof(lcmtypes_region_3d_t));
    
    int idx_obstacles = 0;
    for (list<region*>::iterator iter = obstacles.begin(); iter != obstacles.end(); iter++){
        
        region* obstacleCurr = *iter;
        
        environment->obstacles[idx_obstacles].center[0] = obstacleCurr->center[0];
        environment->obstacles[idx_obstacles].center[1] = obstacleCurr->center[1];
        environment->obstacles[idx_obstacles].center[2] = obstacleCurr->center[2];
        environment->obstacles[idx_obstacles].size[0] = obstacleCurr->size[0];
        environment->obstacles[idx_obstacles].size[1] = obstacleCurr->size[1];
        environment->obstacles[idx_obstacles].size[2] = obstacleCurr->size[2];
        
        idx_obstacles++;
    }
   
    
    lcmtypes_environment_t_publish (lcm, "ENVIRONMENT", environment);
    
    return 1;
}


int publishTraj (lcm_t *lcm, planner_t& planner, System& system) {
    
    
    cout << "Publishing trajectory -- start" << endl;
    
    vertex_t& vertexBest = planner.getBestVertex();
    vertex_t& vertexBest2= planner.getBestVertex2();
    vertex_t& vertexBestA= planner.getBestVertexA();
    vertex_t& vertexBestB= planner.getBestVertexB();
    int a=0;
    double Cost1= planner.getBestVertexCost();
    double Cost2= planner.getBestVertexCost2();
    double Cost3= planner.getBestVertexCostAB();
    
    if(Cost3<Cost1 && Cost3<Cost2)
    {
       if (&vertexBestA == NULL || &vertexBestB == NULL) {
        cout << "No best vertex" << endl;
        return 0;
     }
       cout<<"Cost From root T1 &T2 "<<Cost3<<endl;
       a=3;
    }
    else if(Cost2<Cost1 && Cost2<Cost3)
    {
       if (&vertexBest2 == NULL) {
        cout << "No best vertex" << endl;
        return 0;
     }
       cout<<"Cost From root T2 "<<vertexBest2.costFromRoot<<endl;
       a=2; 
    }
    else if(Cost1<Cost2 && Cost1<Cost3)
    {
       if (&vertexBest == NULL) {
        cout << "No best vertex" << endl;
        return 0;
     }
       cout<<"Cost From root T1 "<<vertexBest.costFromRoot<<endl;
       a=1; 
    }
    
     
    
    list<double*> stateList;
    
    planner.getBestTrajectory (stateList,a);
    
    lcmtypes_trajectory_t *opttraj = (lcmtypes_trajectory_t *) malloc (sizeof (lcmtypes_trajectory_t));
    
    opttraj->num_states = stateList.size();
    opttraj->states = (lcmtypes_state_t *) malloc (opttraj->num_states * sizeof (lcmtypes_state_t));
    
    int stateIndex = 0;
    for (list<double*>::iterator iter = stateList.begin(); iter != stateList.end(); iter++) {
        
        double* stateRef = *iter;
        opttraj->states[stateIndex].x = stateRef[0];
        opttraj->states[stateIndex].y = stateRef[1];
        if (system.getNumDimensions() > 2)
            opttraj->states[stateIndex].z = stateRef[2];
        else
            opttraj->states[stateIndex].z = 0.0;
        
        delete [] stateRef;
        
        stateIndex++;
    }
    
    
    lcmtypes_trajectory_t_publish (lcm, "TRAJECTORY", opttraj);
    
    lcmtypes_trajectory_t_destroy (opttraj);
    
    cout << "Publishing trajectory -- end" << endl;
    
    
    
    return 1;
}

int publishTree (lcm_t *lcm, planner_t& planner, System& system) {
    
    
    cout << "Publishing the tree -- start" << endl;
    
    bool plot3d = (system.getNumDimensions() > 2);
    
    lcmtypes_graph_t *graph = (lcmtypes_graph_t *) malloc (sizeof (lcmtypes_graph_t));
    
    graph->num_vertices  = planner.numVertices + planner.numVertices2; 
    
    if (graph->num_vertices > 0 ) {    
        
        graph->vertices = (lcmtypes_vertex_t *) malloc (graph->num_vertices * sizeof(lcmtypes_vertex_t));
        int vertexIndex = 0;
        for (list<vertex_t*>::iterator iter = planner.listVertices.begin(); iter != planner.listVertices.end(); iter++) {
            
            
            vertex_t &vertexCurr = **iter;
            State &stateCurr = vertexCurr.getState ();
            
            graph->vertices[vertexIndex].state.x = stateCurr[0];
            graph->vertices[vertexIndex].state.y = stateCurr[1];
            if (plot3d) 
                graph->vertices[vertexIndex].state.z = stateCurr[2];
            else 
                graph->vertices[vertexIndex].state.z = 0.0;
            
            vertexIndex++;
            
        }
        for (list<vertex_t*>::iterator iter = planner.listVertices2.begin(); iter != planner.listVertices2.end(); iter++) {
            
            
            vertex_t &vertexCurr = **iter;
            State &stateCurr = vertexCurr.getState ();
            
            graph->vertices[vertexIndex].state.x = stateCurr[0];
            graph->vertices[vertexIndex].state.y = stateCurr[1];
            if (plot3d) 
                graph->vertices[vertexIndex].state.z = stateCurr[2];
            else 
                graph->vertices[vertexIndex].state.z = 0.0;
            
            vertexIndex++;
            
        }
        
        
    }
    else {
        graph->vertices = NULL;
        
    }
    
    if (graph->num_vertices > 1) {
        
        graph->num_edges = graph->num_vertices - 1;
        graph->edges = (lcmtypes_edge_t *) malloc (graph->num_edges * sizeof(lcmtypes_edge_t));
        int edgeIndex = 0;
        for (list<vertex_t*>::iterator iter = planner.listVertices.begin(); iter != planner.listVertices.end(); iter++) {
            
            vertex_t &vertexCurr = **iter;
            
            vertex_t &vertexParent = vertexCurr.getParent();
            
            if ( &vertexParent == NULL ) 
                continue;
            
            State &stateCurr = vertexCurr.getState ();
            State &stateParent = vertexParent.getState();
            
            
            graph->edges[edgeIndex].vertex_src.state.x = stateParent[0];
            graph->edges[edgeIndex].vertex_src.state.y = stateParent[1];
            if (plot3d)
                graph->edges[edgeIndex].vertex_src.state.z = stateParent[2];
            else 
                graph->edges[edgeIndex].vertex_src.state.z = 0.0;
            
            
            graph->edges[edgeIndex].vertex_dst.state.x = stateCurr[0];
            graph->edges[edgeIndex].vertex_dst.state.y = stateCurr[1];
            if (plot3d)
                graph->edges[edgeIndex].vertex_dst.state.z = stateCurr[2];
            else 
                graph->edges[edgeIndex].vertex_dst.state.z = 0.0;
            
            graph->edges[edgeIndex].trajectory.num_states = 0;
            graph->edges[edgeIndex].trajectory.states = NULL;
            
            edgeIndex++;
        }
        for (list<vertex_t*>::iterator iter = planner.listVertices2.begin(); iter != planner.listVertices2.end(); iter++) {
            
            vertex_t &vertexCurr = **iter;
            
            vertex_t &vertexParent = vertexCurr.getParent();
            
            if ( &vertexParent == NULL ) 
                continue;
            
            State &stateCurr = vertexCurr.getState ();
            State &stateParent = vertexParent.getState();
            
            
            graph->edges[edgeIndex].vertex_src.state.x = stateParent[0];
            graph->edges[edgeIndex].vertex_src.state.y = stateParent[1];
            if (plot3d)
                graph->edges[edgeIndex].vertex_src.state.z = stateParent[2];
            else 
                graph->edges[edgeIndex].vertex_src.state.z = 0.0;
            
            
            graph->edges[edgeIndex].vertex_dst.state.x = stateCurr[0];
            graph->edges[edgeIndex].vertex_dst.state.y = stateCurr[1];
            if (plot3d)
                graph->edges[edgeIndex].vertex_dst.state.z = stateCurr[2];
            else 
                graph->edges[edgeIndex].vertex_dst.state.z = 0.0;
            
            graph->edges[edgeIndex].trajectory.num_states = 0;
            graph->edges[edgeIndex].trajectory.states = NULL;
            
            edgeIndex++;
        }
        
    }
    else {
        graph->num_edges = 0;
        graph->edges = NULL;
        
         }
    
    lcmtypes_graph_t_publish (lcm, "GRAPH", graph);
    lcmtypes_graph_t_destroy (graph);
    cout << "Publishing the tree -- end" << endl;
    
    return 1;
}



