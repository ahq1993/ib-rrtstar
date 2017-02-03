/*! 
 * \file rrts.h 
 */ 

#ifndef __RRTS_H_
#define __RRTS_H_


#include "kdtree.h"

#include <list>
#include <set>
#include <vector>



namespace RRTstar {


    template<class State, class Trajectory, class System>
    class Planner;


    /*!
     * \brief RRT* Vertex class
     *
     * More elaborate description
     */
    template<class State, class Trajectory, class System>
    class Vertex {
            public:
        Vertex *parent;
        State *state;
        std::set<Vertex*> children;
        double costFromParent;
        double costFromRoot;
        Trajectory *trajFromParent;
    

    public:
    
        /*!
         * \brief Vertex constructor
         *
         * More elaborate description
         */
        Vertex ();

        /*!
         * \brief Vertex destructor
         *
         * More elaborate description
         */
        ~Vertex ();    
        
        /*!
         * \brief Vertex copy constructor
         *
         * More elaborate description
         * 
         * \param vertexIn A reference to the vertex to be copied.
         *
         */
        Vertex (const Vertex &vertexIn);

        /*!
         * \brief Returns a reference to the state
         *
         * More elaborate description
         */
        State& getState () {return *state;}
        
        /*!
         * \brief Returns a reference to the state (constant)
         *
         * More elaborate description
         */
        State& getState () const {return *state;}
        
        /*!
         * \brief Returns a reference to the parent vertex
         *
         * More elaborate description
         */
        Vertex& getParent () {return *parent;}
        
        /*!
         * \brief Returns the accumulated cost at this vertex
         *
         * More elaborate description
         */
        double getCost () {return costFromRoot;}
    
        friend class Planner<State,Trajectory,System>;
    };

    
    /*!
     * \brief RRT* Planner class
     *
     * More elaborate description
     */
    template<class State, class Trajectory, class System>
    class Planner {


        typedef struct kdtree KdTree;
        typedef struct kdres KdRes;
        typedef Vertex<State,Trajectory,System> vertex_t;
    
        int numDimensions;
        
        
        double gamma;

        double endtoend_Cost;
        double lowerBoundCost;
        double lowerBoundCost2;
        vertex_t *lowerBoundVertex;
        vertex_t *lowerBoundVertex2;
        vertex_t *lowerBoundVertexA;
        vertex_t *lowerBoundVertexB;
        KdTree *kdtree;
        KdTree *kdtree2;
        
        vertex_t *root;
        vertex_t *root2;
        
        int insertIntoKdtree (vertex_t &vertexIn,int a);
        
        int getNearestVertex (State& stateIn, vertex_t*& vertexPointerOut,int a);
        int getNearestVertex2 (State& stateIn, vertex_t*& vertexPointerOut,int a);
        int getNearVertices (State& stateIn, std::vector<vertex_t*>& vectorNearVerticesOut, std::vector<vertex_t*>&vectorNearVerticesOut2);
        
        int checkUpdateBestVertex (vertex_t& vertexIn,int a);
        int checkUpdateBestVertex2(vertex_t& vertexIn_T1,vertex_t& vertexIn_T2, double curr_endtoend_cost);
 
        vertex_t* insertTrajectory (vertex_t& vertexStartIn, Trajectory& trajectoryIn,int a);    
        int insertTrajectory (vertex_t& vertexStartIn, Trajectory& trajectoryIn, vertex_t& vertexEndIn,int a);
        
        int findBestParent (State& stateIn, std::vector<vertex_t*>& vectorNearVerticesIn,
                            vertex_t*& vertexBestOut, Trajectory& trajectoryOut, bool& exactConnection);
    
        int updateBranchCost (vertex_t& vertexIn, int depth,int a);    
        int rewireVertices (vertex_t& vertexNew, std::vector<vertex_t*>& vectorNearVertices,int a); 
        

    
    public:
    
        /*!
         * \brief A list of all the vertices
         *
         * More elaborate description
         */
        std::list<vertex_t*> listVertices;
        std::list<vertex_t*> listVertices2;
      
        
        /*!
         * \brief Number of vertices in the list
         *
         * More elaborate description
         */
        int numVertices;
        int numVertices2;
        
        /*!
         * \brief A pointer to the system class
         *
         * More elaborate description
         */
        System *system;
    
        
        /*!
         * \brief Planner constructor
         *
         * More elaborate description
         */
        Planner ();
        
        
        /*!
         * \brief Planner destructor
         *
         * More elaborate description
         */
        ~Planner ();

        
        /*!
         * \brief Sets the gamma constant of the RRT*
         *
         * More elaborate description
         *
         * \param gammaIn The new value of the gamma parameter 
         *
         */
        int setGamma (double gammaIn);
        
        
        /*!
         * \brief Sets the dynamical system used in the RRT* trajectory generation
         *
         * More elaborate description
         *
         * \param system A reference to the new dynamical system
         *
         */
        int setSystem (System& system);
        
        
        /*!
         * \brief Returns a reference to the root vertex
         *
         * More elaborate description
         */
        vertex_t& getRootVertex ();
        vertex_t& getRootVertex2 ();
        
        
        /*!
         * \brief Initializes the RRT* algorithm
         *
         * More elaborate description
         */
        int initialize ();

        /*!
         * \brief Executes one iteration of the RRT* algorithm
         *
         * More elaborate description
         */
        int iteration ();
        
        /*!
         * \brief Returns the cost of the best vertex in the RRT*
         *
         * More elaborate description
         */
        double getBestVertexCost () {return lowerBoundCost;}
        double getBestVertexCost2 () {return lowerBoundCost2;}
       double getBestVertexCostAB () {return endtoend_Cost;} 
        /*!
         * \brief Returns a reference to the best vertex in the RRT*
         *
         * More elaborate description
         */
        vertex_t& getBestVertex () {return *lowerBoundVertex;}
        vertex_t& getBestVertex2 () {return *lowerBoundVertex2;}
        vertex_t& getBestVertexA () {return *lowerBoundVertexA;}
        vertex_t& getBestVertexB () {return *lowerBoundVertexB;}
        
        /*!
         * \brief Returns the best trajectory as a list of double arrays
         *
         * More elaborate description
         *
         * \param trajectory The trajectory that contains the best trajectory as a 
         *                   list of double arrays of dimension system->getNumDimensions()
         *                   
         *
         */
        int getBestTrajectory (std::list<double*>& trajectory,int a);
       
    };

}

#endif
