

#include <vector>
#include <memory>
#include <array>
#include <map>
#include <iostream>
#include <functional>

/** This is the interface that must be implemented by user.*/
class NavMeshBase
{
public:
	/** Must be implemented in derived class! 
	* This function will calculate the distance(aka the cost) to the goal.
	* Is the Heuristic, that will return the distance from nodeIndex to goalIndex.
	* @param goalIndex is the index of the node that represent the target.
	* @param nodeIndex is the index of the node that you want to calculate the distance.
	* @return the estimated distance from nodeIndex to goalIndex.*/
	virtual int ComputeGoalDistanceEstimate(int goalIndex, int nodeIndex) = 0;

	/**  Must be implemented in derived class! 
	* This function will calculate the movement cost from a node to a neighbor node.
	* @param nodeIndex is the node index .
	* @param neighborIndex is the node index .
	* @return the cost to move from node to neighbor.*/
	virtual int ComputeCost(int nodeIndex, int neighborIndex) = 0;
	
	/** Must be implemented in derived class! 
	* Use this to get all the valid heighbod nodes. A valid node
	* is considered a node that can be used in the path => do not return the
	* nodes which have collision. 
	* @param nodeIndex is the node that you want to get the neighbors for.
	* @return a list with neighbor nodes Indexes.*/
	virtual std::vector<int> GetNeighbors(int nodeIndex) = 0;
};


/** This is the Main class that implemnts the generic A * (A star) search algorithm.
* How to use it:
* // ------------------
* std::shared_ptr<NavMesh> navmesh = std::make_shared<NavMesh>();
*
* FindPathEngine engine(navmesh);
*
* std::shared_ptr<FindPathEngine::Ticket> ticket = std::make_shared<FindPathEngine::Ticket>(NavMesh::GetIndex(1, 1), NavMesh::GetIndex(6, 6));
*
* engine.AddTicket(ticket);
* 
* while (!engine.Update())
* {
*   /// this loop will stop when the path was found, 
*   /// or when the path cannot be determined.
* }
* // ------------------*/
class FindPathEngine
{
	//forward declaration
	class Node;

public:

	/** The constructor.
	* @param navMesh is a pointer to the user's navmesh class.*/
	FindPathEngine(std::shared_ptr<NavMeshBase> navMesh);

	/** This is a ticket used to describe a find path request.*/
	class Ticket
	{
		friend class FindPathEngine;
	public:
		/** The constructor.
		* @param startIndex is the start node.
		* @param goalIndex is the target node */
		Ticket(int startIndex, int goalIndex);

		/** Used to describe the possible states of the Ticket.*/
		enum class State : int
		{
			/** The ticket waiting to be processed.*/
			WAITING = 0,

			/** The ticket is processed...*/
			PROCESSING,

			/** The ticket was processed with success.*/
			COMPLETED,

			/** The ticket was processed but the process was stopped for some reason...*/
			STOPPED,
		};

		/** Getter for the state of Ticket .*/
		State GetState(){ return m_state; }

		/** Getter for the steps required to determine the path .*/
		int GetSteps(){ return m_steps; }

		/** Getter for the detected path. */
		std::vector<int>& GetFoundPath(){ return m_pathFound; }
		 
	private:

		/** This is the target */
		int m_goalIndex;

		/** This is the start location */
		int m_startIndex;

		/** Cuttent node processed */
		std::shared_ptr<Node> m_current;

		/** The status of the ticket */
		State m_state;

		/** How many steps this path required to be detected */
		int m_steps;

		/** The list with nodes that represent the detected path */
		std::vector<int> m_pathFound;

		/** Is the list with possible/available nodes to check.*/
		std::map<int, std::shared_ptr<Node> > m_openList;

		/** Is the list with nodes already checked. */
		std::map<int, std::shared_ptr<Node> > m_closedList;
	};

	/** Add a new request to determine a path */
	void AddTicket(std::shared_ptr<Ticket> ticket);

	/** This function will run and process every Ticket. 
	* If a ticket is solved (aka the path was found, or there is not solution) just remove the 
	* ticket from the pending list.
	* @return true if pending list with tickets is empty.*/
	bool Update();

private:


	/** This is the helper class used internally to
	* store node's props*/
	class Node
	{
	public:

		/** The constructor 
		* @param index is the index of the node.*/
		Node(int index) 
			: m_index(index)
			, m_parent(nullptr)
			, m_cost(-1)
			, m_distToTarget(-1)
			, m_f(-1)
		{
		}

		/** is the index of the node.*/
		int m_index;

		/** This is the parent node*/
		std::shared_ptr<Node> m_parent;

		/** This is the cost to move to this tile. Is called also "G" value. This
		* must be a sum of parent cost and the cost to move to this node starting from parent.
		* By default this is -1 which means that this was not calculated yet => the GetCost function
		* will calculate it.*/
		int m_cost;//"G"


		/** This the distance to target. It is calculated using a heuristic. It is called also the "H" value.
		* The function ComputeGoalDistanceEstimate will compute it.*/
		int m_distToTarget;//"H"

		/** This is a sum of "G" and "H" */
		int m_f; // "G" + "H"

		/** The list with neighbors */
		std::vector<int> m_neighbors;
	};


	/** Is a pointer to the used's nav mesh. */
	std::shared_ptr<NavMeshBase> m_navMesh;

	
	/** Update a ticket (aka a search request). This will make another step forward 
	* to search the path.
	* @param ticket is the request processed
	* @return true if the job is finished. Return false if the job need to be processed also at the next step.*/
	bool ProcessTicket(std::shared_ptr<Ticket> ticket);

	/** Is a list with tickets that must be processed. */
	std::vector<std::shared_ptr<Ticket> > m_tickets;
};
