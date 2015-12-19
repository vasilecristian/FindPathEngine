

#include <vector>
#include <memory>
#include <array>
#include <map>
#include <iostream>
#include <functional>


class NavMeshBase
{
public:
	virtual int ComputeGoalDistanceEstimate(NavMeshBase* navMesh, int goalNode, int node) = 0;
	virtual int ComputeCost(NavMeshBase* navMesh, int node, int neighbor) = 0;
	virtual std::vector<int> GetNeighbors(NavMeshBase* navMesh, int node) = 0;
};



/** This is the template class used as interface.*/
class Node
{
public:
	int m_index;

	/** The constructor */
	Node(int index) : m_index(index), m_parent(nullptr), m_cost(-1), m_distToTarget(-1), m_f(-1)
	{
	}

	/** This is the parent node*/
	Node* m_parent;

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



class FindPathEngine
{
public:

	FindPathEngine(NavMeshBase* navMesh);

	/** This is a ticket used to descrige a find path request.*/
	struct Ticket
	{
		Ticket(int start, int goal);

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

		/** This is the target */
		int m_goal;

		/** This is the start location */
		int m_start;

		/** Cuttent node processed */
		Node* m_current;

		/** The status of the ticket */
		State m_state;

		/** How many steps this path required to be detected */
		int m_steps;

		/** The list with nodes that represent the detected path */
		std::vector<int> m_pathFound;

		std::map<int, Node*> m_openList;

		std::map<int, Node*> m_closedList;

	};

	/** Add a new request to determine a path */
	void AddTicket(Ticket* ticket);

	/** This function will run and process every Ticket. 
	* If a ticket is solved (aka the path was found, or there is not solution) just remove the 
	* ticket from the pending list.
	* @return true if pending list with tickets is empty.*/
	bool Update();

private:

	NavMeshBase* m_navMesh;

	
	/** Update a ticket (aka a search request). This will make another step forward 
	* to search the path.
	* @param ticket is the request processed
	* @return true if the job is finished. Return false if the job need to be processed also at the next step.*/
	bool ProcessTicket(Ticket* ticket);

	std::vector<Ticket*> m_tickets;
};
