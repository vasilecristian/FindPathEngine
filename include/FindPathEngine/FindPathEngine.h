#ifndef FINDPATHENGINE_H
#define FINDPATHENGINE_H

#include <vector>
#include <memory>
#include <array>
#include <map>
#include <iostream>
#include <functional>
#include <atomic>
#include <mutex>


/// forward declaration for ThreadPool
namespace tp
{
	class ThreadPool;
}

namespace fpe
{
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
		virtual int ComputeGoalDistanceEstimate(unsigned int goalIndex, unsigned int nodeIndex) = 0;

		/**  Must be implemented in derived class!
		* This function will calculate the movement cost from a node to a neighbor node.
		* @param nodeIndex is the node index .
		* @param neighborIndex is the node index .
		* @return the cost to move from node to neighbor.*/
		virtual int ComputeCost(unsigned int nodeIndex, unsigned int neighborIndex) = 0;

		/** Must be implemented in derived class!
		* Use this to get all the valid heighbod nodes. A valid node
		* is considered a node that can be used in the path => do not return the
		* nodes which have collision.
		* @param nodeIndex is the node that you want to get the neighbors for.
		* @return a list with neighbor nodes Indexes.*/
		virtual std::vector<unsigned int> GetNeighbors(unsigned int nodeIndex) = 0;
	};

	/** Forward declaration. See bellow the real class.*/
	class Ticket;
	class Node;


	/** This is the Main class that implemnts the generic A * (A star) search algorithm.
	* How to use it:
	* // ------------------
	* std::shared_ptr<NavMesh> navmesh = std::make_shared<NavMesh>();
	*
	* std::shared_ptr<fpe::FindPathEngine> engine = std::make_shared<fpe::FindPathEngine>(navmesh, // the navmesh pointer 
	*																					  6);      // threads in thread pool
	*
	* std::shared_ptr<fpe::Ticket> ticket = std::make_shared<fpe::Ticket>(NavMesh::GetIndex(1, 1), //start 
	*																	  NavMesh::GetIndex(6, 6), //goal
	*																	  true); //run async
	*
	* engine.AddTicket(ticket);
	*
	* while (!engine.Update()) // Call Update each frame.
	* {
	*   /// this loop will stop when the path was found,
	*   /// or when the path cannot be determined.
	* }
	* // ------------------*/
	class FindPathEngine : public std::enable_shared_from_this<FindPathEngine>
	{
	public:

		/** The constructor.
		* @param navMesh is a pointer to the user's navmesh class.
		* @param is the number of threads used to calculate the paths*/
        FindPathEngine(std::weak_ptr<NavMeshBase> navMesh, unsigned int threadsCount);

        /** The destructor will call Finish() for you, and will delete the m_threadsPool.*/
		~FindPathEngine();

        /** Use this function to signal all jobs in progress to finish. This function will
        * send the stop signal and will call Update() once again for the paths that are
        * calculated on the main thread.*/
        void Finish();

		/** Add a new request to determine a path */
		void AddTicket(std::shared_ptr<Ticket> ticket);

		/** This function will run and process every Ticket. If a ticket is supposed to run async,
		* a job will be posted and a thread from the threads pool will process the ticket. When the 
		* ticket is solved, will be removed from the list.
		* If a ticket is not async, will be processed on the same thread (with Update function), but
		* the Update will not block the thread. Each time (aka each frame) the Update function will 
		* process only one step from the search algorithm.
		* If a ticket is solved (aka the path was found, or there is not solution) just remove the
		* ticket from the pending list.
		* @return true if pending list with tickets is empty.*/
		bool Update();

	private:

		/** Is a pointer to the used's nav mesh. */
        std::weak_ptr<NavMeshBase> m_navMesh;


		/** Update a ticket (aka a search request). This will make another step forward
		* to search the path.
		* @param weakTicket is the request processed
		* @return true if the job is finished. Return false if the job need to be processed also at the next step.*/
        bool ProcessTicket(std::weak_ptr<Ticket> weakTicket);

		/** This function is used as a job for the threads pool.
		* This function is blockant, so the thread that will process this,
		* will end when the path is calculated.
		* @param ticket is the request processed*/
        void ProcessTicketAsync(std::weak_ptr<Ticket> ticket);

		/** Is a list with tickets that must be processed. */
		std::vector<std::shared_ptr<Ticket> > m_tickets;

        /** Mutex to protect the access to m_tickets*/
        std::recursive_mutex m_ticketsMutex;

        /** This is the number of threads that will be used to calculate the paths. 
        * Each thread will calculate a path at a time.*/
		unsigned int m_threadsCount;

		/** This is the Thread Pool.*/
		tp::ThreadPool* m_threadsPool;
	};


	/** This is the helper class used internally to
	* store node's props*/
	class Node
	{
		friend class FindPathEngine;
	public:

		/** The constructor
		* @param index is the index of the node.*/
		Node(unsigned int index)
			: m_index(index)
			, m_parent(nullptr)
			, m_cost(-1)
			, m_distToTarget(-1)
			, m_f(-1)
		{
		}

		~Node()
		{
		}

	private:
		/** is the index of the node.*/
		unsigned int m_index;

		/** This is the parent node*/
		std::shared_ptr<Node> m_parent;

		/** This is the cost to move to this tile. Is called also "G" value. This
		* must be a sum of parent cost and the cost to move to this node starting from parent.
		* By default this is -1 which means that this was not calculated yet.*/
		int m_cost;//"G"


		/** This the distance to target. It is calculated using a heuristic. It is called also the "H" value.
		* The function ComputeGoalDistanceEstimate will compute it.*/
		int m_distToTarget;//"H"

		/** This is "F" a sum of "G" and "H" */
		int m_f; // "G" + "H"

		/** The list with neighbors */
		std::vector<unsigned int> m_neighbors;
	};


	/** This is a ticket used to describe a find path request.*/
	class Ticket
	{
		friend class FindPathEngine;
	public:
		/** The constructor.
		* @param startIndex is the start node.
		* @param goalIndex is the target node 
		* @param runAsync if is true the path process will run on a separate thread.*/
		Ticket(unsigned int startIndex, unsigned int goalIndex, bool runAsync);

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
		std::vector<unsigned int>& GetFoundPath();

		/** Getter for the goal node */
		unsigned int GetGoalIndex(){ return m_goalIndex; }

		/** Getter for the start node */
		unsigned int GetStartIndex(){ return m_startIndex; }

		/** Getter for the open list */
		std::map<unsigned int, std::shared_ptr<Node> > GetOpenList();

		/** Getter for the closed list */
		std::map<unsigned int, std::shared_ptr<Node> > GetClosedList();

		/** Use this function to stop the process of path finding.*/
		void Stop();

	private:

		/** This is the target */
		std::atomic<unsigned int> m_goalIndex;

		/** This is the start location */
		std::atomic<unsigned int> m_startIndex;

		/** Cuttent node processed */
		std::shared_ptr<Node> m_current;

		/** The status of the ticket */
		std::atomic<State> m_state;

		/** How many steps this path required to be detected */
		std::atomic<int> m_steps;

		/** The list with nodes that represent the detected path */
		std::vector<unsigned int> m_pathFound;

		/** Protect the m_pathFound for multithread access */
		std::mutex m_pathFoundMutex;

		/** Is the list with possible/available nodes to check.*/
		std::map<unsigned int, std::shared_ptr<Node> > m_openList;

		std::mutex m_openListMutex;

		/** Is the list with nodes already checked. */
		std::map<unsigned int, std::shared_ptr<Node> > m_closedList;

		std::mutex m_closedListMutex;

		/** This will be checked in the ProcessTicket function. If is true, the 
		* path finding process will be stopped.*/
		std::atomic<bool> m_mustStop;

		std::atomic<bool> m_runAsync;

		std::atomic<bool> m_runAsyncQueued;
	};

} // namespace fpe

#endif //FINDPATHENGINE_H
