
#include "FindPathEngine/FindPathEngine.h"

#include "ThreadPool/ThreadPool.h"


namespace fpe
{
	FindPathEngine::FindPathEngine(std::shared_ptr<NavMeshBase> navMesh, unsigned int threadsCount)
		: m_navMesh(navMesh)
		, m_threadsCount(threadsCount)
		, m_threadsPool(nullptr)
	{
		if (m_threadsCount > 0)
			m_threadsPool = new tp::ThreadPool(m_threadsCount);
	}

	FindPathEngine::~FindPathEngine()
	{
		for (auto& ticket : m_tickets)
		{
			ticket->Stop();
		}

		delete m_threadsPool;
	}


	Ticket::Ticket(int startIndex, int goalIndex, bool runAsync)
		: m_startIndex(startIndex)
		, m_goalIndex(goalIndex)
		, m_current(nullptr)
		, m_state(State::WAITING)
		, m_steps(0)
		, m_mustStop(false)
		, m_runAsync(runAsync)
		, m_runAsyncQueued(false)
	{
	}

	std::vector<int>& Ticket::GetFoundPath()
	{ 
		/// protect the m_pathFound for multithread access
		std::lock_guard<std::mutex> lock(m_pathFoundMutex);
		return m_pathFound;
	}

	std::map<int, std::shared_ptr<Node> > Ticket::GetOpenList()
	{ 
		/// protect the m_openList for multithread access
		std::lock_guard<std::mutex> lock(m_openListMutex);

		return m_openList; 
	}

	std::map<int, std::shared_ptr<Node> > Ticket::GetClosedList()
	{ 
		/// protect the m_closedList for multithread access
		std::lock_guard<std::mutex> lock(m_closedListMutex);

		return m_closedList; 
	}


	void Ticket::Stop()
	{
		m_mustStop = true;
	}




	/** Add a new request to determine a path */
	void FindPathEngine::AddTicket(std::shared_ptr<Ticket> ticket)
	{
		m_tickets.push_back(ticket);
	}

	/** This function will run and process every Ticket.
	* If a ticket is solved (aka the path was found, or there is not solution) just remove the
	* ticket from the pending list.
	* @return true if pending list with tickets is empty.*/
	bool FindPathEngine::Update()
	{
		/// Check the state of all tickets. 
		/// If the ticket was processed, just remove it from the list.
		for (auto it = m_tickets.begin(); it != m_tickets.end();)
		{
			/// Check if the thread pool is ok. The thread pool can be null
			/// because the threadsCount parameter in the constructor is 0!
			if ((m_threadsPool == nullptr) && ((*it)->m_runAsync))
			{
				/// In this case if a job is supposed to run async, will be executed on 
				/// the same thread (with Update function). So, change the runAsync flag to false.
				(*it)->m_runAsync = false;
			}

			if ((*it)->m_runAsync)
			{
				/// If the Ticket must be processed async (aka on a separate thread)

				if (!(*it)->m_runAsyncQueued)
				{
					/// If the ticket was not already started, Add a job to the thread pool.
					(*it)->m_runAsyncQueued = true;
					m_threadsPool->AddJob(std::bind(&FindPathEngine::ProcessTicketAsync, this->shared_from_this(), (*it)));
				}
				else
				{
					/// Check if the ticket was processed. 
					if (((*it)->m_state == Ticket::State::COMPLETED)
					 || ((*it)->m_state == Ticket::State::STOPPED))
					{
						/// Remove the ticket from the list.
						it = m_tickets.erase(it);
						continue;
					}
				}
			}

			/// If the ticket must be processed on the same thread with Update(),
			/// Just check what ProcessTicket function returns. 
			else if (ProcessTicket((*it)))
			{
				/// Remove the ticket from the list.
				it = m_tickets.erase(it);
				continue;
			}
			
			/// Go to next ticket
			++it;
		}

		return (m_tickets.size() == 0);
	}

	void FindPathEngine::ProcessTicketAsync(std::shared_ptr<Ticket> ticket)
	{
		while (!ProcessTicket(ticket))
		{
			/// Execute the ProcessTicket until true is returned.
		}
	}

	bool FindPathEngine::ProcessTicket(std::shared_ptr<Ticket> ticket)
	{
		ticket->m_state = Ticket::State::PROCESSING;

		ticket->m_steps++;

		/// Check if the process must be stopped due to external reasons...
		if (ticket->m_mustStop)
		{
			/// Search is stopped because the Ticket::Stop() was called.
			ticket->m_state = Ticket::State::STOPPED;
			return true;
		}

		/// Chekc if the m_start is the same with m_goalIndex
		if (ticket->m_startIndex == ticket->m_goalIndex)
		{
			/// protect the m_pathFound for multithread access
			std::lock_guard<std::mutex> lock(ticket->m_pathFoundMutex);

			ticket->m_pathFound.push_back(ticket->m_startIndex);

			/// Search is stopped because the goal si the same with start node
			ticket->m_state = Ticket::State::COMPLETED;
			return true;
		}

		/// protect the m_closedList for multithread access
		{
			std::lock_guard<std::mutex> lock(ticket->m_closedListMutex);

			/// This is the first step -> closed list is empty.
			/// Add the start node to the closed list.
			if (ticket->m_closedList.size() == 0)
			{
				std::shared_ptr<Node> start = std::make_shared<Node>(ticket->m_startIndex);

				/// In this case, we will add the start node.
				start->m_parent = nullptr;

				/// calculate the distance to target
				start->m_distToTarget = m_navMesh->ComputeGoalDistanceEstimate(ticket->m_goalIndex, ticket->m_startIndex);

				/// calculate the cost to travel from m_startIndex node to the neighbor note
				start->m_cost = 0;

				/// Calculate the "F" value
				start->m_f = start->m_distToTarget;

				/// Add the neighbor node to the open list
				ticket->m_closedList[ticket->m_startIndex] = start;

				/// Set the current node to be the start node
				ticket->m_current = start;
			}
		}


		/// If the start node doe not have valid neighbors, try to GetNeighbors and add them to the open list
		if (ticket->m_current->m_neighbors.size() == 0)
		{
			ticket->m_current->m_neighbors = m_navMesh->GetNeighbors(ticket->m_current->m_index);// ticket->m_current->GetNeighbors();
		}

		for (auto& neighbor : ticket->m_current->m_neighbors)
		{
			/// Check if the goal is one of the neighbors
			if (ticket->m_goalIndex == neighbor)
			{
				/// protect the m_pathFound for multithread access
				std::lock_guard<std::mutex> lock(ticket->m_pathFoundMutex);

				ticket->m_pathFound.push_back(ticket->m_goalIndex);

				std::shared_ptr<Node> node = ticket->m_current;
				do
				{
					ticket->m_pathFound.push_back(node->m_index);
					//std::cout << "result " << node << " " << node->m_index << " " << node->m_f << std::endl;
					node = node->m_parent;
				} while (node != nullptr);

				ticket->m_state = Ticket::State::COMPLETED;
				return true;
			}

			bool addedAlready = false;
			std::shared_ptr<Node> neigh = nullptr;

			/// protect the m_openList for multithread access
			std::lock_guard<std::mutex> lockOpenList(ticket->m_openListMutex);

			/// protect the m_closedList for multithread access
			std::lock_guard<std::mutex> lockClosedList(ticket->m_closedListMutex);

			/// check if is in open list
			if (ticket->m_openList.find(neighbor) != ticket->m_openList.end())
			{
				addedAlready = true;
				neigh = ticket->m_openList[neighbor];
			}
			else if (ticket->m_closedList.find(neighbor) != ticket->m_closedList.end())
			{
				addedAlready = true;
				neigh = ticket->m_closedList[neighbor];
			}

			if (!addedAlready)
			{
				neigh = std::make_shared<Node>(neighbor);
				neigh->m_parent = ticket->m_current;
			}



			/// calculate the distance to target
			neigh->m_distToTarget = m_navMesh->ComputeGoalDistanceEstimate(ticket->m_goalIndex, neighbor);

			/// calculate the cost to travel from m_startIndex node to the neighbor note
			neigh->m_cost = m_navMesh->ComputeCost(ticket->m_current->m_index, neighbor);

			/// Calculate the "F" value
			float f = neigh->m_distToTarget + neigh->m_cost;

			/// If the "F" have the default value it means that this node does not have added 
			/// to the open list before.
			if (!addedAlready)
			{
				neigh->m_f = f;

				/// Add the neighbor node to the open list
				ticket->m_openList[neighbor] = neigh;
			}
			else if (neigh->m_f > f)
			{
				neigh->m_f = f;
			}
		}

		/// protect the m_openList for multithread access
		{
			std::lock_guard<std::mutex> lock(ticket->m_openListMutex);

			/// Chekc if there are some nodes in Open list
			if (ticket->m_openList.size() == 0)
			{
				/// protect the m_pathFound for multithread access
				std::lock_guard<std::mutex> lock(ticket->m_pathFoundMutex);

				std::shared_ptr<Node> node = ticket->m_current;
				do
				{
					ticket->m_pathFound.push_back(node->m_index);
					//std::cout << "result " << node << " " << node->m_index << " " << node->m_f << std::endl;
					node = node->m_parent;
				} while (node != nullptr);

				/// If no nodes -> search is stopped due to no path to goal.
				ticket->m_state = Ticket::State::STOPPED;
				return true;
			}
		}


		/// protect the m_openList for multithread access
		{
			std::lock_guard<std::mutex> lock(ticket->m_openListMutex);

			/// Get the object with the minimal "F"
			float f = (float)INT_MAX;
			for (auto& on : ticket->m_openList)
			{
				if (on.second->m_f < f)
				{
					f = on.second->m_f;
					ticket->m_current = on.second;
				}
			}

		}

		
		/// protect the m_openList for multithread access
		{
			//int i = 0;
			//for (auto& on : ticket->m_openList)
			//{
			//	std::cout << i << " I" << on.first << " " << on.first % 8 << "x" << on.first / 8 << " F" << on.second->m_f << " G" << on.second->m_cost << " H" << on.second->m_distToTarget << std::endl;
			//	i++;
			//}
			//std::cout << "Current I" << ticket->m_current->m_index << " " << ticket->m_current->m_index % 8 << "x" << ticket->m_current->m_index / 8 << " F" << ticket->m_current->m_f << " G" << ticket->m_current->m_cost << " H" << ticket->m_current->m_distToTarget << std::endl;

			std::lock_guard<std::mutex> lock(ticket->m_openListMutex);
			ticket->m_openList.erase(ticket->m_current->m_index);
		}


		/// protect the m_closedList for multithread access
		{
			std::lock_guard<std::mutex> lock(ticket->m_closedListMutex);
			ticket->m_closedList[ticket->m_current->m_index] = ticket->m_current;
		}

		

		return false;
	}
} //namespace fpe