# FindPathEngine
This is a library that implements the A* (A star) search algorithm. Is based on C++11 features.

## Why I need it.
1. Because I need to learn how the A* star algorithm works. There are other similar libs/implementations, but when you want to learn something is better, also, to write the code. 
2. Because I need it in my tests.
 
This is still somewhat a work in progress; it's stable, but there are probably places where can be improved.

## Licensing 
Public Domain. If my licensing is wrong, please let me know. Use at your own risk for whatever you want. Apparently licensing is hard and complicated. If your country doesn't have a public domain, feel free to say you found this on the side of the road. 

## Requirements
`FindPathEngine` is based on:
- c++11 features (shared pointers, atomics, mutexes, functors)
- ThreadPool (https://github.com/vasilecristian/ThreadPool) which is derived from https://github.com/nbsdx/ThreadPool


## Overview
`FindPathEngine` is a simple class that manages tickets. When you need the path you just place a ticket, and check periodically to see if the ticket was processed. Prior to use the FindPathEngine you need to implement functions from abstract interface 'NavMeshBase'.

Example: 
```c++
/** This class will be a navmesh based on a rectangular map of tiles.*/
class NavMesh : public fpe::NavMeshBase
{
public:
	NavMesh();

	static int GetIndex(int x, int y);

	float ComputeGoalDistanceEstimate(int goalIndex, int nodeIndex) override;
	
	int ComputeCost(int nodeIndex, int neighborIndex) override;
	
	std::vector<int> GetNeighbors(int nodeIndex) override;
};


int main(int argc, char* argv[])
{
	/// Create the user's nav mesh.
	std::shared_ptr<NavMesh> navmesh = std::make_shared<NavMesh>();

	/// Create a shared pointer for the FindPathEngine. Must be a shared pointer!
	std::shared_ptr<fpe::FindPathEngine> engine = std::make_shared<fpe::FindPathEngine>(navmesh, // the pointer to nav mesh
																						6);      // how many threads will be in the pool

	/// Create a ticket.
	std::shared_ptr<fpe::Ticket> ticket = std::make_shared<fpe::Ticket>(NavMesh::GetIndex(1, 1), // start point
																		NavMesh::GetIndex(6, 6), // goal point
																		true);					 // run async

	/// Add the ticket to the FindPathEngine.
	engine->AddTicket(ticket);

	/// Run the main loop and at each frame , call the Update() function.
	while (!engine->Update())
	{

	}

	/// desplay the results.
	for (auto& nodeIndex : ticket->GetFoundPath())
	{
		std::cout << "result " << nodeIndex << " " << (nodeIndex % NavMesh::k_w) << "x" << (nodeIndex / NavMesh::k_w) << std::endl;
	}

	return 0;
}
```
