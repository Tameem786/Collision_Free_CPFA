#ifndef CPFA_LOOP_FUNCTIONS_H
#define CPFA_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <source/CPFA/CPFA_controller.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>

using namespace argos;
using namespace std;

static const size_t GENOME_SIZE = 7; // There are 7 parameters to evolve

struct QueueEntry {
    std::string robotId;
    int positionIndex;  // Index in waitingQueuePositions vector
    
    QueueEntry(const std::string& id, int index) 
        : robotId(id), positionIndex(index) {}
};

struct CircleEntry {
	bool isHit;
	int positionIndex;	

	CircleEntry(bool hit, int index)
		: isHit(hit), positionIndex(index) {}
};

class CPFA_loop_functions : public argos::CLoopFunctions
{

	friend class CPFA_controller;
	friend class CPFA_qt_user_functions;

	public:

		CPFA_loop_functions();
	   
		void Init(argos::TConfigurationNode &t_tree);
		void Reset();
		void PreStep();
		void PostStep();
		bool IsExperimentFinished();
		void PostExperiment();
		argos::CColor GetFloorColor(const argos::CVector2 &c_pos_on_floor);

		// GA Functions
		
		/* Configures the robot controller from the genome */
		void ConfigureFromGenome(Real* pf_genome);
		/* Calculates the performance of the robot in a trial */
		Real Score();
	
		/**
		 * Returns the current trial.
		 */
		UInt32 GetTrial() const;
	
		/**
		 * Sets the current trial.
		 * @param un_trial The trial number.
		 */
		void SetTrial(UInt32 un_trial);
	
		/* public helper functions */
		void UpdatePheromoneList();
		void SetFoodDistribution();
		void SetSpiralPathCoordinates();

		argos::Real getSimTimeInSeconds();

		std::vector<argos::CColor>   TargetRayColorList;
		std::vector<argos::CVector3> CircleCoordinates;
		std::vector<argos::CVector3> SpiralPathCoordinates;
		std::vector<argos::CVector2> SpiralPathCoordinatesForController;

		std::vector<argos::CVector2> FirstInnerCircleCoordinates;
		std::vector<argos::CVector2> SecondInnerCircleCoordinates;
		std::vector<argos::CVector2> ThirdInnerCircleCoordinates;
		std::vector<argos::CVector2> FourthInnerCircleCoordinates;
		std::vector<argos::CVector2> FifthInnerCircleCoordinates;
		std::vector<argos::CVector2> SixthInnerCircleCoordinates;

		argos::CVector3 entryPoint;
		std::vector<argos::CVector3> nest1EntryPoints;
		std::vector<argos::CVector3> nest2EntryPoints;
		std::vector<argos::CVector3> nest3EntryPoints;
		std::vector<argos::CVector3> nest4EntryPoints;

		std::vector<argos::CVector3> nest1ExitPoints;
		std::vector<argos::CVector3> nest2ExitPoints;
		std::vector<argos::CVector3> nest3ExitPoints;
		std::vector<argos::CVector3> nest4ExitPoints;

		std::vector<argos::CVector2> forbiddenAreaCoordinates;

		unsigned int getNumberOfRobots();
        void increaseNumDistributedFoodByOne();
		double getProbabilityOfSwitchingToSearching();
		double getProbabilityOfReturningToNest();
		double getUninformedSearchVariation();
		double getRateOfInformedSearchDecay();
		double getRateOfSiteFidelity();
		double getRateOfLayingPheromone();
		double getRateOfPheromoneDecay();

		bool IsNearExitPoint(const argos::CVector2& position);

		// Red circle (barrier) management functions
		bool IsNearRedCircle(const argos::CVector2& position);
		bool IsInsideRedCircle(const argos::CVector2& position);
		CircleEntry IsNearForbiddenArea(const argos::CVector2& position);
		argos::CVector2 GetClosestExitPoint(const argos::CVector2& robotPosition);
		argos::CVector2 GetClosestNest(argos::CVector2& robotPosition);
		std::vector<argos::CVector2> GetAllEntryPoints();
	
		// Queue management methods
		void AddRobotToPathQueue(const std::string& robotId);
		void RemoveRobotFromPathQueue(const std::string& robotId);
		int GetPathQueueSize();
		int GetMaxPathQueueSize();
		void SetMaxPathQueueSize(int maxSize);
		bool IsRobotInPathQueue(const std::string& robotId);
		void ClearPathQueue(); // For cleanup/reset

		void SetNestsPredefinedEntryPathCoordinates();
		void SetNestsPredefinedExitPathCoordinates();

		void AddRobotToPathQueueToNest(int nestIndex, const std::string& robotId);
		void RemoveRobotFromPathQueueToNest(int nestIndex, const std::string& robotId);
		bool IsPathAvailableToNest(int nestIndex);
		int GetAvailablePathToNest();

		// Queue management methods
		int AddToQueue(const std::string& robotId);
		void RemoveFromQueue(const std::string& robotId);
		int GetCurrentQueueLength();
		argos::CVector2 GetUpdatedQueuePosition(const std::string& robotId);
		void UpdateQueuePositions();
		int FindRobotInQueue(const std::string& robotId);
		int GetNextAvailableQueuePosition();
		void PrintQueueStatus();
		bool IsQueueFull();
		bool IsQueueEmpty();
		std::vector<std::string> GetQueuedRobotIds();
		void ResetQueue();
		bool ShouldRobotMoveForward(const std::string& robotId, const argos::CVector2& currentPos);
		bool IsFirstInQueue(const std::string& robotId);

		CircleEntry IsNearFirstInnerCircle(const argos::CVector2& position);
		CircleEntry IsNearSecondInnerCircle(const argos::CVector2& position);
		CircleEntry IsNearThirdInnerCircle(const argos::CVector2& position);
		CircleEntry IsNearFourthInnerCircle(const argos::CVector2& position);
		CircleEntry IsNearFifthInnerCircle(const argos::CVector2& position);
		
	protected:

		void setScore(double s);

		std::vector<std::string> pathQueue;  // Queue of robot IDs currently using the predefined path
    	int maxPathQueueSize;                // Maximum number of robots allowed in path simultaneously
		

		std::vector<std::string> pathQueueToNest1;
		std::vector<std::string> pathQueueToNest2;
		std::vector<std::string> pathQueueToNest3;
		std::vector<std::string> pathQueueToNest4;

		std::vector<bool> pathAvailableToNest;

		std::vector<bool> waitingQueueForNest;
		std::vector<CVector2> waitingQueuePositions;
		std::vector<bool> waitingQueuePositionsAvailability;
		int lastWaitingQueueIndexForNest;
		
		Real RedCircleRadiusMultiplier;
		
		argos::CRandom::CRNG* RNG;
        size_t NumDistributedFood;
		size_t MaxSimTime;
		size_t ResourceDensityDelay;
		size_t RandomSeed;
		size_t SimCounter;
		size_t MaxSimCounter;
		size_t VariableFoodPlacement;
		size_t OutputData;
		size_t DrawDensityRate;
		size_t DrawIDs;
		size_t DrawTrails;
		size_t DrawTargetRays;
		size_t FoodDistribution;
		size_t FoodItemCount;
		size_t PowerlawFoodUnitCount;
		size_t NumberOfClusters;
		size_t ClusterWidthX;
		size_t ClusterWidthY;
		size_t PowerRank;
                size_t ArenaWidth;
                size_t SimTime; 
                Real curr_time_in_minutes; 
                Real last_time_in_minutes; 
  
		/* CPFA variables */
		argos::Real ProbabilityOfSwitchingToSearching;
		argos::Real ProbabilityOfReturningToNest;
		argos::CRadians UninformedSearchVariation;
		argos::Real RateOfInformedSearchDecay;
		argos::Real RateOfSiteFidelity;
		argos::Real RateOfLayingPheromone;
		argos::Real RateOfPheromoneDecay;

		/* physical robot & world variables */
		argos::Real FoodRadius;
		argos::Real FoodRadiusSquared;
		argos::Real NestRadius;
		argos::Real RedCircleRadius;
		argos::Real NestRadiusSquared;
		argos::Real NestElevation;
		argos::Real SearchRadiusSquared;
		argos::CVector2 RedCirclePosition;

		/* list variables for food & pheromones */
		std::vector<argos::CVector2> FoodList;
		std::vector<argos::CColor>   FoodColoringList;
		vector<argos::CVector2> CollectedFoodList;
                map<string, argos::CVector2> FidelityList; 
		std::vector<Pheromone>   PheromoneList; 
		std::vector<argos::CRay3>    TargetRayList;
		argos::CRange<argos::Real>   ForageRangeX;
		argos::CRange<argos::Real>   ForageRangeY;

		std::vector<QueueEntry> robotQueue;  // Ordered list of robots in queue
  
                Real   CollisionTime;
                size_t currCollisionTime; 
                size_t lastCollisionTime; 
                size_t lastNumCollectedFood;
                size_t currNumCollectedFood;
                size_t Num_robots;
      
                vector<size_t>		ForageList;
				argos::CVector2 NestPosition;
				std::vector<argos::CVector2> NestPositions;
		
	private:

		/* private helper functions */
		void RandomFoodDistribution();
		void ClusterFoodDistribution();
		void PowerLawFoodDistribution();
                bool IsOutOfBounds(argos::CVector2 p, size_t length, size_t width);
		bool IsCollidingWithNest(argos::CVector2 p);
		bool IsCollidingWithFood(argos::CVector2 p);
		double score;
		int PrintFinalScore;

};

#endif /* CPFA_LOOP_FUNCTIONS_H */
