#ifndef CPFA_CONTROLLER_H
#define CPFA_CONTROLLER_H

#include <source/Base/BaseController.h>
#include <source/Base/Pheromone.h>
#include <source/CPFA/CPFA_loop_functions.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>

using namespace std;
using namespace argos;

static unsigned int num_targets_collected = 0;

class CPFA_loop_functions;

class CPFA_controller : public BaseController {

	public:

		CPFA_controller();

		// CCI_Controller inheritence functions
		void Init(argos::TConfigurationNode &node);
		void ControlStep();
		void Reset();

		bool IsHoldingFood();
		bool IsUsingSiteFidelity();
		bool IsInTheNest(CVector2 position);

		Real FoodDistanceTolerance;
		int selectedNestIndex;

		void SetLoopFunctions(CPFA_loop_functions* lf);
  
  size_t     GetSearchingTime();//qilu 09/26/2016
  size_t      GetTravelingTime();//qilu 09/26/2016
  string      GetStatus();//qilu 09/26/2016
  size_t      startTime;//qilu 09/26/2016
        

	private:
  string 			controllerID;//qilu 07/26/2016

		CPFA_loop_functions* LoopFunctions;
		argos::CRandom::CRNG* RNG;

		/* pheromone trail variables */
		std::vector<argos::CVector2> TrailToShare;
		std::vector<argos::CVector2> TrailToFollow;
		std::vector<argos::CRay3>    MyTrail;

		/* robot position variables */
		argos::CVector2 SiteFidelityPosition;
  bool			 updateFidelity; //qilu 09/07/2016
  
		vector<CRay3> myTrail;
		CColor        TrailColor;

		bool isCircumnavigatingRedCircle = false;
		argos::CVector2 circumnavigationFinalTarget;
		int circumnavigationDirection = 1; // 1 for clockwise, -1 for counterclockwise

		bool isInformed;
		bool isHoldingFood;
		bool isUsingSiteFidelity;
		bool isGivingUpSearch;
  
		size_t ResourceDensity;
		size_t MaxTrailSize;
		size_t SearchTime;//for informed search
  
  size_t           searchingTime; //qilu 09/26
  size_t           travelingTime;//qilu 09/26
        
  
		/* iAnt CPFA state variable */
		enum CPFA_state {
			DEPARTING = 0,
			SEARCHING = 1,
			RETURNING = 2,
			SURVEYING = 3,
			EXITING = 4,
			FOLLOWING_ENTRY_PATH = 5
		} CPFA_state;

		/* iAnt CPFA state functions */
		void CPFA();
		void Departing();
		void Searching();
		void Returning();
		void Surveying();
		void Exiting();
		void FollowingEntryPath();
		bool CanEnterPredefinedPath();
		void GeneratePredefinedPath();

		/* CPFA helper functions */
		void SetRandomSearchLocation(bool isExit);
		void SetHoldingFood();
		void SetLocalResourceDensity();
		void SetFidelityList(argos::CVector2 newFidelity);
		void SetFidelityList();
		bool SetTargetPheromone();

		 argos::CVector2 GetRedCircleAvoidanceTarget(const argos::CVector2& desiredTarget);
		bool IsPathThroughRedCircle(const argos::CVector2& start, const argos::CVector2& end);
		argos::CVector2 GetCircumnavigationTarget(const argos::CVector2& currentPos, const argos::CVector2& finalTarget);


		argos::Real GetExponentialDecay(argos::Real value, argos::Real time, argos::Real lambda);
		argos::Real GetBound(argos::Real value, argos::Real min, argos::Real max);
		argos::Real GetPoissonCDF(argos::Real k, argos::Real lambda);

		void UpdateTargetRayList();
  
		CVector2 previous_position;

		string results_path;
		string results_full_path;
		bool isUsingPheromone;

		unsigned int survey_count;
		/* Pointer to the LEDs actuator */
        CCI_LEDsActuator* m_pcLEDs;

		bool logReport;
		bool hasSetEntryPoint;
		bool isFollowingCircleBoundary;
		bool isHeadingToEntryPoint;
		bool hasStartedBoundaryFollow;
		bool isPausingOnBoundary;
		argos::CVector2 entryPoint;
		argos::Real redCircleRadius;

		// New helper functions for wall-following behavior
		void ImplementWallFollowing(argos::Real circleRadius, argos::CVector2 entryPoint);
		void HandleNestBehavior();
		
		// Optional: Add these member variables for better wall-following control
		bool isWallFollowing;
		argos::CVector2 lastWallPosition;
		argos::Real wallFollowingStartTime;

		bool isBouncing;
		enum SearchType {
			SITE_FIDELITY, PHEROMONE_TRAIL, RANDOM_SEARCH
		} nextSearchType;

		bool isFollowingPredefinedPath;
		bool isFollowingNestPredefinedEntryPath;
		bool isFollowingNestPredefinedExitPath;
		int predefinedPathIndex;
		int predefinedNestEntryPathIndex;
		int predefinedNestExitPathIndex;
		std::vector<argos::CVector2> predefinedPath;
		std::vector<argos::CVector3> predefinedNestEntryPath;
		std::vector<argos::CVector3> predefinedNestExitPath;
};

#endif /* CPFA_CONTROLLER_H */
