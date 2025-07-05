#include "CPFA_controller.h"
#include <unistd.h>

CPFA_controller::CPFA_controller() :
	RNG(argos::CRandom::CreateRNG("argos")),
	isInformed(false),
	isHoldingFood(false),
	isUsingSiteFidelity(false),
	isGivingUpSearch(false),
	ResourceDensity(0),
	MaxTrailSize(50),
	SearchTime(0),
	CPFA_state(DEPARTING),
	LoopFunctions(NULL),
	survey_count(0),
	isUsingPheromone(0),
    SiteFidelityPosition(1000, 1000), 
        searchingTime(0),
        travelingTime(0),
        startTime(0),
    m_pcLEDs(NULL),
        updateFidelity(false),
		logReport(true),
		hasSetEntryPoint(false),
		isHeadingToEntryPoint(false),
		isFollowingCircleBoundary(false),
		entryPoint(0.0, 0.0),
		redCircleRadius(0.0),
		hasStartedBoundaryFollow(false),
		isPausingOnBoundary(false),
		isBouncing(false),
		nextSearchType(RANDOM_SEARCH),
		selectedWaitingQueueIndex(-1),
		currentQueuePosition(-1),
		isInQueue(false),
		nextPosCheckingTried(0),
		isHitFirstInnerCircle(false),
		isReachedFirstInnerCircle(false),
		isReachedSecondInnerCircle(false),
		isReachedThirdInnerCircle(false),
		isReachedFourthInnerCircle(false),
		isReachedFifthInnerCircle(false),
		isReachedSixthInnerCircle(false),
		isFollowingNestPredefinedEntryPath(false),
		isFollowingNestPredefinedExitPath(false),
		isNearForbiddenArea(false),
		isEndingForbiddenArea(false),
		isFollowingPredefinedPathOnForbiddenArea(false),
		isFollowingAvoidance(false),
		isFollowingOuterCircle(false),
		isWaitingForNest(false),
		isWaitingForCollision(false),
		timeSet(false),
		timeInsideRedCircle(0.0),
		totalTimeInsideCirle(0.0)
{
	GoStraightAngleRangeInDegreesInRegion.Set(-30.0, 30.0);
	GoStraightAngleRangeInDegreesGoingToRegion.Set(-55.0, 55.0);
	// redCircleRadius = LoopFunctions->NestRadius * LoopFunctions->RedCircleRadiusMultiplier;
}

void CPFA_controller::Init(argos::TConfigurationNode &node) {
	compassSensor   = GetSensor<argos::CCI_PositioningSensor>("positioning");
	wheelActuator   = GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering");
	proximitySensor = GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity");
	argos::TConfigurationNode settings = argos::GetNode(node, "settings");

	argos::GetNodeAttribute(settings, "FoodDistanceTolerance",   FoodDistanceTolerance);
	argos::GetNodeAttribute(settings, "TargetDistanceTolerance", TargetDistanceTolerance);
	argos::GetNodeAttribute(settings, "NestDistanceTolerance", NestDistanceTolerance);
	argos::GetNodeAttribute(settings, "NestAngleTolerance",    NestAngleTolerance);
	argos::GetNodeAttribute(settings, "TargetAngleTolerance",    TargetAngleTolerance);
	argos::GetNodeAttribute(settings, "SearchStepSize",          SearchStepSize);
	argos::GetNodeAttribute(settings, "RobotForwardSpeed",       RobotForwardSpeed);
	argos::GetNodeAttribute(settings, "RobotRotationSpeed",      RobotRotationSpeed);
	argos::GetNodeAttribute(settings, "ResultsDirectoryPath",      results_path);
	argos::GetNodeAttribute(settings, "DestinationNoiseStdev",      DestinationNoiseStdev);
	argos::GetNodeAttribute(settings, "PositionNoiseStdev",      PositionNoiseStdev);

	argos::CVector2 p(GetPosition());
	SetStartPosition(argos::CVector3(p.GetX(), p.GetY(), 0.0));
	
	FoodDistanceTolerance *= FoodDistanceTolerance;
	SetIsHeadingToNest(true);
	//qilu 10/21/2016 Let robots start to search immediately
	SetTarget(p);
        controllerID= GetId();
    m_pcLEDs   = GetActuator<CCI_LEDsActuator>("leds");
    controllerID= GetId();//qilu 07/26/2016
		m_pcLEDs->SetAllColors(CColor::GREEN);

	// if(!LoopFunctions->SpiralPathCoordinates.empty()) {
	// 	LOG << "SpiralPathCoordinates size: " << LoopFunctions->SpiralPathCoordinates.size() << endl;
	// } else {
	// 	LOG << "SpiralPathCoordinates is empty" << endl;
	// }
}

void CPFA_controller::ControlStep() {
	
	// if(GetStatus() == "RETURNING" && LoopFunctions->IsNearRedCircle(GetPosition()) && isHoldingFood && !timeSet) {
	// 	LOG << "Timer start..." << std::endl;
	// 	timeInsideRedCircle = LoopFunctions->getSimTimeInSeconds();
	// 	timeSet = true;
	// }

	// if(timeSet && IsInTheNest(GetPosition())) {
	// 	LOG << "Time stop..." << std::endl;
	// 	LOG << GetId() << " took " << LoopFunctions->getSimTimeInSeconds() - timeInsideRedCircle << " seconds." << std::endl;
	// 	timeSet = false;
	// }

	// Add line so we can draw the trail

	CVector3 position3d(GetPosition().GetX(), GetPosition().GetY(), 0.00);
	CVector3 target3d(previous_position.GetX(), previous_position.GetY(), 0.00);
	CRay3 targetRay(target3d, position3d);
	myTrail.push_back(targetRay);
	LoopFunctions->TargetRayList.push_back(targetRay);
	LoopFunctions->TargetRayColorList.push_back(TrailColor);

	previous_position = GetPosition();

	//UpdateTargetRayList();
	CPFA();
	Move();

}

void CPFA_controller::Reset() {
	num_targets_collected =0;
	isHoldingFood   = false;
    isInformed      = false;
    SearchTime      = 0;
    ResourceDensity = 0;
    collisionDelay = 0;
    
  	LoopFunctions->CollisionTime=0; //qilu 09/26/2016
    
    
    /* Set LED color */
    /* m_pcLEDs->SetAllColors(CColor::BLACK); //qilu 09/04 */
	// Now we will set the target to a fixed nest position (0, 0.3)
    // SetTarget(); //qilu 09/08
	argos::CVector2 currentPos = GetPosition();

	// // Calculate direction to closest nest
	// argos::CVector2 directionToNest = (LoopFunctions->RedCirclePosition - currentPos).Normalize();

	// // Set target to a point slightly outside the red circle (safe distance)
	// argos::Real redCircleRadius = LoopFunctions->NestRadius * LoopFunctions->RedCircleRadiusMultiplier;
	// argos::Real safeDistance = redCircleRadius + 0.1;
	// argos::CVector2 targetOnCircle = LoopFunctions->RedCirclePosition - directionToNest * safeDistance;
	
	// SetTarget(targetOnCircle);
	// SetIsHeadingToNest(false);

    updateFidelity = false;
    TrailToShare.clear();
    TrailToFollow.clear();
    MyTrail.clear();

	myTrail.clear();

	isInformed = false;
	isHoldingFood = false;
	isUsingSiteFidelity = false;
	isGivingUpSearch = false;

	hasStartedBoundaryFollow = false;
	isFollowingCircleBoundary = false;
	isPausingOnBoundary = false;
	isHeadingToEntryPoint = false;
	isFollowingPredefinedPath = false;
	predefinedPathIndex = 0;

	
	
}

bool CPFA_controller::IsHoldingFood() {
		return isHoldingFood;
}

bool CPFA_controller::IsUsingSiteFidelity() {
		return isUsingSiteFidelity;
}

void CPFA_controller::CPFA() {
	
	switch(CPFA_state) {
		// depart from nest after food drop off or simulation start
		case DEPARTING:
			//SetIsHeadingToNest(false);
			Departing();
			break;
		// after departing(), once conditions are met, begin searching()
		case SEARCHING:
			//SetIsHeadingToNest(false);
			if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0) {
				Searching();
			}
			break;
		// return to nest after food pick up or giving up searching()
		case RETURNING:
			//SetIsHeadingToNest(true);
			Returning();
			break;
		case SURVEYING:
			//SetIsHeadingToNest(false);
			Surveying();
			break;
		case EXITING:
			Exiting();
			break;
		case FOLLOWING_ENTRY_PATH:
			FollowingEntryPath();
			break;
		case FOLLOWING_OUTER_CIRCLE:
			FollowingOuterCircle();
			break;
	}
}

bool CPFA_controller::IsInTheNest(CVector2 position) {

	for(const auto& nestPos : LoopFunctions->NestPositions) {
		if(nestPos == position) {
			if((GetPosition() - nestPos).SquareLength() < LoopFunctions->NestRadiusSquared) {
				return true;
			}
		}
	}
	return false;
}

void CPFA_controller::SetLoopFunctions(CPFA_loop_functions* lf) {
	LoopFunctions = lf;

	// Initialize the SiteFidelityPosition

	// Create the output file here because it needs LoopFunctions
		
	// Name the results file with the current time and date
	time_t t = time(0);   // get time now
	struct tm * now = localtime(&t);
	stringstream ss;

	char hostname[1024];                                                   
	hostname[1023] = '\0';                    
	gethostname(hostname, 1023);  

	/* ss << "CPFA-"<<GIT_BRANCH<<"-"<<GIT_COMMIT_HASH<<"-"
		<< hostname << '-'
		<< getpid() << '-'
		<< (now->tm_year) << '-'
		<< (now->tm_mon + 1) << '-'
		<<  now->tm_mday << '-'
		<<  now->tm_hour << '-'
		<<  now->tm_min << '-'
		<<  now->tm_sec << ".csv";

		string results_file_name = ss.str();
		results_full_path = results_path+"/"+results_file_name;
         */
         
	// Only the first robot should do this:	 
	if (GetId().compare("CPFA_0") == 0) {
		/*
		ofstream results_output_stream;
		results_output_stream.open(results_full_path, ios::app);
		results_output_stream << "NumberOfRobots, "
			<< "TargetDistanceTolerance, "
			<< "TargetAngleTolerance, "
			<< "FoodDistanceTolerance, "
			<< "RobotForwardSpeed, "
			<< "RobotRotationSpeed, "
			<< "RandomSeed, "
			<< "ProbabilityOfSwitchingToSearching, "
			<< "ProbabilityOfReturningToNest, "
			<< "UninformedSearchVariation, "   
			<< "RateOfInformedSearchDecay, "   
			<< "RateOfSiteFidelity, "          
			<< "RateOfLayingPheromone, "       
			<< "RateOfPheromoneDecay" << endl
			<< LoopFunctions->getNumberOfRobots() << ", "
			<< CSimulator::GetInstance().GetRandomSeed() << ", "  
			<< TargetDistanceTolerance << ", "
			<< TargetAngleTolerance << ", "
			<< FoodDistanceTolerance << ", "
			<< RobotForwardSpeed << ", "
			<< RobotRotationSpeed << ", "
			<< LoopFunctions->getProbabilityOfSwitchingToSearching() << ", "
			<< LoopFunctions->getProbabilityOfReturningToNest() << ", "
			<< LoopFunctions->getUninformedSearchVariation() << ", "
			<< LoopFunctions->getRateOfInformedSearchDecay() << ", "
			<< LoopFunctions->getRateOfSiteFidelity() << ", "
			<< LoopFunctions->getRateOfLayingPheromone() << ", "
			<< LoopFunctions->getRateOfPheromoneDecay()
			<< endl;
				
			results_output_stream.close();
		*/
	}

}

void CPFA_controller::Departing()
{
	
	argos::Real distanceToTarget = (GetPosition() - GetTarget()).Length();
	argos::Real randomNumber = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));

	// Calculate red circle radius if not already set
	if(redCircleRadius == 0.0) {
		redCircleRadius = LoopFunctions->RedCircleRadius;
	}

	// Update target if we're circumnavigating and need course correction
    if(isCircumnavigatingRedCircle && !isHoldingFood) {
        argos::CVector2 currentPos = GetPosition();
        argos::CVector2 updatedTarget = GetRedCircleAvoidanceTarget(circumnavigationFinalTarget);
        SetTarget(updatedTarget);
    }

	/* When not informed, continue to travel until randomly switching to the searching state. */
	if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0) {
		if(isInformed == false){
			if(SimulationTick()%(5*SimulationTicksPerSecond())==0 && randomNumber < LoopFunctions->ProbabilityOfSwitchingToSearching){
				Stop();
				SearchTime = 0;
				CPFA_state = SEARCHING;
				travelingTime+=SimulationTick()-startTime;
				startTime = SimulationTick();
			
				argos::Real USV = LoopFunctions->UninformedSearchVariation.GetValue();
				argos::Real rand = RNG->Gaussian(USV);
				argos::CRadians rotation(rand);
				argos::CRadians angle1(rotation.UnsignedNormalize());
				argos::CRadians angle2(GetHeading().UnsignedNormalize());
				argos::CRadians turn_angle(angle1 + angle2);
				argos::CVector2 turn_vector(SearchStepSize, turn_angle);
				SetIsHeadingToNest(false);
				
				// Ensure new search target is outside red circle
				argos::CVector2 newTarget = turn_vector + GetPosition();
				argos::Real distanceToCenter = (newTarget - LoopFunctions->RedCirclePosition).Length();
				if(distanceToCenter <= redCircleRadius) {
					// If new target is inside red circle, adjust it to be outside
					argos::CVector2 directionFromCenter = (newTarget - LoopFunctions->RedCirclePosition).Normalize();
					newTarget = LoopFunctions->RedCirclePosition + directionFromCenter * (redCircleRadius + 0.1);
				}
				SetTarget(newTarget);
			}
			else if(distanceToTarget < TargetDistanceTolerance){
				SetRandomSearchLocation(false);
			}
		}
	}
	
	/* Are we informed? I.E. using site fidelity or pheromones. */	
	if(isInformed && distanceToTarget < TargetDistanceTolerance) {
		if(isCircumnavigatingRedCircle) {
            // Continue circumnavigating
            argos::CVector2 nextTarget = GetRedCircleAvoidanceTarget(circumnavigationFinalTarget);
            SetTarget(nextTarget);
        } else {
			SearchTime = 0;
			CPFA_state = SEARCHING;
			travelingTime+=SimulationTick()-startTime;
			startTime = SimulationTick();

			if(isUsingSiteFidelity) {
				isUsingSiteFidelity = false;
				SetFidelityList();
			}
		}
	}
	

}

void CPFA_controller::Searching() {

    // "scan" for food only every half of a second
    if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0) {
        SetHoldingFood();
    }
    // When not carrying food, calculate movement.
    if(IsHoldingFood() == false) {
        argos::CVector2 distance = GetPosition() - GetTarget();
        argos::Real     random   = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
     
        // Calculate red circle radius if not already set
        if(redCircleRadius == 0.0) {
            redCircleRadius = LoopFunctions->RedCircleRadius;
        }

        // Check if current target is inside red circle
		// argos::Real distanceToCenter = (GetTarget() - LoopFunctions->RedCirclePosition).Length();
		// if(distanceToCenter <= redCircleRadius) {
		// 	// Target is inside red circle, set a new random search location outside
		// 	SetRandomSearchLocation(false);
		// 	return;
		// }
     
        // If we reached our target search location, set a new one. The 
        // new search location calculation is different based on whether
        // we are currently using informed or uninformed search.
        if(distance.SquareLength() < TargetDistanceTolerance) {
            // randomly give up searching
            if(SimulationTick()% (5*SimulationTicksPerSecond())==0 && random < LoopFunctions->ProbabilityOfReturningToNest) {
                // argos::LOG << GetId() << " Given up search. Heading to red circle." << std::endl;
                SetFidelityList();
                TrailToShare.clear();
                argos::CVector2 currentPos = GetPosition();
                argos::CVector2 directionToCenter = (LoopFunctions->RedCirclePosition - currentPos).Normalize();
                
                // Set target to a point slightly outside the red circle (safe distance)
                argos::Real safeDistance = redCircleRadius + 0.1;
                argos::CVector2 targetOnCircle = LoopFunctions->RedCirclePosition - directionToCenter * safeDistance;
                
                SetTarget(targetOnCircle);
                SetIsHeadingToNest(false);

                isGivingUpSearch = true;
                LoopFunctions->FidelityList.erase(controllerID);
                isUsingSiteFidelity = false; 
                updateFidelity = false; 
                CPFA_state = RETURNING;
                searchingTime+=SimulationTick()-startTime;
                startTime = SimulationTick();
                return; 
            }

            argos::Real USCV = LoopFunctions->UninformedSearchVariation.GetValue();
            argos::Real rand = RNG->Gaussian(USCV);

            // uninformed search
            if(isInformed == false) {
                argos::CRadians rotation(rand);
                argos::CRadians angle1(rotation);
                argos::CRadians angle2(GetHeading());
                argos::CRadians turn_angle(angle1 + angle2);
                argos::CVector2 turn_vector(SearchStepSize, turn_angle);
                SetIsHeadingToNest(false);
                
                // Ensure new search target is outside red circle
                argos::CVector2 newTarget = turn_vector + GetPosition();
                argos::Real distanceToCenter = (newTarget - LoopFunctions->RedCirclePosition).Length();
                if(distanceToCenter <= redCircleRadius) {
                    // If new target is inside red circle, adjust it to be outside
                    argos::CVector2 directionFromCenter = (newTarget - LoopFunctions->RedCirclePosition).Normalize();
                    newTarget = LoopFunctions->NestPosition + directionFromCenter * (redCircleRadius + 0.1);
                }
                SetTarget(newTarget);
            }
            // informed search
            else {
                SetIsHeadingToNest(false);

                if(IsAtTarget()) {
                    size_t t = SearchTime++;
                    argos::Real twoPi = (argos::CRadians::TWO_PI).GetValue();
                    argos::Real pi = (argos::CRadians::PI).GetValue();
                    argos::Real isd = LoopFunctions->RateOfInformedSearchDecay;
                    Real correlation = GetExponentialDecay(rand, t, isd);
                    argos::CRadians rotation(GetBound(correlation, -pi, pi));
                    argos::CRadians angle1(rotation);
                    argos::CRadians angle2(GetHeading());
                    argos::CRadians turn_angle(angle2 + angle1);
                    argos::CVector2 turn_vector(SearchStepSize, turn_angle);
                    
                    // Ensure new search target is outside red circle
                    argos::CVector2 newTarget = turn_vector + GetPosition();
                    argos::Real distanceToCenter = (newTarget - LoopFunctions->RedCirclePosition).Length();
                    if(distanceToCenter <= redCircleRadius) {
                        // If new target is inside red circle, adjust it to be outside
                        argos::CVector2 directionFromCenter = (newTarget - LoopFunctions->RedCirclePosition).Normalize();
                        newTarget = LoopFunctions->RedCirclePosition + directionFromCenter * (redCircleRadius + 0.1);
                    }
                    SetTarget(newTarget);
                }
            }
        }
    }
}

// Cause the robot to rotate in place as if surveying the surrounding targets
// Turns 36 times by 10 degrees
void CPFA_controller::Surveying() {
 //LOG<<"Surveying..."<<endl;
	if (survey_count <= 4) { 
		CRadians rotation(survey_count*3.14/2); // divide by 10 so the vecot is small and the linear motion is minimized
		argos::CVector2 turn_vector(SearchStepSize, rotation.SignedNormalize());
			
		SetIsHeadingToNest(true); // Turn off error for this
		SetTarget(turn_vector + GetPosition());
		/*
		ofstream log_output_stream;
		log_output_stream.open("log.txt", ios::app);
		log_output_stream << (GetHeading() - rotation ).SignedNormalize() << ", "  << SearchStepSize << ", "<< rotation << ", " <<  turn_vector << ", " << GetHeading() << ", " << survey_count << endl;
		log_output_stream.close();
		*/
		
		if(fabs((GetHeading() - rotation).SignedNormalize().GetValue()) < TargetAngleTolerance.GetValue()) survey_count++;
			//else Keep trying to reach the turning angle
	}
	// Set the survey countdown
	else {
		SetIsHeadingToNest(false); // Turn on error for this
		SetTarget(LoopFunctions->RedCirclePosition); 
		
		CPFA_state = RETURNING;
		survey_count = 0; // Reset
                searchingTime+=SimulationTick()-startTime;//qilu 10/22
                startTime = SimulationTick();//qilu 10/22
            
	}
}

// Helper function to join the queue
void CPFA_controller::JoinQueue() {
    selectedWaitingQueueIndex = LoopFunctions->AddToQueue(GetId());
    
    if(selectedWaitingQueueIndex != -1) {
        argos::CVector2 queuePosition = LoopFunctions->waitingQueuePositions[selectedWaitingQueueIndex];
        SetTarget(queuePosition);
        SetIsHeadingToNest(false);
        isInQueue = true;
        // argos::LOG << GetId() << " Joining queue at position " << selectedWaitingQueueIndex << std::endl;
    } else {
        // Queue is full - implement fallback behavior
        // argos::LOG << GetId() << " Queue is full! Waiting nearby..." << std::endl;
        
        // Option 1: Wait near red circle
        argos::CVector2 waitPosition = LoopFunctions->RedCirclePosition + 
                                      argos::CVector2(redCircleRadius + 2.0, 0.0);
        SetTarget(waitPosition);
        SetIsHeadingToNest(false);
        
        // Option 2: Could implement a secondary waiting area
        // Option 3: Could try again next tick
    }
}

// Helper function to leave the queue
void CPFA_controller::LeaveQueue() {
    if(isInQueue && selectedWaitingQueueIndex != -1) {
        LoopFunctions->RemoveFromQueue(GetId());
        isInQueue = false;
        selectedWaitingQueueIndex = -1;
    }
}

// Helper function to update position in queue (move forward when others leave)
void CPFA_controller::UpdateQueuePosition() {
    if(!IsAtTarget()) {
        // Still moving to current queue position
        return;
    }
    
    // Check if our position in queue has changed (moved forward)
    argos::CVector2 newQueuePosition = LoopFunctions->GetUpdatedQueuePosition(GetId());
    argos::CVector2 currentTarget = GetTarget();
    
    if((newQueuePosition - currentTarget).Length() > 0.1) {
        // We've been moved to a new position - go there
        SetTarget(newQueuePosition);
        SetIsHeadingToNest(false);
        // argos::LOG << GetId() << " Moving forward in queue" << std::endl;
    } else {
        // Wait in current position
        Wait(2);
    }
}

// Optional: Reset method to clear queue state when robot resets
void CPFA_controller::ResetQueueState() {
    if(isInQueue && selectedWaitingQueueIndex != -1) {
        LeaveQueue();
    }
    isInQueue = false;
    selectedWaitingQueueIndex = -1;
    nextSearchType = RANDOM_SEARCH;
}

/*****
 * RETURNING: Stay in this state until the robot has returned to the nest.
 * This state is triggered when a robot has found food or when it has given
 * up on searching and is returning to the nest.
 *****/
void CPFA_controller::Returning() {
    //LOG<<"Returning..."<<endl;
    //SetHoldingFood();
	if(isGivingUpSearch) {
		// If the robot is near the red circle, bounce back after getting information.
		if(LoopFunctions->IsNearRedCircle(GetPosition()) && !isHoldingFood) {
			// argos::LOG << GetId() << " Reached red circle without food - bouncing back." << std::endl;
			
			// BOUNCE BACK LOGIC: Move away from the red circle
			argos::CVector2 currentPos = GetPosition();
			argos::CVector2 directionAwayFromCenter = (currentPos - LoopFunctions->RedCirclePosition).Normalize();
			
			// Calculate red circle radius if not already set
			redCircleRadius = LoopFunctions->RedCircleRadius;
			
			// Set target to a position further away from the red circle
			argos::Real bounceDistance = redCircleRadius + 1.0; // Move 1 unit beyond the red circle
			argos::CVector2 bounceTarget = LoopFunctions->RedCirclePosition + directionAwayFromCenter * bounceDistance;
			
			SetTarget(bounceTarget);
			SetIsHeadingToNest(false);
			
			// Based on a Poisson CDF, the robot may or may not create a pheromone
			// located at the last place it picked up food.
			argos::Real poissonCDF_pLayRate    = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfLayingPheromone);
			argos::Real poissonCDF_sFollowRate = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfSiteFidelity);
			argos::Real r1 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
			argos::Real r2 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
			
			if(poissonCDF_pLayRate > r1 && updateFidelity) {
				TrailToShare.push_back(argos::CVector2(0.0, 0.3));
				argos::Real timeInSeconds = (argos::Real)(SimulationTick() / SimulationTicksPerSecond());
				Pheromone sharedPheromone(SiteFidelityPosition, TrailToShare, timeInSeconds, LoopFunctions->RateOfPheromoneDecay, ResourceDensity);
				LoopFunctions->PheromoneList.push_back(sharedPheromone);
				sharedPheromone.Deactivate();
			}
			TrailToShare.clear();
			
			// Store the decision for next search behavior but DON'T execute it yet
			if(updateFidelity && poissonCDF_sFollowRate > r2) {
				// Will use site fidelity after bounce
				nextSearchType = SITE_FIDELITY;
			}
			else if(LoopFunctions->PheromoneList.size() > 0) {
				// Check if there are pheromones to follow (don't call SetTargetPheromone yet)
				nextSearchType = PHEROMONE_TRAIL;
			}
			else {
				// Will use random search after bounce
				nextSearchType = RANDOM_SEARCH;
			}
			
			// Set bounce state - don't reset other flags yet
			isBouncing = true;
			
			// argos::LOG << GetId() << " Bouncing to: " << bounceTarget.GetX() << ", " << bounceTarget.GetY() << std::endl;
			
		} else if(isBouncing) {
			// Check if we've moved far enough away from the red circle or reached bounce target
			argos::Real distanceFromCenter = (GetPosition() - LoopFunctions->RedCirclePosition).Length();
			argos::Real minBounceDistance = redCircleRadius + LoopFunctions->RedCircleRadiusMultiplier; // Minimum distance to be considered "bounced back"
			
			if(distanceFromCenter >= minBounceDistance || IsAtTarget()) {
				// argos::LOG << GetId() << " Bounce back complete. Starting next search phase." << std::endl;
				
				// Now execute the stored search decision
				if(nextSearchType == SITE_FIDELITY) {
					SetTarget(SiteFidelityPosition);
					SetIsHeadingToNest(false);
					isInformed = true;
					isUsingSiteFidelity = true;
					// argos::LOG << GetId() << " Using site fidelity for next search." << std::endl;
				}
				else if(nextSearchType == PHEROMONE_TRAIL && SetTargetPheromone()) {
					SetIsHeadingToNest(false);
					isInformed = true;
					isUsingSiteFidelity = false;
					// argos::LOG << GetId() << " Using pheromone trail for next search." << std::endl;
				}
				else {
					// Use random search (fallback)
					SetRandomSearchLocation(false);
					SetIsHeadingToNest(false);
					isInformed = false;
					isUsingSiteFidelity = false;
					// argos::LOG << GetId() << " Using random search." << std::endl;
				}
				
				// Reset all flags for next food collection cycle
				hasStartedBoundaryFollow = false;
				isFollowingCircleBoundary = false;
				isPausingOnBoundary = false;
				isHeadingToEntryPoint = false;
				isGivingUpSearch = false;
				isBouncing = false;
				nextSearchType = RANDOM_SEARCH; // Reset search type
				CPFA_state = DEPARTING;   
				travelingTime += SimulationTick() - startTime;
				startTime = SimulationTick();
			}
			// Continue moving to bounce target if not far enough yet
		}
	} else {


		//Version 2.0: Tameem
		if(isHoldingFood && !isReachedFifthInnerCircle) {

			if(LoopFunctions->IsNearRedCircle(GetPosition()) || LoopFunctions->IsInsideRedCircle(GetPosition())) {

				if(LoopFunctions->IsInsideRedCircle(GetPosition())) {
					if(!timeSet) {
						timeInsideRedCircle = LoopFunctions->getSimTimeInSeconds();
						timeSet = true;
					}
				}

				if(LoopFunctions->IsNearExitPoint(GetPosition())) {
					isReachedFifthInnerCircle = true;
					isFollowingPredefinedPath = true;
					predefinedPathIndex = RNG->Uniform(CRange<UInt32>(5, 20));
					SetTarget(LoopFunctions->SpiralPathCoordinatesForController[predefinedPathIndex]);
					SetIsHeadingToNest(false);
					CPFA_state = FOLLOWING_ENTRY_PATH;
					return;
				}
				
				if(LoopFunctions->IsNearFifthInnerCircle(GetPosition()).isHit) {
					// LOG << GetId() << " hit at fifth inner circle" << std::endl;
					isReachedFifthInnerCircle = true;
					isFollowingPredefinedPath = true;
					predefinedPathIndex = LoopFunctions->IsNearFifthInnerCircle(GetPosition()).positionIndex;
					// LoopFunctions->AddRobotToPathQueue(GetId());
					SetTarget(LoopFunctions->SpiralPathCoordinatesForController[predefinedPathIndex]);
					SetIsHeadingToNest(false);
					CPFA_state = FOLLOWING_ENTRY_PATH;
					return;
					
				} else {

					if(!isReachedFirstInnerCircle && LoopFunctions->IsNearFirstInnerCircle(GetPosition()).isHit) {
						predefinedPathIndexOnFirstInnerCircle = LoopFunctions->IsNearFirstInnerCircle(GetPosition()).positionIndex;
						isReachedFirstInnerCircle = true;
					}
					if(!isReachedSecondInnerCircle && LoopFunctions->IsNearSecondInnerCircle(GetPosition()).isHit) {
						predefinedPathIndexOnSecondInnerCircle = LoopFunctions->IsNearSecondInnerCircle(GetPosition()).positionIndex;
						isReachedSecondInnerCircle = true;
					}
					if(!isReachedThirdInnerCircle && LoopFunctions->IsNearThirdInnerCircle(GetPosition()).isHit) {
						predefinedPathIndexOnThirdInnerCircle = LoopFunctions->IsNearThirdInnerCircle(GetPosition()).positionIndex;
						isReachedThirdInnerCircle = true;
					}
					if(!isReachedFourthInnerCircle && LoopFunctions->IsNearFourthInnerCircle(GetPosition()).isHit) {
						predefinedPathIndexOnFourthInnerCircle = LoopFunctions->IsNearFourthInnerCircle(GetPosition()).positionIndex;
						isReachedFourthInnerCircle = true;
					}

					if(CollisionDetection()) {
						if(!isWaitingForCollision) {
							timeCollided = collision_counter;
							isWaitingForCollision = true;
						}

						if(isWaitingForCollision) {
							size_t collision = collision_counter - timeCollided;
							if(collision > 50) {
								// LOG << LoopFunctions->SimTime << ": " << GetId() << " collided " 
								//     << collision << "times. Moving to outer circle." << std::endl;
								if(isReachedFourthInnerCircle) {
									predefinedPathIndex = predefinedPathIndexOnFourthInnerCircle;
								} else if(isReachedThirdInnerCircle) {
									predefinedPathIndex = predefinedPathIndexOnThirdInnerCircle;
								} else if(isReachedSecondInnerCircle) {
									predefinedPathIndex = predefinedPathIndexOnSecondInnerCircle;
								} else if(isReachedFirstInnerCircle) {
									predefinedPathIndex = predefinedPathIndexOnFirstInnerCircle;
								}
								else {
									return;
								}

								SetTarget(LoopFunctions->SpiralPathCoordinatesForController[predefinedPathIndex]);
								SetIsHeadingToNest(false);
								isFollowingOuterCircle = true;
								isWaitingForCollision = false;
								CPFA_state = FOLLOWING_OUTER_CIRCLE;
								return;
							}
						}
					} else {
						if(isWaitingForCollision) {
							isWaitingForCollision = false;
							timeCollided = 0;
						}
					}

					// Robot is not at last inner circle yet, head towards it
					argos::CVector2 currentPos = GetPosition();
					argos::CVector2 directionToCenter = (LoopFunctions->RedCirclePosition - currentPos).Normalize();
					
					//fRedCircleRadius * 0.9f - First Inner Circle
					//fRedCircleRadius * 0.8f - Second Inner Circle
					//fRedCircleRadius * 0.7f - Third Inner Circle
					//fRedCircleRadius * 0.6f - Fourth Inner Circle
					//fRedCircleRadius * 0.5f - Fourth Inner Circle
					
					// Set target to a point slightly outside the red circle (safe distance)
					argos::Real safeDistance = (redCircleRadius * 0.5f) + 0.1;
					argos::CVector2 targetOnCircle = LoopFunctions->RedCirclePosition - directionToCenter * safeDistance;
					
					SetTarget(targetOnCircle);
					SetIsHeadingToNest(false);
				}
			} else {
				// Robot is not at last inner circle yet, head towards it
				argos::CVector2 currentPos = GetPosition();
				argos::CVector2 directionToCenter = (LoopFunctions->RedCirclePosition - currentPos).Normalize();

				// Set target to a point slightly outside the red circle (safe distance)
				argos::Real safeDistance = redCircleRadius + 0.1;
				argos::CVector2 targetOnCircle = LoopFunctions->RedCirclePosition - directionToCenter * safeDistance;
				
				SetTarget(targetOnCircle);
				SetIsHeadingToNest(false);
			}


		}

		if(isHoldingFood && isNearForbiddenArea) {
				
			if(IsAtTarget()) {
				isNearForbiddenArea = false;
				isFollowingAvoidance = false;
				isReachedFifthInnerCircle = true;
				isFollowingPredefinedPath = true;
				// LoopFunctions->AddRobotToPathQueue(GetId());
				predefinedPathIndex = 0;
				SetTarget(LoopFunctions->SpiralPathCoordinatesForController[predefinedPathIndex]);
				SetIsHeadingToNest(false);
				CPFA_state = FOLLOWING_ENTRY_PATH;
				return;
			} else {
				CVector2 entryPoint(LoopFunctions->RedCirclePosition.GetX(), 
                       LoopFunctions->RedCirclePosition.GetY() + LoopFunctions->RedCircleRadius
                    );
				SetTarget(entryPoint);
				SetIsHeadingToNest(false);
			}
		}
		
	}
}

void CPFA_controller::FollowingOuterCircle() {
	if(isFollowingOuterCircle && isHoldingFood) {
		if(IsAtTarget()) {
			isReachedFifthInnerCircle = true;
			isFollowingPredefinedPath = true;
			isFollowingOuterCircle = false;
			// LoopFunctions->AddRobotToPathQueue(GetId());
			CPFA_state = FOLLOWING_ENTRY_PATH;
			return;
		} else {
			SetTarget(LoopFunctions->SpiralPathCoordinatesForController[predefinedPathIndex]);
			SetIsHeadingToNest(false);
		}
	}
}

bool CPFA_controller::CollisionDetection() {
	//log current CPFA State
	argos::CVector2 collisionVector = GetCollisionVector();
	argos::Real collisionAngle = ToDegrees(collisionVector.Angle()).GetValue();
	bool isCollisionDetected = false;
	
	// this is the normal collision logic where a robot determines a where it is colliding with another robot and the turn it must take.
    if (GetStatus() == "FOLLOWING_ENTRY_PATH" || GetStatus() == "EXITING") {
        if (GoStraightAngleRangeInDegreesInRegion.WithinMinBoundIncludedMaxBoundIncluded(collisionAngle)
            && collisionVector.Length() > 0.0) {
			Stop();
            stopCounter++; // Increment the stop counter
            if (stopCounter > 75) {
                // Resume movement after 200 timesteps
                stopCounter = 0; // Reset the counter
            }
			Stop();
            return true; // Keep the robot stopped
        } else{
			// Reset the stop counter if no collision is detected
			stopCounter = 0;
		}
    }

	else{
	
		if(GoStraightAngleRangeInDegrees.WithinMinBoundIncludedMaxBoundIncluded(collisionAngle)
			&& collisionVector.Length() > 0.0) {

			// if a robot is following a path, we dont want to avoid collisions since paths dont overlap. So there will be no
			// collisions as longs other robots don't interfere.
			isCollisionDetected = true;
			collision_counter++;
			// if(goingtoentry) {
			// 	return isCollisionDetected;
			// }	 
			Stop();


			while(MovementStack.size() > 0) MovementStack.pop();

			PushMovement(FORWARD, SearchStepSize);

			Real randomNumber = RNG->Uniform(CRange<Real>(0.5, 1.0));
			collisionDelay = SimulationTick() + (size_t)(randomNumber*SimulationTicksPerSecond());//qilu 10/26/2016	

			if(collisionAngle <= 0.0)  {
				//argos::LOG << collisionAngle << std::endl << collisionVector << std::endl << std::endl;
				SetLeftTurn(collisionAngle); //qilu 09/24/2016
			} else {
				//argos::LOG << collisionAngle << std::endl << collisionVector << std::endl << std::endl;
				SetRightTurn(collisionAngle); //qilu 09/24/2016
			}

		}
	}

	return isCollisionDetected;
}

void CPFA_controller::FollowingEntryPath() {

	argos::CVector2 currentPos = GetPosition();
	argos::CVector2 currentTarget = LoopFunctions->SpiralPathCoordinatesForController[predefinedPathIndex];
	argos::Real distanceToCurrentWaypoint = (currentPos - currentTarget).Length();
	
	// Check if we've reached the current waypoint
	if(distanceToCurrentWaypoint < 0.1 && isFollowingPredefinedPath && !isWaitingForNest) { // Waypoint reached threshold
		predefinedPathIndex++;

		// Check if we've completed the path (reached nest) - Works Fine Up to Here
		if(predefinedPathIndex >= LoopFunctions->SpiralPathCoordinatesForController.size()-1) {
			
			int availablePathToNest = LoopFunctions->GetAvailablePathToNest();
			
			if(availablePathToNest != -1) {
			
				selectedNestIndex = availablePathToNest;
				// LoopFunctions->RemoveRobotFromPathQueue(GetId());
				LoopFunctions->AddRobotToPathQueueToNest(selectedNestIndex, GetId());

				isFollowingPredefinedPath = false;
				isFollowingNestPredefinedEntryPath = true;
				predefinedNestEntryPathIndex = 0;

				Real nestX = LoopFunctions->NestPositions[selectedNestIndex].GetX();
				Real nestY = LoopFunctions->NestPositions[selectedNestIndex].GetY();

				if(nestX == 0.0 && nestY == -0.3) { // index 2
					predefinedNestEntryPath = LoopFunctions->nest1EntryPoints;
					predefinedNestExitPath = LoopFunctions->nest2ExitPoints;
				} else if(nestX == -0.3 && nestY == 0) { // index 3
					predefinedNestEntryPath = LoopFunctions->nest2EntryPoints;
					predefinedNestExitPath = LoopFunctions->nest1ExitPoints;
				} else if(nestX == 0.0 && nestY == 0.3) { // index 0
					predefinedNestEntryPath = LoopFunctions->nest3EntryPoints;
					predefinedNestExitPath = LoopFunctions->nest4ExitPoints;
				} else if(nestX == 0.3 && nestY == 0.0) { // index 1
					predefinedNestEntryPath = LoopFunctions->nest4EntryPoints;
					predefinedNestExitPath = LoopFunctions->nest3ExitPoints;
				} else {
					LOG << "NO nest position found!" << std::endl;
				}

			} 
			else {
				isWaitingForNest = true;
				SetTarget(LoopFunctions->SpiralPathCoordinatesForController[predefinedPathIndex-1]);
				SetIsHeadingToNest(false);
				// argos::LOG << GetId() << " No available path to nest. Waiting in queue." << std::endl;
			}
			
		} else {
			// LOG << LoopFunctions->NestPositions[3].GetX() << ", " << LoopFunctions->NestPositions[3].GetY() << std::endl;
			// Move to next waypoint
			SetTarget(LoopFunctions->SpiralPathCoordinatesForController[predefinedPathIndex]);
		}
	}

	if(isWaitingForNest && isHoldingFood) {
		int availablePathToNest = LoopFunctions->GetAvailablePathToNest();
		if(availablePathToNest != -1) {
			selectedNestIndex = availablePathToNest;
			// LoopFunctions->RemoveRobotFromPathQueue(GetId());
			LoopFunctions->AddRobotToPathQueueToNest(selectedNestIndex, GetId());

			isFollowingPredefinedPath = false;
			isFollowingNestPredefinedEntryPath = true;
			isWaitingForNest = false;
			predefinedNestEntryPathIndex = 0;

			Real nestX = LoopFunctions->NestPositions[selectedNestIndex].GetX();
			Real nestY = LoopFunctions->NestPositions[selectedNestIndex].GetY();
			if(nestX == 0.0 && nestY == -0.3) {
				predefinedNestEntryPath = LoopFunctions->nest1EntryPoints;
				predefinedNestExitPath = LoopFunctions->nest2ExitPoints;
			} else if(nestX == -0.3 && nestY == 0.0) {
				predefinedNestEntryPath = LoopFunctions->nest2EntryPoints;
				predefinedNestExitPath = LoopFunctions->nest1ExitPoints;
			} else if(nestX == 0.0 && nestY == 0.3) {
				predefinedNestEntryPath = LoopFunctions->nest3EntryPoints;
				predefinedNestExitPath = LoopFunctions->nest4ExitPoints;
			} else if(nestX == 0.3 && nestY == 0.0) {
				predefinedNestEntryPath = LoopFunctions->nest4EntryPoints;
				predefinedNestExitPath = LoopFunctions->nest3ExitPoints;
			}
		} else {
			SetTarget(LoopFunctions->SpiralPathCoordinatesForController[predefinedPathIndex-1]);
			SetIsHeadingToNest(false);
		}

	}

	if(isFollowingNestPredefinedEntryPath && isHoldingFood) {
		argos::CVector2 currentPos = GetPosition();
		argos::CVector2 currentTarget(predefinedNestEntryPath[predefinedNestEntryPathIndex].GetX(), predefinedNestEntryPath[predefinedNestEntryPathIndex].GetY());
		argos::Real distanceToCurrentWaypoint = (currentPos - currentTarget).Length();
		if(distanceToCurrentWaypoint < 0.15) {
			predefinedNestEntryPathIndex++;
			if(predefinedNestEntryPathIndex >= predefinedNestEntryPath.size()-1) {
				isFollowingNestPredefinedEntryPath = false;
				isHeadingToEntryPoint = true;
			} else {
				CVector2 target(predefinedNestEntryPath[predefinedNestEntryPathIndex].GetX(), predefinedNestEntryPath[predefinedNestEntryPathIndex].GetY());
				SetTarget(target);
			}
		}
	}

	// Check if robot reached nest from predefined path
	if(isHeadingToEntryPoint && isHoldingFood) {
		if(IsInTheNest(LoopFunctions->NestPositions[selectedNestIndex])) {

			if(timeSet) {
				totalTimeInsideCirle += (LoopFunctions->getSimTimeInSeconds() - timeInsideRedCircle);
				timeSet = false;
			}

			Wait(5);

			isFollowingNestPredefinedExitPath = true;
			predefinedNestExitPathIndex = 0;
			
			// Based on a Poisson CDF, the robot may or may not create a pheromone
			// located at the last place it picked up food.
			argos::Real poissonCDF_pLayRate    = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfLayingPheromone);
			argos::Real r1 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
			
			// Update the location of the nest
			num_targets_collected++;
			LoopFunctions->currNumCollectedFood++;
			LoopFunctions->setScore(num_targets_collected);
			
			if(poissonCDF_pLayRate > r1 && updateFidelity) {
				TrailToShare.push_back(LoopFunctions->NestPositions[selectedNestIndex]);
				argos::Real timeInSeconds = (argos::Real)(SimulationTick() / SimulationTicksPerSecond());
				Pheromone sharedPheromone(SiteFidelityPosition, TrailToShare, timeInSeconds, LoopFunctions->RateOfPheromoneDecay, ResourceDensity);
				LoopFunctions->PheromoneList.push_back(sharedPheromone);
				sharedPheromone.Deactivate();
			}
			TrailToShare.clear();

			// Food is now dropped, update state
			isHoldingFood = false;
			
			// EXIT STRAIGHT: Set exit direction from nest outward (away from entry point)
			SetTarget(
				CVector2(predefinedNestExitPath[predefinedNestExitPathIndex].GetX(), 
					predefinedNestExitPath[predefinedNestExitPathIndex].GetY()
				)
			);
			SetIsHeadingToNest(false);
			CPFA_state = EXITING;
			return;
			

		} else {
			SetTarget(LoopFunctions->NestPositions[selectedNestIndex]);
		}
	}
}

// NEW: Generate predefined spiral/curved path from entry point to nest
void CPFA_controller::GeneratePredefinedPath() {
	// redCircleRadius = LoopFunctions->NestRadius * LoopFunctions->RedCircleRadiusMultiplier;
	predefinedPath.clear();
	
	// Parameters for spiral path
	// argos::Real numTurns = 2.0; // Number of spiral turns
	// int numWaypoints = 20; // Number of waypoints along the path
	// argos::Real startRadius = redCircleRadius * 0.8; // Start radius (near entry point)
	// argos::Real endRadius = LoopFunctions->NestRadius * LoopFunctions->RedCircleRadiusMultiplier; // End radius (near nest center)
	
	for(int i = 0; i < LoopFunctions->SpiralPathCoordinates.size(); i++) {
		// argos::Real t = (argos::Real)i / (argos::Real)(numWaypoints - 1); // Parameter from 0 to 1
		
		// // Spiral inward: radius decreases as we progress
		// argos::Real currentRadius = startRadius - (startRadius - endRadius) * t;
		
		// // Angle: start from north (π/2) and spiral inward
		// argos::Real currentAngle = M_PI/2.0 - numTurns * 2.0 * M_PI * t;
		
		// Calculate waypoint position
		argos::CVector2 waypoint(
			LoopFunctions->SpiralPathCoordinates[i].GetX(),
			LoopFunctions->SpiralPathCoordinates[i].GetY()
		);
		
		predefinedPath.push_back(waypoint);
	}
	
	// Ensure the last waypoint is at the nest center
	// predefinedPath.push_back(argos::CVector2(0.0, 0.3)); // need to fix later.
	
	// argos::LOG << "Generated predefined spiral path with " << predefinedPath.size() << " waypoints." << std::endl;
}

// NEW: Check if robot can enter the predefined path (queue management)
bool CPFA_controller::CanEnterPredefinedPath() {
	// Check if there's space in the path queue
	// This assumes LoopFunctions has methods to manage the queue
	return LoopFunctions->GetPathQueueSize() < LoopFunctions->GetMaxPathQueueSize();
}


void CPFA_controller::Exiting() {

	if(isFollowingNestPredefinedExitPath && !isHoldingFood) {
		argos::CVector2 currentPos = GetPosition();
		argos::CVector2 currentTarget(predefinedNestExitPath[predefinedNestExitPathIndex].GetX(), predefinedNestExitPath[predefinedNestExitPathIndex].GetY());
		argos::Real distanceToCurrentWaypoint = (currentPos - currentTarget).Length();
		if(distanceToCurrentWaypoint < 0.15) {
			predefinedNestExitPathIndex++;
			if(predefinedNestExitPathIndex == 4) {
				LoopFunctions->RemoveRobotFromPathQueueToNest(selectedNestIndex, GetId());
			}
			if(predefinedNestExitPathIndex >= predefinedNestExitPath.size()-1) {
				isFollowingNestPredefinedExitPath = false;
			} else {
				CVector2 target(predefinedNestExitPath[predefinedNestExitPathIndex].GetX(), predefinedNestExitPath[predefinedNestExitPathIndex].GetY());
				SetTarget(target);
			}
		}
	}

	if(!isFollowingNestPredefinedExitPath) {

		argos::Real poissonCDF_sFollowRate = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfSiteFidelity);
		argos::Real r2 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));

		// Determine probabilistically whether to use site fidelity, pheromone trails, or random search
		if(updateFidelity && poissonCDF_sFollowRate > r2) {
			// Use site fidelity
			// LOG << GetId() << " Using site fidelity." << std::endl;
			SetIsHeadingToNest(false);
			argos::CVector2 safeTarget = GetRedCircleAvoidanceTarget(SiteFidelityPosition);
			SetTarget(safeTarget);
			isInformed = true;
		}
		else if(SetTargetPheromone()) {
			// Use pheromone waypoints
			// LOG << GetId() << " Using pheromone waypoints." << std::endl;
			SetIsHeadingToNest(false);
			isInformed = true;
			isUsingSiteFidelity = false;
		}
		else {
			// Use random search
			SetRandomSearchLocation(false);
			SetIsHeadingToNest(false);
			isInformed = false;
			isUsingSiteFidelity = false;
		}

		// Reset all flags for next food collection cycle
		hasStartedBoundaryFollow = false;
		isFollowingCircleBoundary = false;
		isPausingOnBoundary = false;
		isHeadingToEntryPoint = false;
		isGivingUpSearch = false;
		isFollowingPredefinedPath = false;
		isNearForbiddenArea = false;
		isFollowingNestPredefinedExitPath = false;
		isFollowingNestPredefinedEntryPath = false;
		isReachedFirstInnerCircle = false;
		isReachedSecondInnerCircle = false;
		isReachedThirdInnerCircle = false;
		isReachedFourthInnerCircle = false;
		isReachedFifthInnerCircle = false;
		isFollowingAvoidance = false;
		isNearForbiddenArea = false;
		// isEndingForbiddenArea = false;
		predefinedPathIndex = 0;
		predefinedNestExitPathIndex = 0;
		predefinedNestEntryPathIndex = 0;
		CPFA_state = DEPARTING;  
		// CPFA_state = RETURNING;
		travelingTime += SimulationTick() - startTime;
		startTime = SimulationTick();
	}

}

// Method to check if a direct path would go through the red circle
bool CPFA_controller::IsPathThroughRedCircle(const argos::CVector2& start, const argos::CVector2& end) {
    if(isHoldingFood) return false; // Allow passage when carrying food
    
    // Calculate red circle radius if not set
    if(redCircleRadius == 0.0) {
        redCircleRadius = LoopFunctions->RedCircleRadius;
    }
    
    argos::CVector2 circleCenter = LoopFunctions->RedCirclePosition;
    
    // Vector from start to end
    argos::CVector2 pathVector = end - start;
    argos::Real pathLength = pathVector.Length();
    
    // If path is very short, don't bother checking
    if(pathLength < 0.1) return false;
    
    argos::CVector2 pathDirection = pathVector.Normalize();
    
    // Vector from start to circle center
    argos::CVector2 toCenter = circleCenter - start;
    
    // Project toCenter onto path direction to find closest point on path to circle center
    argos::Real projectionLength = toCenter.DotProduct(pathDirection);
    
    // Clamp projection to path bounds
    projectionLength = std::max(0.0, std::min(pathLength, projectionLength));
    
    // Find closest point on path to circle center
    argos::CVector2 closestPoint = start + pathDirection * projectionLength;
    
    // Check if closest point is within the red circle (with small buffer)
    argos::Real distanceToCenter = (closestPoint - circleCenter).Length();
    argos::Real safeRadius = redCircleRadius + 0.2; // Add buffer
    
    return distanceToCenter < safeRadius;
}

// Method to get circumnavigation target around red circle
argos::CVector2 CPFA_controller::GetCircumnavigationTarget(const argos::CVector2& currentPos, const argos::CVector2& finalTarget) {
    argos::CVector2 circleCenter = LoopFunctions->RedCirclePosition;
    argos::Real safeRadius = redCircleRadius + 0.3; // Safe distance from red circle
    
    // If this is the start of circumnavigation, determine direction
    if(!isCircumnavigatingRedCircle) {
        isCircumnavigatingRedCircle = true;
        circumnavigationFinalTarget = finalTarget;
        
        // Determine circumnavigation direction based on which way is shorter
        argos::CVector2 currentRelative = currentPos - circleCenter;
        argos::CVector2 targetRelative = finalTarget - circleCenter;
        
        argos::Real currentAngle = atan2(currentRelative.GetY(), currentRelative.GetX());
        argos::Real targetAngle = atan2(targetRelative.GetY(), targetRelative.GetX());
        
        argos::Real clockwiseDistance = targetAngle - currentAngle;
        if(clockwiseDistance < 0) clockwiseDistance += 2.0 * M_PI;
        
        argos::Real counterClockwiseDistance = 2.0 * M_PI - clockwiseDistance;
        
        // Choose shorter path
        circumnavigationDirection = (clockwiseDistance <= counterClockwiseDistance) ? 1 : -1;
    }
    
    // Calculate current angle relative to circle center
    argos::CVector2 currentRelative = currentPos - circleCenter;
    argos::Real currentAngle = atan2(currentRelative.GetY(), currentRelative.GetX());
    
    // Move along the circle boundary in the chosen direction
    argos::Real angularStep = 0.2 * circumnavigationDirection; // Adjust step size as needed
    argos::Real nextAngle = currentAngle + angularStep;
    
    // Calculate next target position on circle boundary
    argos::CVector2 nextTarget(
        circleCenter.GetX() + safeRadius * cos(nextAngle),
        circleCenter.GetY() + safeRadius * sin(nextAngle)
    );
    
    return nextTarget;
}

// Method to get target that avoids red circle
argos::CVector2 CPFA_controller::GetRedCircleAvoidanceTarget(const argos::CVector2& desiredTarget) {
    argos::CVector2 currentPos = GetPosition();
    
    // Check if we're currently circumnavigating
    if(isCircumnavigatingRedCircle) {
        // Check if we can now go directly to final target
        if(!IsPathThroughRedCircle(currentPos, circumnavigationFinalTarget)) {
            // We can go directly now
            isCircumnavigatingRedCircle = false;
            return circumnavigationFinalTarget;
        } else {
            // Continue circumnavigating
            return GetCircumnavigationTarget(currentPos, circumnavigationFinalTarget);
        }
    }
    
    // Check if direct path would go through red circle
    if(IsPathThroughRedCircle(currentPos, desiredTarget)) {
        // Start circumnavigation
        return GetCircumnavigationTarget(currentPos, desiredTarget);
    }
    
    // Direct path is safe
    return desiredTarget;
}

void CPFA_controller::SetRandomSearchLocation(bool isExit) {
    argos::Real random_wall = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
    argos::Real x = 0.0, y = 0.0;
    argos::CVector2 searchPosition;
    bool validPosition = false;

    // Calculate red circle radius if not already set
    if(redCircleRadius == 0.0) {
        redCircleRadius = LoopFunctions->RedCircleRadius;
    }

	if(isExit) {
		if(random_wall < 0.5) {
			x = RNG->Uniform(ForageRangeX);
			y = ForageRangeY.GetMax();
		} else {
			x = RNG->Uniform(ForageRangeX);
			y = ForageRangeY.GetMin();
		}
	} else {

		// Keep trying until we find a valid position outside the red circle
		while(!validPosition) {
			/* north wall */
			if(random_wall < 0.25) {
				x = RNG->Uniform(ForageRangeX);
				y = ForageRangeY.GetMax();
			}
			/* south wall */
			else if(random_wall < 0.5) {
				x = RNG->Uniform(ForageRangeX);
				y = ForageRangeY.GetMin();
			}
			/* east wall */
			else if(random_wall < 0.75) {
				x = ForageRangeX.GetMax();
				y = RNG->Uniform(ForageRangeY);
			}
			/* west wall */
			else {
				x = ForageRangeX.GetMin();
				y = RNG->Uniform(ForageRangeY);
			}

			searchPosition.Set(x, y);
			
			// Check if the position is outside the red circle
			argos::Real distanceFromCenter = (searchPosition - LoopFunctions->RedCirclePosition).Length();
			if(distanceFromCenter > redCircleRadius) {
				validPosition = true;
			} else {
				// Try a different random wall if current position is invalid
				random_wall = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
			}
		}
	}

        
    SetIsHeadingToNest(true); // Turn off error for this
    SetTarget(searchPosition);
}

/*****
 * Check if the iAnt is finding food. This is defined as the iAnt being within
 * the distance tolerance of the position of a food item. If the iAnt has found
 * food then the appropriate boolean flags are triggered.
 *****/
void CPFA_controller::SetHoldingFood() {
    // Is the iAnt already holding food?
    if(IsHoldingFood() == false) {
        // No, the iAnt isn't holding food. Check if we have found food at our
        // current position and update the food list if we have.

        std::vector<argos::CVector2> newFoodList;
        std::vector<argos::CColor> newFoodColoringList;
        size_t i = 0, j = 0;

        for(i = 0; i < LoopFunctions->FoodList.size(); i++) {
            if((GetPosition() - LoopFunctions->FoodList[i]).SquareLength() < FoodDistanceTolerance ) {
                // We found food! Calculate the nearby food density.
                isHoldingFood = true;
                CPFA_state = SURVEYING;
                j = i + 1;
                searchingTime+=SimulationTick()-startTime;
                startTime = SimulationTick();
				break;
            } else {
                //Return this unfound-food position to the list
                newFoodList.push_back(LoopFunctions->FoodList[i]);
                newFoodColoringList.push_back(LoopFunctions->FoodColoringList[i]);
            }
        }

        if(j>0){
            for(; j < LoopFunctions->FoodList.size(); j++) {
                newFoodList.push_back(LoopFunctions->FoodList[j]);
                newFoodColoringList.push_back(LoopFunctions->FoodColoringList[j]);
            }
        }
   
        // We picked up food. Update the food list minus what we picked up.
        if(IsHoldingFood()) {
            LoopFunctions->FoodList = newFoodList;
            LoopFunctions->FoodColoringList = newFoodColoringList;
            SetLocalResourceDensity();
        }
    }
}

/*****
 * If the robot has just picked up a food item, this function will be called
 * so that the food density in the local region is analyzed and saved. This
 * helps facilitate calculations for pheromone laying.
 *
 * Ideally, given that: [*] is food, and [=] is a robot
 *
 * [*] [*] [*] | The maximum resource density that should be calculated is
 * [*] [=] [*] | equal to 9, counting the food that the robot just picked up
 * [*] [*] [*] | and up to 8 of its neighbors.
 *
 * That being said, the random and non-grid nature of movement will not
 * produce the ideal result most of the time. This is especially true since
 * item detection is based on distance calculations with circles.
 *****/
void CPFA_controller::SetLocalResourceDensity() {
	argos::CVector2 distance;

	// remember: the food we picked up is removed from the foodList before this function call
	// therefore compensate here by counting that food (which we want to count)
	ResourceDensity = 1;

	/* Calculate resource density based on the global food list positions. */
	for(size_t i = 0; i < LoopFunctions->FoodList.size(); i++) {
		   distance = GetPosition() - LoopFunctions->FoodList[i];

		   if(distance.SquareLength() < LoopFunctions->SearchRadiusSquared*2) {
			      ResourceDensity++;
			      LoopFunctions->FoodColoringList[i] = argos::CColor::ORANGE;
			      LoopFunctions->ResourceDensityDelay = SimulationTick() + SimulationTicksPerSecond() * 10;
		   }
	}
 
	/* Set the fidelity position to the robot's current position. */
    SiteFidelityPosition = GetPosition();
    isUsingSiteFidelity = true;
    updateFidelity = true; 
    TrailToShare.push_back(SiteFidelityPosition);
    LoopFunctions->FidelityList[controllerID] = SiteFidelityPosition;
    /* Delay for 4 seconds (simulate iAnts scannning rotation). */
	//  Wait(4); // This function is broken. It causes the rover to move in the wrong direction after finishing its local resource density test 

	//ofstream log_output_stream;
	//log_output_stream.open("cpfa_log.txt", ios::app);
	//log_output_stream << "(Survey): " << ResourceDensity << endl;
	//log_output_stream << "SiteFidelityPosition: " << SiteFidelityPosition << endl;
	//log_output_stream.close();
}

/*****
 * Update the global site fidelity list for graphics display and add a new fidelity position.
 *****/
void CPFA_controller::SetFidelityList(argos::CVector2 newFidelity) {
	std::vector<argos::CVector2> newFidelityList;

	/* Remove this robot's old fidelity position from the fidelity list. */
	/*for(size_t i = 0; i < LoopFunctions->FidelityList.size(); i++) {
  if((LoopFunctions->FidelityList[i] - SiteFidelityPosition).SquareLength() != 0.0) {
			newFidelityList.push_back(LoopFunctions->FidelityList[i]);
		}
	} */


	/* Update the global fidelity list. */
	//LoopFunctions->FidelityList = newFidelityList;

        LoopFunctions->FidelityList[controllerID] = newFidelity;
	/* Add the robot's new fidelity position to the global fidelity list. */
	//LoopFunctions->FidelityList.push_back(newFidelity);
 

	/* Update the local fidelity position for this robot. */
	SiteFidelityPosition = newFidelity;
 
  updateFidelity = true;
}

/*****
 * Update the global site fidelity list for graphics display and remove the old fidelity position.
 *****/
void CPFA_controller::SetFidelityList() {
	std::vector<argos::CVector2> newFidelityList;

	/* Remove this robot's old fidelity position from the fidelity list. */
	/* Update the global fidelity list. */
        LoopFunctions->FidelityList.erase(controllerID);
 SiteFidelityPosition = CVector2(10000, 10000);
 updateFidelity = true; 
}

/*****
 * Update the pheromone list and set the target to a pheromone position.
 * return TRUE:  pheromone was successfully targeted
 *        FALSE: pheromones don't exist or are all inactive
 *****/
bool CPFA_controller::SetTargetPheromone() {
	argos::Real maxStrength = 0.0, randomWeight = 0.0;
	bool isPheromoneSet = false;

 if(LoopFunctions->PheromoneList.size()==0) return isPheromoneSet; //the case of no pheromone.
	/* update the pheromone list and remove inactive pheromones */

	/* default target = nest; in case we have 0 active pheromones */
	//SetIsHeadingToNest(true);
	//SetTarget(LoopFunctions->NestPosition);
	/* Calculate a maximum strength based on active pheromone weights. */
	for(size_t i = 0; i < LoopFunctions->PheromoneList.size(); i++) {
		if(LoopFunctions->PheromoneList[i].IsActive()) {
			maxStrength += LoopFunctions->PheromoneList[i].GetWeight();
		}
	}

	/* Calculate a random weight. */
	randomWeight = RNG->Uniform(argos::CRange<argos::Real>(0.0, maxStrength));

	/* Randomly select an active pheromone to follow. */
	for(size_t i = 0; i < LoopFunctions->PheromoneList.size(); i++) {
		   if(randomWeight < LoopFunctions->PheromoneList[i].GetWeight()) {
			       /* We've chosen a pheromone! */
			       SetIsHeadingToNest(false);
          SetTarget(LoopFunctions->PheromoneList[i].GetLocation());
          TrailToFollow = LoopFunctions->PheromoneList[i].GetTrail();
          isPheromoneSet = true;
          /* If we pick a pheromone, break out of this loop. */
          break;
     }

     /* We didn't pick a pheromone! Remove its weight from randomWeight. */
     randomWeight -= LoopFunctions->PheromoneList[i].GetWeight();
	}

	//ofstream log_output_stream;
	//log_output_stream.open("cpfa_log.txt", ios::app);
	//log_output_stream << "Found: " << LoopFunctions->PheromoneList.size()  << " waypoints." << endl;
	//log_output_stream << "Follow waypoint?: " << isPheromoneSet << endl;
	//log_output_stream.close();

	return isPheromoneSet;
}

/*****
 * Calculate and return the exponential decay of "value."
 *****/
argos::Real CPFA_controller::GetExponentialDecay(argos::Real w, argos::Real time, argos::Real lambda) {
	/* convert time into units of haLoopFunctions-seconds from simulation frames */
	//time = time / (LoopFunctions->TicksPerSecond / 2.0);

	//LOG << "time: " << time << endl;
	//LOG << "correlation: " << (value * exp(-lambda * time)) << endl << endl;

	//return (value * std::exp(-lambda * time));
    Real     twoPi       = (CRadians::TWO_PI).GetValue();
    return w + (twoPi-w)* exp(-lambda * time);
}

/*****
 * Provides a bound on the value by rolling over a la modulo.
 *****/
argos::Real CPFA_controller::GetBound(argos::Real value, argos::Real min, argos::Real max) {
	/* Calculate an offset. */
	argos::Real offset = std::abs(min) + std::abs(max);

	/* Increment value by the offset while it's less than min. */
	while (value < min) {
			value += offset;
	}

	/* Decrement value by the offset while it's greater than max. */
	while (value > max) {
			value -= offset;
	}

	/* Return the bounded value. */
	return value;
}

size_t CPFA_controller::GetSearchingTime(){//qilu 10/22
    return searchingTime;
}
size_t CPFA_controller::GetTravelingTime(){//qilu 10/22
    return travelingTime;
}

string CPFA_controller::GetStatus(){//qilu 10/22
    //DEPARTING, SEARCHING, RETURNING
    if (CPFA_state == DEPARTING) return "DEPARTING";
    else if (CPFA_state ==SEARCHING)return "SEARCHING";
    else if (CPFA_state == RETURNING)return "RETURNING";
    else if (CPFA_state == SURVEYING) return "SURVEYING";
	else if (CPFA_state == EXITING) return "EXITING";
	else if (CPFA_state == FOLLOWING_ENTRY_PATH) return "FOLLOWING_ENTRY_PATH";
    //else if (MPFA_state == INACTIVE) return "INACTIVE";
    else return "SHUTDOWN";
    
}

Real CPFA_controller::GetTotalTimeInSideRedCircle() {
	return totalTimeInsideCirle;
}


/*****
 * Return the Poisson cumulative probability at a given k and lambda.
 *****/
argos::Real CPFA_controller::GetPoissonCDF(argos::Real k, argos::Real lambda) {
	argos::Real sumAccumulator       = 1.0;
	argos::Real factorialAccumulator = 1.0;

	for (size_t i = 1; i <= floor(k); i++) {
		factorialAccumulator *= i;
		sumAccumulator += pow(lambda, i) / factorialAccumulator;
	}

	return (exp(-lambda) * sumAccumulator);
}

void CPFA_controller::UpdateTargetRayList() {
	if(SimulationTick() % LoopFunctions->DrawDensityRate == 0 && LoopFunctions->DrawTargetRays == 1) {
		/* Get position values required to construct a new ray */
		argos::CVector2 t(GetTarget());
		argos::CVector2 p(GetPosition());
		argos::CVector3 position3d(p.GetX(), p.GetY(), 0.02);
		argos::CVector3 target3d(t.GetX(), t.GetY(), 0.02);

		/* scale the target ray to be <= searchStepSize */
		argos::Real length = std::abs(t.Length() - p.Length());

		if(length > SearchStepSize) {
			MyTrail.clear();
		} else {
			/* add the ray to the robot's target trail */
			argos::CRay3 targetRay(target3d, position3d);
			MyTrail.push_back(targetRay);

			/* delete the oldest ray from the trail */
			if(MyTrail.size() > MaxTrailSize) {
				MyTrail.erase(MyTrail.begin());
			}

			LoopFunctions->TargetRayList.insert(LoopFunctions->TargetRayList.end(), MyTrail.begin(), MyTrail.end());
			// loopFunctions.TargetRayList.push_back(myTrail);
		}
	}
}

REGISTER_CONTROLLER(CPFA_controller, "CPFA_controller")
