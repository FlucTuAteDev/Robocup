#include "Phase.h"
#include "variables.h"

WaitPhase::WaitPhase(float sec) : delay(sec * 1000) {}
void WaitPhase::start()
{
	start_time = millis();
}
bool WaitPhase::is_finished()
{
	if (!is_started)
		return false;

	if (millis() - start_time >= delay)
	{
		return true;
	}

	return false;
}

#define BALANCE_POS 100
#define PPR 66
#define ROBOT_DIAMETER 23.5
#define DONE_THRESHOLD 200

MovePhase::MovePhase(int cm, bool reset) : distance(cm), reset(reset) {}
void MovePhase::start()
{
	if(reset) variables::positions = BALANCE_POS;
	variables::positions += distance * PPR;
}
bool MovePhase::is_finished()
{
	return abs(variables::positions - BALANCE_POS) < DONE_THRESHOLD; //abs(distance * PPR / 15);
}

TurnPhase::TurnPhase(float angle) : angle(angle) {}
void TurnPhase::start()
{
	float outer_circumference = angle / 360 * (4 * ROBOT_DIAMETER * PI * PPR) ;
	variables::positions += abs(outer_circumference);
	variables::cumpulseright += outer_circumference;
}
bool TurnPhase::is_finished()
{
	return abs(variables::cumpulseright - variables::cumpulseleft) <= 200;
}

StopPhase::StopPhase(){}
void StopPhase::start()
{
	Serial.println((String)"Positions: " + variables::positions);
	variables::positions = BALANCE_POS;
	Serial.println((String)"Positions: " + variables::positions);
}
bool StopPhase::is_finished()
{
	return true;
}