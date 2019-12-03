#ifndef _MY_STRATEGY_HPP_
#define _MY_STRATEGY_HPP_

#include "Debug.hpp"
#include "model/CustomData.hpp"
#include "model/Game.hpp"
#include "model/Unit.hpp"
#include "model/UnitAction.hpp"
#include "strategy/RunawayDirection.h"
#include "strategy/Strategy.h"

class MyStrategy {
public:
  MyStrategy();
  UnitAction getAction(const Unit &unit, const Game &game, Debug &debug);
	
  int getRunawayDirection() const;
  int getStopRunawayTick() const;
  void setRunaway(RunawayDirection runaway_direction, int sjt);
  void decreaseStopRunawayTick();
private:
	int stop_runaway_tick_ = -1;
	RunawayDirection runaway_direction_ = NONE;
	Strategy strategy_ = Strategy();
};

#endif