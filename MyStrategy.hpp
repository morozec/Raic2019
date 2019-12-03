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
private:
	
	Strategy strategy_ = Strategy();
};

#endif