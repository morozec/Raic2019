#pragma once
#include "model/Bullet.hpp"

struct ShootMeBullet {
public:
	Bullet bullet;
	int shootMeTick;
	int shootWallTick;
	ShootMeBullet(Bullet b, int smt, int swt) :bullet(b), shootMeTick(smt), shootWallTick(swt) {}
};