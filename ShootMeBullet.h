#pragma once
#include "model/Bullet.hpp"

struct ShootMeBullet {
public:
	Bullet bullet;
	int shootMeTick;
	ShootMeBullet(Bullet b, int smt) :bullet(b), shootMeTick(smt) {}
};