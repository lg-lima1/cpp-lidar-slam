#pragma once

#include "configuration.h"
#include "lidar.h"
#include "olcPixelGameEngine.h"

class GUI : public olc::PixelGameEngine {
public:
	GUI(Configuration configuration);
	~GUI();

	bool OnUserCreate() override;
	bool OnUserUpdate(float fElapsedTime) override;
    bool OnUserDestroy() override;

private:
	double Degree2Rad(double degree) const;
	double ScaleDistance(double distance) const;

	olc::vd2d Transform(double angle, double distance) const;
	olc::vd2d Offset(olc::vd2d v) const;

	void SaveMap();

private:
    Lidar* m_lidar;
	olc::vi2d m_screen_size;
};
