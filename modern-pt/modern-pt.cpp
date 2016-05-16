// modern-pt.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Geometry.h"
#include "PathTracer.h"
#include <Windows.h>

class PerformanceTimer
{
public:

	double m_PCFreq;
	__int64 m_CounterStart;
	LPCTSTR m_p;

	PerformanceTimer(LPCTSTR p) :
		m_PCFreq(0),
		m_CounterStart(0),
		m_p(p)
	{
		LARGE_INTEGER li;

		if (!QueryPerformanceFrequency(&li))
			assert(false);

		m_PCFreq = double(li.QuadPart) / 1000.0;

		QueryPerformanceCounter(&li);
		m_CounterStart = li.QuadPart;

	}

	~PerformanceTimer()
	{
		LARGE_INTEGER li;
		QueryPerformanceCounter(&li);
		double Diff = double(li.QuadPart - m_CounterStart) / m_PCFreq;
		wprintf(L"\n-------------------------------------------\n");
		wprintf(L"%s execute in %12.8lf ms \n", m_p, Diff);
		wprintf(L"-------------------------------------------\n");
	}
};



int main()
{

	using namespace Geometry;

	Tracer<double> PathTracer;

	PathTracer.AddSceneObject(

		new Sphered(1e5, Vec3d(1e5 + 1, 40.8, 81.6), Vec3d(), Vec3d(.75, .25, .25), ReflT::DIFF),//Left
		new Sphered(1e5, Vec3d(-1e5 + 99, 40.8, 81.6), Vec3d(), Vec3d(.25, .25, .75), ReflT::DIFF),//Rght
		new Sphered(1e5, Vec3d(50, 40.8, 1e5), Vec3d(), Vec3d(1, 1, 1)*.999, ReflT::SPEC),//Back
		new Sphered(1e5, Vec3d(50, 40.8, -1e5 + 170), Vec3d(), Vec3d(1, 1, 1)*.999, ReflT::SPEC),//Frnt
		new Sphered(1e5, Vec3d(50, 1e5, 81.6), Vec3d(), Vec3d(.75, .75, .75), ReflT::DIFF),//Botm
		new Sphered(1e5, Vec3d(50, -1e5 + 81.6, 81.6), Vec3d(), Vec3d(.75, .75, .75), ReflT::DIFF),//Top
		new Sphered(16.5, Vec3d(27, 16.5, 47), Vec3d(), Vec3d(1, 1, 1)*.999, ReflT::SPEC),//Mirr
		new Sphered(16.5, Vec3d(73, 16.5, 78), Vec3d(), Vec3d(1, 1, 1)*.999, ReflT::REFR),//Glas
		new Sphered(1.5, Vec3d(50, 81.6 - 16.5, 81.6), Vec3d(4, 4, 4) * 100, Vec3d(), ReflT::DIFF)//Lite
		);

	{
		PerformanceTimer Timer(L"modern-pt");
		PathTracer.RenderScene<100>();
	}

	PathTracer.WritePPMImage();

    return 0;
}

