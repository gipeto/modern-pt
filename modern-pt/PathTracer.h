#pragma once

#include "Geometry.h"
#include<memory>
#include<limits>

#include <random>
#include <chrono>

#include <math.h>   // smallpt, a Path Tracer by Kevin Beason, 2008
//#include <stdlib.h> // Make : g++ -O3 -fopenmp smallpt.cpp -Origin smallpt
//#include <stdio.h>  //        Remove "-fopenmp" for g++ version < 4.2
// Usage: time ./smallpt 5000 && xv image.ppm


using namespace Geometry;
using namespace std;

template<typename T>
class Tracer
{
	using pSceneObj = unique_ptr< SceneObject<T> >;

	vector<pSceneObj> Scene;

	static T constexpr _PI = 3.1415926535897932384626433832795;
	static T constexpr _1_PI = 1.0 / _PI;

	int w = 1024 ;
	int h = 768 ;

	unique_ptr< Vec3<T>[] > cimage = make_unique< Vec3<T>[] >(w*h);
	
public:

	Tracer(int _w,int _h):
		w(_w),h(_h)
	{}

	Tracer() = default;

	template<typename First, typename ... Rest>
	inline void AddSceneObject(First && first, Rest && ... args)
	{
		Scene.push_back(pSceneObj(first));
		AddSceneObject(std::forward<Rest>(args)...);
	}

	template<typename First>
	inline void AddSceneObject(First && first)
	{
		Scene.push_back(pSceneObj(first));
	}

	inline T clamp(T x)
	{
		return x < 0 ? 0 : x > 1 ? 1 : x;
	}

	inline bool intersect(const Ray3<T> & r, T &t, int &id)
	{

		t = std::numeric_limits<T>::max();
		id = 0;

		auto cnt = 0;

		for (auto & co : Scene)
		{
			auto d = co->Intersect(r);

			if (d < t && d > 0)
			{
				id = cnt;
				t = d;
			}

			cnt++;
		}

		return t < std::numeric_limits<T>::max();
	}


	// unrolled version modified for direct light sampling
	Vec3<T> radiance_it(const Ray3<T> &r_, int depth_, std::default_random_engine &generator, std::uniform_real_distribution<T> & distribution) 
	{
		T t;                                 // distance to intersection
		int id = 0;                               // id of intersected object
		auto r = r_;
		int depth = depth_;
		// L0 = Le0 + f0*(L1)
		//    = Le0 + f0*(Le1 + f1*L2)
		//    = Le0 + f0*(Le1 + f1*(Le2 + f2*(L3))
		//    = Le0 + f0*(Le1 + f1*(Le2 + f2*(Le3 + f3*(L4)))
		//    = ...
		//    = Le0 + f0*Le1 + f0*f1*Le2 + f0*f1*f2*Le3 + f0*f1*f2*f3*Le4 + ...
		// 
		// So:
		// F = 1
		// while (1){
		//   L += F*Lei
		//   F *= fi
		// }
		
		Vec3<T> cl(0, 0, 0);   // accumulated color
		Vec3<T> cf(1, 1, 1);   // accumulated reflectance
		
		auto shadowHit = false;
		
		while (true) 
		{
			
			if (!intersect(r, t, id))
			{
				return cl; // if miss, return black
			}
		
			const auto & obj = Scene[id];        // the hit object
			Vec3<T> x = r.Origin + r.Direction*t;
			auto n = Vec3<T>(x - obj->GetPosition()).Norm();
			Vec3<T> nl = n.Dot(r.Direction) < 0 ? n : n*static_cast<T>(-1);
			auto f = obj->GetColor();
			T p = f[0]>f[1] && f[0]>f[2] ? f[0] : f[1]>f[2] ? f[1] : f[2]; // max refl
				
			auto surfType = obj->GetReflectionType();

			if ( ReflT::DIFF != surfType || !shadowHit )
			{
				cl = cl + cf * obj->GetEmission();
			}

			if (++depth > 5)
			{
				if (distribution(generator) >= p)
				{
					return cl;	
				}

				f = f * static_cast<T>( 1. / p);
			}
				
			cf = cf * f ;
			
			if (surfType == ReflT::DIFF)
			{                  // Ideal DIFFUSE reflection

				T r1 = 2 *_PI * distribution(generator);
				T r2 = distribution(generator);
				T r2s = sqrt(r2);
				auto wl = nl;
				auto u = ((abs(wl[0]) > .1 ? Vec3<T>(0, 1, 0) : Vec3<T>(1, 0, 0)) % wl).Norm();
				auto v = wl%u;
				auto d = Vec3<T>(u*cos(r1)*r2s + v*sin(r1)*r2s + wl*sqrt(1 - r2)).Norm();

				// Loop over any lights
				Vec3<T> e;
				for (auto i = 0; i< Scene.size(); i++)
				{
					const auto & s = Scene[i];

					if (!s->IsEmitting())
					{
						continue; // skip non-lights
					}

					Vec3<T> sw = s->GetPosition() - x;
					auto su = ((abs(sw[0]) > .1 ? Vec3<T>(0, 1, 0) : Vec3<T>(1, 0, 0)) % sw).Norm();
					auto sv = sw % su;

					T cos_a_max = sqrt(1 - s->GetSquareRadius() / Vec3<T>(x - s->GetPosition()).Dot(x - s->GetPosition()));
					T eps1 = distribution(generator);
					T eps2 = distribution(generator);
					T cos_a = 1 - eps1 + eps1*cos_a_max;
					T sin_a = sqrt(1 - cos_a*cos_a);
					T phi = 2 * _PI*eps2;

					auto l = Vec3<T>(su*cos(phi)*sin_a + sv*sin(phi)*sin_a + sw*cos_a).Norm();

					if (intersect(Ray3<T>(x, l), t, id) && id == i)
					{  // shadow ray
						T omega = 2 * _PI*(1 - cos_a_max);
						e = e + cf * (s->GetEmission() * l.Dot(nl) *omega) * _1_PI;  // 1/pi for brdf
					}
				}

				cl = cl + e;
				shadowHit = true;
				r = Ray3<T>(x, d);
				continue;

			}
			else if (surfType == ReflT::SPEC)
			{           // Ideal SPECULAR reflection
				r = Ray3<T>(x, r.Direction - n * 2 * n.Dot(r.Direction));
				continue;
			}

			Ray3<T> reflRay(x, r.Direction - n * 2 * n.Dot(r.Direction));     // Ideal dielectric REFRACTION
			auto into = n.Dot(nl) > 0;                // Ray from outside going in?
			T nc = 1;
			T nt = 1.5;
			T nnt = into ? nc / nt : nt / nc;
			T ddn = r.Direction.Dot(nl);
			T cos2t = 1 - nnt*nnt*(1 - ddn*ddn);
			if ( cos2t < 0) 
			{    // Total internal reflection												
				r = reflRay;
				continue;
			}

			auto tdir = Vec3<T>(r.Direction*nnt - n*((into ? 1 : -1)*(ddn*nnt + sqrt(cos2t)))).Norm();
			T a = nt - nc;
			T b = nt + nc;
			T R0 = a*a / (b*b);
			T c = 1 - (into ? -ddn : tdir.Dot(n));
			T Re = R0 + (1 - R0)*c*c*c*c*c;
			T Tr = 1 - Re;
			T P = .25 + .5*Re;
			T RP = Re / P;
			T TP = Tr / (1 - P);
			
			if (distribution(generator)<P) 
			{
				cf = cf*RP;
				r = reflRay;
			}
			else 
			{
				cf = cf*TP;
				r = Ray3<T>(x, tdir);
			}
			
			continue;
		}
	}

	/*
	// Original recursive version with direct light sampling
	inline Vec3<T> radiance(const Ray3<T> &r, int depth, std::default_random_engine &generator, std::uniform_real_distribution<T> & distribution, int E)
	{
		T t;                               // distance to intersection
		int id = 0;                               // id of intersected object
		if (!intersect(r, t, id))
		{
			return Vec3<T>(); // if miss, return black
		}
		
		const auto & obj = Scene[id];        // the hit object
		Vec3<T> x = r.Origin + r.Direction*t;
		auto n = Vec3<T>(x - obj->GetPosition()).Norm();
		Vec3<T> nl = n.Dot(r.Direction) < 0 ? n : n*-1;
		auto f = obj->GetColor();
		auto p = f[0] > f[1] && f[0]>f[2] ? f[0] : f[1] > f[2] ? f[1] : f[2]; // max refl
		
		if (++depth > 5 || 0 == p)
		{
			if (distribution(generator) >= p)
			{
				return  obj->GetEmission()*E;
			}
	
			f = f*(1 / p);
		}

		if (depth > 512)
		{
			return obj->GetEmission();
		}
		
				
		if (obj->GetReflectionType() == ReflT::DIFF) 
		{                  // Ideal DIFFUSE reflection
			
			auto r1  = 2 * M_PI * distribution(generator);
			auto r2  = distribution(generator);
			auto r2s = sqrt(r2);
			auto wl = nl;
			auto u = ((abs(wl[0]) > .1 ? Vec3<T>(0, 1, 0) : Vec3<T>(1, 0, 0)) % wl).Norm();
			auto v = wl%u;
			auto d = Vec3<T>(u*cos(r1)*r2s + v*sin(r1)*r2s + wl*sqrt(1 - r2)).Norm();

			// Loop over any lights
			Vec3<T> e;
			for (auto i = 0; i< Scene.size() ; i++) 
			{
				const auto & s = Scene[i];
				
				if (!s->IsEmitting())
				{
					continue; // skip non-lights
				}
				
				Vec3<T> sw = s->GetPosition() - x;
				auto su = ((abs(sw[0]) > .1 ? Vec3<T>(0, 1, 0) : Vec3<T>(1,0,0) ) % sw).Norm();
				auto sv = sw % su;
				
				auto cos_a_max = sqrt(1 - s->GetSquareRadius() / Vec3<T>(x - s->GetPosition()).Dot(x - s->GetPosition()));
				auto eps1 = distribution(generator);
				auto eps2 = distribution(generator);
				auto cos_a = 1 - eps1 + eps1*cos_a_max;
				auto sin_a = sqrt(1 - cos_a*cos_a);
				auto phi = 2 * M_PI*eps2;
				
			    auto l = Vec3<T>(su*cos(phi)*sin_a + sv*sin(phi)*sin_a + sw*cos_a).Norm();
		
				if (intersect(Ray3<T>(x, l), t, id) && id == i) 
				{  // shadow ray
					auto omega = 2 * M_PI*(1 - cos_a_max);
					e = e + f * (s->GetEmission() * l.Dot(nl) *omega ) * M_1_PI;  // 1/pi for brdf
				}
			}

			return obj->GetEmission() * E + e + f * radiance(Ray3<T>(x, d), depth, generator, distribution, 0);
		}
		else if (obj->GetReflectionType() == ReflT::SPEC)
		{ // Ideal SPECULAR reflection
			return obj->GetEmission() + f *  radiance(Ray3<T>(x, r.Direction - n * 2 * n.Dot(r.Direction)), depth, generator, distribution,1);
		}
		
		Ray3<T> reflRay(x, r.Direction - n * 2 * n.Dot(r.Direction));     // Ideal dielectric REFRACTION
		auto into = n.Dot(nl) > 0;                // Ray from outside going in?
		T nc = 1; 
		T nt = 1.5;
		auto nnt = into ? nc / nt : nt / nc;
		auto ddn = r.Direction.Dot(nl);
		T cos2t = 1 - nnt*nnt*(1 - ddn*ddn);
		
		if (cos2t < 0)    // Total internal reflection
		{
			return obj->GetEmission() + f * radiance(reflRay, depth, generator,distribution,1) ;
		}
		
		auto tdir = Vec3<T>( r.Direction*nnt - n*((into ? 1 : -1)*(ddn*nnt + sqrt(cos2t)))).Norm();
		auto a = nt - nc;
		auto b = nt + nc;
		auto R0 = a*a / (b*b);
		auto c = 1 - (into ? -ddn : tdir.Dot(n) );
		auto Re = R0 + (1 - R0)*c*c*c*c*c;
		auto Tr = 1 - Re;
		auto P = .25 + .5*Re;
		auto RP = Re / P;
		auto TP = Tr / (1 - P);
		
		
		if (depth > 2)
		{
			if (distribution(generator) < P)
			{
				return obj->GetEmission() + f * radiance(reflRay, depth, generator, distribution,1) * RP;
			}
			return obj->GetEmission() + f * radiance(Ray3<T>(x, tdir), depth, generator, distribution,1)*TP;
		}

		return obj->GetEmission() + f * (radiance(reflRay, depth, generator, distribution,1)*Re + radiance(Ray3<T>(x, tdir), depth, generator, distribution,1)*Tr);



}


     // Original recursive version 
	inline Vec3<T> radiance(const Ray3<T> & r, int depth, std::default_random_engine &generator, std::uniform_real_distribution<T> & distribution)
	{

		T t;                               // distance to intersection
		int id = 0;                       // id of intersected object

		if (!intersect(r, t, id))
		{
			return Vec3<T>(); // if miss, return black
		}

		auto obj = Scene[id];        // the hit object

		Vec3<T> x = r.Origin + r.Direction*t;

		Vec3<T> n = Vec3<T>(x - obj->GetPosition()).Norm();

		Vec3<T> nl = n.Dot(r.Direction) < 0 ? n : n*-1;
		Vec3<T> f = obj->GetColor();
		T p = (f[0] > f[1] && f[0] > f[2]) ? f[0] : (f[1] > f[2]) ? f[1] : f[2]; // max refl

		if (++depth > 5)
		{
			if (distribution(generator) < p)
			{
				f = f*(1 / p);
			}
			else
			{
				return obj->GetEmission();; //R.R.
			}
		}

		if (depth > 512)
		{
			return obj->GetEmission();
		}

		if (ReflT::DIFF == obj->GetReflectionType())
		{
			// Ideal DIFFUSE reflection
			T r1 = 2 * M_PI*distribution(generator);
			T r2 = distribution(generator);
			T r2s = sqrt(r2);
			Vec3<T> wl = nl;
			Vec3<T> u = ((abs(wl[0]) > .1 ? Vec3<T>(0, 1, 0) : Vec3<T>(1, 0, 0)) % wl).Norm();
			//u.Norm();
			Vec3<T> v = wl%u;
			auto d = Vec3<T>(u*cos(r1)*r2s + v*sin(r1)*r2s + wl*sqrt(1 - r2)).Norm();// .Norm();


			return obj->GetEmission() + f * radiance(Ray3<T>(x, d), depth, generator, distribution);
		}
		else if (ReflT::SPEC == obj->GetReflectionType()) // Ideal SPECULAR reflection
		{
			return obj->GetEmission() + f * radiance(Ray3<T>(x, r.Direction - n * 2 * n.Dot(r.Direction)), depth, generator, distribution);
		}

		Ray3<T> reflRay(x, r.Direction - n * 2 * n.Dot(r.Direction));     // Ideal dielectric REFRACTION
		bool into = n.Dot(nl) > 0;                      // Ray from outside going in?
		T nc = 1;
		T nt = 1.5;
		T nnt = into ? nc / nt : nt / nc;
		T ddn = r.Direction.Dot(nl);
		T cos2t;

		if ((cos2t = 1 - nnt*nnt*(1 - ddn*ddn)) < 0)    // Total internal reflection
		{
			return obj->GetEmission() + f * radiance(reflRay, depth, generator, distribution);

		}

		auto tdir = Vec3<T>(r.Direction*nnt - n*((into ? 1 : -1)*(ddn*nnt + sqrt(cos2t)))).Norm();
		//tdir.Norm();
		T a = nt - nc;
		T b = nt + nc;
		T R0 = a*a / (b*b);
		T c = 1 - (into ? -ddn : tdir.Dot(n));
		T Re = R0 + (1 - R0)*c*c*c*c*c;
		T Tr = 1 - Re;
		T P = .25 + .5*Re;
		T RP = Re / P;
		T TP = Tr / (1 - P);


		if (depth > 2)
		{
			if (distribution(generator) < P)
			{
				return obj->GetEmission() + f * radiance(reflRay, depth, generator, distribution) * RP;
			}
			return obj->GetEmission() + f * radiance(Ray3<T>(x, tdir), depth, generator, distribution)*TP;
		}

		return obj->GetEmission() + f * (radiance(reflRay, depth, generator, distribution)*Re + radiance(Ray3<T>(x, tdir), depth, generator, distribution)*Tr);


	}

	*/
	
template<int spp = 40>
	void RenderScene()
	{

		Ray3<T> cam(Vec3<T>(50, 52, 295.6), Vec3<T>(0, -0.042612, -1).Norm()); // cam pos, dir
		Vec3<T> cx = Vec3<T>(w*.5135 / h, 0, 0);
		Vec3<T> cy = Vec3<T>(cx%cam.Direction).Norm();

		cy = cy *0.5135;
		Vec3<T> r;
		
		typedef std::chrono::high_resolution_clock myclock;


#pragma omp parallel for schedule(dynamic, 1) private(r)       // OpenMP

		for (auto y = 0; y < h; y++)
		{

			std::uniform_real_distribution<T> distribution;// (0.0, 1.0);
			std::default_random_engine			   generator;
			std::random_device                     rd;

			// Loop over image rows
			fprintf(stderr, "\rRendering (%d spp) %5.2f%%", spp * 4, 100.*y / (h - 1));

			for (auto x = 0; x < w; x++)   // Loop cols
			{
				for (int sy = 0, i = (h - y - 1)*w + x; sy < 2; sy++)
				{
					cimage[i] = {};

					generator.seed(rd());
					// 2x2 subpixel rows
					for (auto sx = 0; sx < 2; sx++, r = Vec3<T>())
					{
						// 2x2 subpixel cols
						for (auto s = 0; s < spp; s++)
						{
							T r1 = 2 * distribution(generator);
							T dx = r1 < 1 ? sqrt(r1) - 1 : 1 - sqrt(2 - r1);
							T r2 = 2 * distribution(generator);
							T dy = r2 < 1 ? sqrt(r2) - 1 : 1 - sqrt(2 - r2);
							auto d = Vec3<T>(cx * (((sx + .5 + dx) / 2 + x) / w - .5) + cy * (((sy + .5 + dy) / 2 + y) / h - .5) + cam.Direction).Norm();
							r = r + radiance_it(Ray3<T>(cam.Origin + d * 140, d), 0, generator, distribution);
					
						} // Camera rays are pushed ^^^^^ forward to start in interior

						r = r * static_cast<T>(1. / static_cast<T>(spp));

						cimage[i] = cimage[i] + Vec3<T>(clamp(r[0]), clamp(r[1]), clamp(r[2]))* static_cast<T>(.25);
					}
				}
			}
		}

	}

	inline void WritePPMImage()
	{
		FILE *f;

		fopen_s(&f, "image.ppm", "w");         // Write image to PPM file.
		fprintf(f, "P3\n%d %d\n%d\n", w, h, 255);
		for (auto i = 0; i < w*h; i++)
			fprintf(f, "%d %d %d ", toInt(cimage[i][0]), toInt(cimage[i][1]), toInt(cimage[i][2]));

		fclose(f);
	}


	inline int toInt(T x)
	{
		return static_cast<int>(pow(clamp(x), 1 / 2.2) * 255 + .5);
	}


};



