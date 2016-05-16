#pragma once

#include <assert.h>
#include <type_traits>
#include<vector>


namespace Geometry
{

	
	struct Plus
	{
		template<typename T, typename Left, typename Right>
		inline static T eval(const Left &lhs, const Right& rhs, unsigned int i)
		{
			return lhs[i] + rhs[i];
		}
	};

	struct Minus
	{
		template<typename T, typename Left, typename Right>
		inline static T eval(const Left &lhs, const Right& rhs, unsigned int i)
		{
			return lhs[i] - rhs[i];
		}
	};

	struct Times
	{
		template<typename T, typename Left, typename Right>
		inline static T eval(const Left &lhs, const Right& rhs, unsigned int i)
		{
			return lhs[i] * rhs[i];
		}
	};

	struct Scale
	{
		template<typename T, typename Left, typename Right>
		inline static T eval(const Left &lhs, const Right rhs, unsigned int i)
		{
			return lhs[i] * rhs;
		}
	};


	template<bool B,typename T>
	struct ScalarStorageHelper
	{
		typedef const T type;
	};

	template<typename T>
	struct ScalarStorageHelper<false,T>
	{
		typedef const T& type;
	};

	
	template<typename T, class Left, class Operator, class Right>
	class Expression
	{
	
		// Scalar types have to be stored by value
		typedef typename ScalarStorageHelper<std::is_same<T,Right>::value && std::is_fundamental<Right>::value, Right>::type RightType;
		
		const Left  &  left;
		RightType      right;

	public:

		typedef T type;

		Expression() = delete;

		Expression(const Left& lhs, RightType rhs)
			: left(lhs), right(rhs)
		{}

		inline auto operator[](unsigned int i) const -> T
		{
			return Operator::eval<T>(left, right, i);
 		}


		template<typename E>
		auto operator +(const E & rhs) const ->Expression<T, decltype(*this), Plus, E>
		{
			return Expression<T, decltype(*this), Plus, E>(*this, rhs);
		}

		template<typename E>
		auto operator -(const E & rhs) const ->Expression<T, decltype(*this), Minus, E>
		{
			return Expression<T, decltype(*this), Minus, E>(*this, rhs);
		}


		template<typename E>
		auto operator *(const E & rhs) const -> Expression<T, decltype(*this), Times, E> 
		{
			return Expression<T, decltype(*this), Times, E>(*this, rhs);
		}

		
		auto operator *(const T rhs) const ->Expression<T, decltype(*this), Scale, T>
		{
			return Expression<T, decltype(*this), Scale, T>(*this, rhs);
		}
	

	};

	
	template<typename T, typename E>
	typename std::enable_if<std::is_fundamental<T>::value, Expression<typename E::type, E, Scale, typename E::type> >::type
	operator * (const T scalar, const E & expr)
	{
		return Expression<typename E::type, E, Scale, typename E::type>(expr, static_cast<typename E::type>(scalar));
	}


	template<typename T, int N, typename ... Args>
	class VecBase 
	{ 

		typedef T type;
	
		template<int CNT>
		struct Loop
		{
			template<class Left, class Right>
			inline static void Assign(Left & lhs, const Right & rhs)
			{
				lhs[CNT - 1] = rhs[CNT - 1];
				Loop<CNT - 1>::Assign(lhs, rhs);
			}

			template<class Left>
			inline static void Assign(Left & lhs, const T rhs)
			{
				lhs[CNT - 1] = rhs;
				Loop<CNT - 1>::Assign(lhs, rhs);
			}

			template<typename T,class Operator, class Left, class Right>
			inline static T Reduce(Left & lhs, const Right & rhs)
			{
				return Operator::eval<T>(lhs, rhs, CNT - 1) + Loop<CNT - 1>::Reduce<T,Operator,Left,Right>(lhs, rhs);
			}

		};

		template<>
		struct Loop<1>
		{
			template<class Left, class Right>
			inline static void Assign(Left & lhs, const Right & rhs)
			{
				lhs[0] = rhs[0];
			}

			template<class Left>
			inline static void Assign(Left & lhs, const T rhs)
			{
				lhs[0] = rhs;
			}


			template<typename T,class Operator, class Left, class Right>
			inline static T Reduce(Left & lhs, const Right & rhs)
			{
				return Operator::eval<T>(lhs, rhs, 0) ;
			}

		};

	
		//static constexpr auto N = sizeof...(Args); // Nice but doesn't allow default constructor
		T m_Data[N] = { 0 };

	public:

		VecBase()
		{}

		VecBase(Args... x)
			: m_Data{ x... }
		{}

		VecBase(const VecBase &rhs)
		{
			Loop<N>::Assign(m_Data, rhs.m_Data);
		}

		VecBase & operator=(const VecBase &rhs)
		{
			Loop<N>::Assign(m_Data, rhs.m_Data);
			return *this;
		}

		template<unsigned int n>
		inline T Get()
		{
			static_assert(n < N, "Index out of bound");
			return m_Data[n];
		}

		template<unsigned int n>
		inline void Set(T Value)
		{
			static_assert(n < N, "Index out of bound");
			m_Data[n] = Value;
		}

		inline T & operator[](unsigned int n)
		{
			assert(n < N);
			return m_Data[n];
		}

		inline T operator[](unsigned int n) const
		{
			assert(n < N);
			return m_Data[n];
		}


		T Dot(const VecBase & rhs)
		{
			return Loop<N>::Reduce<T, Times>(m_Data, rhs);
		}

		T Dot(const VecBase & rhs) const
		{
			return Loop<N>::Reduce<T, Times>(m_Data, rhs);
		}


		VecBase& Norm()
		{
			return *this = *this  * (1 / sqrt(Loop<N>::Reduce<T, Times>(m_Data, m_Data)));
		}

		template<typename RightType>
		typename std::enable_if<3==N && std::is_same<RightType,VecBase>::value,VecBase>::type operator %(const RightType & rhs)
		{
			return VecBase(m_Data[1] * rhs[2] - m_Data[2] * rhs[1], m_Data[2] * rhs[0] - m_Data[0] * rhs[2], m_Data[0] * rhs[1] - m_Data[1] * rhs[0]);
		} 

		
		// Expression methods	
		template<class E>
		VecBase & operator=(const E & rhs)
		{
			Loop<N>::Assign(m_Data, rhs);
			return *this;
		}

		
		template<class E>
		VecBase(const E & rhs)
		{
			Loop<N>::Assign(m_Data, rhs);
		}

		template <class Right>
		auto operator+(const Right& rhs)->Expression<T, decltype(*this), Plus, Right>
		{
			return Expression<T, decltype(*this), Plus, Right>(*this, rhs);
		}

		template <class Right>
		auto operator+(const Right& rhs) const ->Expression<T, decltype(*this), Plus, Right>
		{
			return Expression<T, decltype(*this), Plus, Right>(*this, rhs);
		}


		template <class Right>
		auto operator-(const Right& rhs)->Expression<T, decltype(*this), Minus, Right>
		{
			return Expression<T, decltype(*this), Minus, Right>(*this, rhs);
		}

		template <class Right>
		auto operator-(const Right& rhs) const->Expression<T, decltype(*this), Minus, Right>
		{
			return Expression<T, decltype(*this), Minus, Right>(*this, rhs);
		}

		template <class Right>
		auto operator*(const Right& rhs)-> typename std::enable_if<!std::is_fundamental<Right>::value, Expression<T, decltype(*this), Times, Right> >::type
		{
			return Expression<T, decltype(*this), Times, Right>(*this, rhs);
		}

		template <class Right>
		auto operator*(const Right& rhs) const -> typename std::enable_if<!std::is_fundamental<Right>::value, Expression<T, decltype(*this), Times, Right> >::type
		{
			return Expression<T, decltype(*this), Times, Right>(*this, rhs);
		}

		template<typename ScalarType>
		auto operator*(const ScalarType rhs)->typename std::enable_if<std::is_fundamental<ScalarType>::value, Expression<T, decltype(*this), Scale, T> >::type
		{
			return Expression<T, decltype(*this), Scale, T>(*this, static_cast<T>(rhs));
		}

		template<typename ScalarType>
		auto operator*(const ScalarType rhs) const ->typename std::enable_if<std::is_fundamental<ScalarType>::value, Expression<T, decltype(*this), Scale, T> >::type
		{
			return Expression<T, decltype(*this), Scale, T>(*this, static_cast<T>(rhs));
		}
		
	};


	// Generate parameter pack made of N variables of type T and use it to define  class C type
	template< template<typename, int, typename...> class C, typename T, int CNT, int N, typename...Args>
	struct PackTHelper
	{
		typedef typename PackTHelper<C, T, CNT - 1, N, T, Args...>::type type;
	};

	template<template<typename, int, typename...> class C, typename T, int N, typename...Args>
	struct PackTHelper< C, T, 0, N, Args...>
	{
		typedef C< T, N, Args...> type;
	};

	template<typename T, int N>
	using Vec = typename PackTHelper<VecBase, T, N, N>::type;

	template<typename T>
	using Vec3 = typename PackTHelper<VecBase, T, 3, 3>::type;
	
	using Vec3d =  PackTHelper<VecBase, double, 3, 3>::type;
	using Vec3f =  PackTHelper<VecBase, float, 3, 3>::type;


	template<typename T,int N>
	struct Ray
	{
		Vec<T,N> Origin;
		Vec<T,N> Direction;
		
		Ray(Vec<T,N> Origin_, Vec<T,N> Direction_)
			: Origin(Origin_), 
			  Direction(Direction_)
		{}
	};

	template<typename T>
	using Ray3 = Ray<T,3>;

	using Ray3d = Ray<double, 3>;
	using Ray3f = Ray<float, 3>;

	enum class ReflT { DIFF, SPEC, REFR };  // material types, used in radiance()

	
	template<typename T>
	class SceneObject 
	{
		Vec3<T> m_Emission;
		Vec3<T> m_Color;
		ReflT   m_ReflectionType;

	public:


		SceneObject(const Vec3<T> & Emission, const Vec3<T> & Color, ReflT RefType) :
				m_Emission(Emission),
				m_Color(Color),
				m_ReflectionType(RefType)
			{}

		Vec3<T> GetColor()
		{
			return m_Color;
		}

		Vec3<T> GetEmission()
		{
			return m_Emission;
		}

		inline bool IsEmitting()
		{
			return m_Emission[0] > 0 || m_Emission[1] > 0 || m_Emission[2] > 0;
		}


		ReflT GetReflectionType()
		{
			return m_ReflectionType;
		}

		virtual T Intersect(const Ray3<T> & Ray)   =  0;
		virtual Vec3<T> GetPosition() = 0;
		virtual T GetSquareRadius() = 0;

	};



	template<typename T>
	class Sphere : public SceneObject<T>
	{
		
		T       m_Radius2 = 0; 
		Vec3<T> m_Position;
		
		static constexpr T eps = 1e-4;

	
	public:

 
		Sphere(T Radius, const Vec3<T> & Position, const Vec3<T> & Emission, const Vec3<T> &Color, ReflT RefType) :
			m_Radius2(Radius*Radius),
			m_Position(Position),
			SceneObject(Emission, Color , RefType)
		{}

		virtual Vec3<T> GetPosition() override
		{
			return m_Position;
		}

		virtual T GetSquareRadius() override
		{
			return m_Radius2;
		}


		virtual T Intersect(const Ray3<T> & Ray)  override  // returns distance, 0 if nohit
		{
			Vec3<T> op = m_Position - Ray.Origin; // Solve t^2*Direction.Direction + 2*t*(Origin-p).Direction + (Origin-p).(Origin-p)-R^2 = 0
			
			auto b = op.Dot(Ray.Direction);
			auto det = b*b - op.Dot(op) + m_Radius2;
	
			if (det < 0)
			{
				return 0;
			}
	
			det = sqrt(det);
	
			auto t1 = b - det;
				
			if (t1 > eps)
			{
				return t1;
			}
	
			auto t2 = b + det;
	
			if (t2 > eps)
			{
				return t2;
			}
	
			return 0;	
		}
	};

	using Sphered = Sphere<double>;
	using Spheref = Sphere<float>;

}