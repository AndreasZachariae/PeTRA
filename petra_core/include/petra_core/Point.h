#pragma once

#include <petra_core/default.h>
#include <cmath>
#include <ostream>

// Methoden funktionieren nur für zweidimensionale Punkte
struct Point{
	public:
		/* The x and y variable are made PUBLIC because they are only limited by
		 * the float data type constraints.
		 */
		float x, y, z;
    
		/* The constructor will accept float and int data types or none at all.
		 * Two parameters sets to x,y respectively; one parameter sets both
		 * x and y to the same value.  An empty constructor sets x,y = 0.0
		 */
		Point() : x(0.0), y(0.0) ,z(0.0){};
		Point(float x, float y, float z = 0.0) : x(x), y(y), z(z) {};
		Point(float val) : x(val), y(val) ,z(val) {};
		Point(int x, int y, int z = 0) : x(static_cast<float>(x)), y(static_cast<float>(y)), z(static_cast<float>(z)) {};
		Point(int val) : x(static_cast<float>(val)), y(static_cast<float>(val)) , z(static_cast<float>(val)){};


		/* Returns the slope of two points.
		 * Formula for finding the slope(m) between 
		 * two coordinate points is: (y2-y1) / (x2-x1)
		 */
		float slope(Point p) const {
			return ((p.y-y) / (p.x-x));
		}


		/* Returns the distance between two points.
		 * Formula for finding the distance(d) between
		 * two points: square_root of [ (x2-x1)^2 + (y2-y1)^2 ]
		 */
		float distance(Point p) const {
			float d = ((p.x-x) * (p.x-x)) + ((p.y-y) * (p.y-y)) + ((p.z-z) * (p.z-z));
			return std::sqrt(d);
		}


		/* Increments the distance between two points.
		 * Formula for finding a new point between this point
		 * and point 'p' given a distance to increment:
		 * New(x) = x + (dist / square root of [ (m^2 + 1) ])
		 * New(y) = y + (m*dist / square root of [ (m^2 + 1) ])
		 */
		Point increment(Point p, float distance) {
			float m = slope(p);

			float newX = (distance / (std::sqrt((m*m) + 1))) ;
			newX = ((x < p.x) ? (x + newX) : (x - newX));

			float newY = ((m * distance) / (std::sqrt((m*m) + 1)));
			if (m >= 0.0) newY = ((y < p.y) ? (y + newY) : (y - newY));
			if (m < 0.0) newY = ((y < p.y) ? (y - newY) : (y + newY));

			return Point(newX, newY);
		}


		//Returns the magnitude of a point.
		//Distance from point to (0,0)
		float magnitude() const {
			return distance(Point(0,0));
		}


		//Returns a normalized point
		Point normalize() const {
			return Point(x/magnitude(), y/magnitude());
		}


		//Operators:

		//Addition
		Point operator+=(Point pnt){ (*this).x += pnt.x; (*this).y += pnt.y; return (*this); }
		Point operator+=(float num){ (*this).x += num; (*this).y += num; return (*this); }
		Point operator+(Point pnt) { return Point((*this).x + pnt.x, (*this).y + pnt.y);	}
		Point operator+(float num) { return Point((*this).x + num, (*this).y + num); }

		//Subtraction
		Point operator-=(Point pnt){ (*this).x -= pnt.x; (*this).y -= pnt.y; return (*this); }
		Point operator-=(float num){ (*this).x -= num; (*this).y -= num; return (*this); }
		Point operator-(Point pnt) { return Point((*this).x - pnt.x, (*this).y - pnt.y); }
		Point operator-(float num) { return Point((*this).x - num, (*this).y - num); }

		//Multiplication
		Point operator*=(Point pnt){ (*this).x *= pnt.x; (*this).y *= pnt.y; return (*this); }
		Point operator*=(float num){ (*this).x *= num; (*this).y *= num; return (*this); }
		Point operator*(Point pnt) { return Point((*this).x * pnt.x, (*this).y * pnt.y); }
		Point operator*(float num) { return Point((*this).x * num, (*this).y * num); }

		//Division
		Point operator/=(Point pnt){ (*this).x /= pnt.x; (*this).y /= pnt.y; return (*this); }
		Point operator/=(float num){ (*this).x /= num; (*this).y /= num; return (*this); }
		Point operator/(Point pnt) { return Point((*this).x / pnt.x, (*this).y / pnt.y); }
		Point operator/(float num) { return Point((*this).x / num, (*this).y / num); }

		//Equal (Assignment)
		Point operator=(Point pnt) { (*this).x = pnt.x; (*this).y = pnt.y; return (*this); }
		Point operator=(float num) { (*this).x = num; (*this).y = num; return (*this); }

		//return point as string
		std::string to_string() const {
			std::string s = "(" + std::to_string(x) + "," + std::to_string(y) + ")";
			return s;
		}
};
/*
//Comparative operators:
bool operator==(Point a, Point b) { return a.x==b.x && a.y==b.y; }
bool operator==(Point a, float num) { return a.x==num && a.y==num; }
bool operator!=(Point a, Point b) { return !(a==b); }
bool operator!=(Point a, float num) { return !(a.x==num && a.y==num); }


//Other operators:
Point operator+(float num, Point pnt) { return (pnt + num); }
Point operator*(float num, Point pnt) { return (pnt * num); }


//Allows the display of the points in a 'coordinate' format (x,y) to the output stream
std::ostream& operator<<(std::ostream& os, const Point a) {
	os << "(" << a.x << "," << a.y << ")";
	return os;
}
*/