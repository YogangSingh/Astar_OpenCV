//A Star pathfinding algorithm for OpenCV
//	features:
//		variable A* coefficient: adjustable steering towards destination
//		wall avoidance
//by L?rinc Serf?z?, 2015
//license: http://creativecommons.org/licenses/by-sa/4.0/

#pragma once
#include "opencv2\core.hpp"
#include "opencv2\imgproc.hpp"
#include <cmath>
#include <vector>
#include <queue>
#include <map>
#include <string>

using namespace std;
using namespace cv;

class Pathfinding {

private:
	//twodimensional, integer-based point structure, contains additional variables for the pathfinding calculation
	struct Point2A {

		//x, y, the coordinates of the point
		//dir is the direction from previous point. see direction codings below
		int x, y, dir;

		//level: the cost of the route from start to this point
		//fscore: the essence of the A* algorithm, value is : [level] + [in air distance from destination] * astar_weight
		float fScore, level;

		//constructors
		Point2A() :x(0), y(0), fScore(0), dir(0) {};
		Point2A(int _x, int _y, float _level=0.f, int _dir=0) :x(_x), y(_y), level(_level), fScore(0), dir(_dir) {};

		//== operator overload
		bool operator==(const Point2A other) ;
	};
	//CompByPos : struct used in the stl map<Point2A, Point2A> during the pathfinding
	//it only contains a comparator function
	//we need this, because every point is unique by position, but not by fscore
	struct CompByPos {

		bool operator()(const Point2A a, const Point2A b) const;
	};

	//CompByFScore : contains a comparating function, which works by fscore
	//it gives priority for the smaller fScore
	//used in stl priority_queue<Point2A>
	struct CompByFScore {

		bool operator()(const Point2A a, const Point2A b);
	};

	//mapimg is the map got, pathmap is the same, but the pixels of the path are colored
	//pathmap is only valid after calculatepath
	//blurmap is matimg blurred with opencv function, it's used in keeping away from walls
	Mat mapimg, pathmap, blurmap;

	//astar_weight is the multiplier of A* coefficient
	//wall_weight is used in the keeping away from walls feature
	float astar_weight, wall_weight;

	//no comment
	Point2A start, dest;

	//diagonal decides if a pixel(which is a node) has 4 or 8 neighbours
	//see direction coding below
	//calculated decides if the Pathfinding object has a valid path for the current map and settings
	bool diagonal, calculated;

	//mrows and mcols refer to the size of mapimg
	//blur size is used in the wall avoiding feature 
	int mrows, mcols, blur_size;

	//stores the list of directions for the path
	string dir;

	//calculates euclidean distance between a and b points
	float Distance(Point2A a, Point2A b);

	//returns an array of the points surrounding point p
	//the length of the array is not constant, because the function performs
	//OnMap checks too. use arraySize ref variable to get the size of the array returned
	Point2A* GetNeighbors(Point2A p, int& arraySize);

	//as the name says, this function sets defaut values
	void InitValues();

	//checks if point p is wall
	//currently the class only supports black/white maps, where black pixels are wall.
	bool IsWall(Point2A p);

	//function to decide if the coordinates of this point are on the map or not
	bool OnMap(int x, int y);

public:
	enum ErrorCodes {NoError = 0, NoMap, StartIsWall, DestIsWall, NoPath, AlreadyCalculated };
	static const int diagonalDirX[];
	static const int diagonalDirY[];
	static const int nonDiagonalDirX[];
	static const int nonDiagonalDirY[];
	//constructor: sets default values diagonal=true, astar coefficient 0.3
	Pathfinding();

	//constructor, argument map is the map on which the algorithm is evaluated
	Pathfinding(Mat map, bool _diagonal = true);

	//as the function name suggests, this can set the opencv mat image as the map
	void SetMap(Mat map);

	//sets the A* pathfinding coefficient. 0.f means Dijkstra's algorithm, anything else is A* (positive values recommended).
	//The bigger the value, the more the algorithm steers towards the destination
	//but setting it too high can result in suboptimal path
	//after changing that, have to call CalculatePath again
	void SetAWeight(float weight);

	//if set to true, each pixel has 8 connected neighbor, else only 4 - see GetDirections() comment
	//after changing that, have to call CalculatePath again
	void SetDiagonal(bool _diagonal);

	//sets the value of how much the algorithm tries to avoid going near walls. 
	//weight: the amount the walls push away the route. default 10.f
	//0.f disables the feature
	//avoidZoneLevel: the size of the zone surrounding the walls, in which the upper effect works. default: 5
	void SetWallWeight(float weight, int avoidZoneLevel);

	//sets the start point. the coordinate system is the OpenCV/image default, the origin is the upper left corner of the image.
	//start and destination points have to be set after the map image!
	void SetStart(int x, int y);
	void SetDestination(int x, int y);

	//returns the map, on which the calculated path is marked red
	//call this after CalculatePath(), otherwise returns empty map
	Mat GetPathMap();

	//returns a std::string of numbers, which represent the directions along the path. Direction coding (relative to p):
	//call after CalculatePath()
	//if diagonal is set to true				if diagonal == false
	//		[0]	[1]	[2]									[3]
	//		[3]	[p]	[4]								[2]	[p]	[0]
	//		[5]	[6]	[7]									[1]
	string GetDirections();

	//evaluates the algorithm. It's a separate function because it takes some time
	//check out the ErrorCodes enum to decode the returned values
	ErrorCodes CalculatePath();
};