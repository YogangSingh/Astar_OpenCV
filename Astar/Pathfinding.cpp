#include "Pathfinding.h"

bool Pathfinding::Point2A::operator==(const Point2A other) {

	return x == other.x && y == other.y;
}

bool Pathfinding::CompByPos::operator()(const Point2A a, const Point2A b) const {

	if (a.x == b.x)
		return a.y > b.y;
	else
		return a.x > b.x;
}

bool Pathfinding::CompByFScore::operator()(const Point2A a, const Point2A b) {

	return a.fScore > b.fScore;
}

float Pathfinding::Distance(Point2A a, Point2A b) {

	float x = static_cast<float>(a.x - b.x);
	float y = static_cast<float>(a.y - b.y);
	return sqrtf(x*x + y*y);
}

Pathfinding::Point2A* Pathfinding::GetNeighbors (Point2A p, int& arraySize) {

	arraySize = 0;
	uchar size;

	if (diagonal)
		size = 8;
	else
		size = 4;

		Point2A* ret = new Point2A[size];
		for (int i = 0; i < size; i++) {
			
			int x, y;
			if (diagonal) {

				x = p.x + diagonalDirX[i];
				y = p.y + diagonalDirY[i];
			}
			else {
				x = p.x + nonDiagonalDirX[i];
				y = p.y + nonDiagonalDirY[i];
			}

			if (!OnMap(x, y))
				continue;

			float level = p.level + 1.f + (255 - blurmap.at<Vec3b>(y, x)[2]) / 255.f * wall_weight;
			Point2A n = Point2A(x, y, level, i);

			if (diagonal && (i == 0 || i == 2 || i == 5 || i == 7))
				n.level += 0.414213f;

			ret[arraySize] = n;
			arraySize++;			
		}
		return ret;
}

void Pathfinding::InitValues() {

	astar_weight = 0.3f;
	wall_weight = 10.f;
	blur_size = 11;
	diagonal = true;
	calculated = false;
}

bool Pathfinding::IsWall(Point2A p) {

	if (mapimg.at<Vec3b>(p.y, p.x) == Vec3b(0, 0, 0))
		return true;
	return false;
}

bool Pathfinding::OnMap(int x, int y) {

	if (x >= 0 && y >= 0 && x < mcols && y < mrows)
		return true;
	return false;
}

const int Pathfinding::diagonalDirX[] = { -1, 0, 1, -1, 1, -1, 0, 1 };
const int Pathfinding::diagonalDirY[] = { -1, -1, -1, 0, 0, 1, 1, 1 };
const int Pathfinding::nonDiagonalDirX[] = { 1, 0, -1, 0 };
const int Pathfinding::nonDiagonalDirY[] = { 0, 1, 0, -1 };

Pathfinding::Pathfinding() {

	InitValues();
}

Pathfinding::Pathfinding(Mat map, bool _diagonal) {

	InitValues();
	SetMap(map);
	diagonal = _diagonal;
}

void Pathfinding::SetMap(Mat map) {

	if (!map.empty()) {

		mapimg = map;
		calculated = false;
		mrows = map.rows;
		mcols = map.cols;
		GaussianBlur(mapimg, blurmap, Size(blur_size, blur_size), 0);
	}
}

void Pathfinding::SetAWeight(float weight) {

	if (astar_weight != weight) {

		astar_weight = weight;
		calculated = false;
	}
}

void Pathfinding::SetDiagonal(bool _diagonal) {

	if (diagonal != _diagonal) {

		diagonal = _diagonal;
		calculated = false;
	}
}

void Pathfinding::SetWallWeight(float weight, int avoidZoneLevel) {

	if (wall_weight == weight && blur_size == 2 * avoidZoneLevel + 1)
		return;

	wall_weight = weight;
	if (avoidZoneLevel >= 0)
		blur_size = 2 * avoidZoneLevel + 1;
	calculated = false;
}

void Pathfinding::SetStart(int x, int y) {

	if (!mapimg.empty()) {

		if (OnMap(x, y)) {

			start = Point2A(x, y);
			calculated = false;
		}
	}
}

void Pathfinding::SetDestination(int x, int y) {

	if (!mapimg.empty()) {

		if (OnMap(x, y)) {

			dest = Point2A(x, y);
			calculated = false;
		}
	}
}

Mat Pathfinding::GetPathMap() {

	if (calculated) return pathmap;
	else return Mat();
}

string Pathfinding::GetDirections() {

	if (calculated) return dir;
	else return string();
}

Pathfinding::ErrorCodes Pathfinding::CalculatePath() {

	if (calculated)
		return AlreadyCalculated;

	if (mapimg.empty())
		return NoMap;

	if (IsWall(start))
		return StartIsWall;

	if (IsWall(dest))
		return DestIsWall;

	dir = string();
	mapimg.copyTo(pathmap);
	int **closedSet = new int*[mrows];
	float **openSet = new float*[mrows];
	for (int i = 0; i < mrows; i++) {

		closedSet[i] = new int[mcols];
		openSet[i] = new float[mcols];
		for (int j = 0; j < mcols; j++) {

			closedSet[i][j] = 0;
			openSet[i][j] = -1.0f;
		}
	}

	priority_queue<Pathfinding::Point2A, vector<Point2A>, CompByFScore> openSetQue[2];
	int osq = 0;
	map<Pathfinding::Point2A, Pathfinding::Point2A, CompByPos> cameFrom;

	start.fScore = Distance(start, dest);
	openSetQue[osq].push(start);
	openSet[start.y][start.x] = 0.0f;
	while (openSetQue[osq].size() != 0) {
		Point2A current = openSetQue[osq].top();
		if (current == dest) {
			while (cameFrom.size() != 0) {

				pathmap.at<Vec3b>(current.y, current.x) = Vec3b(0, 0, 255);
				dir = to_string(current.dir) + dir;
				auto it = cameFrom.find(current);
				Point2A keytmp = current;
				if (it == cameFrom.end()) {
					for (int i = 0; i < mrows; i++) {

						delete openSet[i];
						delete closedSet[i];
					}
					delete openSet;
					delete closedSet;
					calculated = true;
					dir = dir.substr(1, dir.length() - 1);
					return NoError;
				}
				current = cameFrom[current];
				cameFrom.erase(keytmp);
			}
		}
		openSetQue[osq].pop();
		closedSet[current.y][current.x] = 1;
		int arraySize;
		Point2A *neighbors = GetNeighbors(current, arraySize);

		for (int i = 0; i < arraySize; i++) {

			Point2A neighbor = neighbors[i];

			if (closedSet[neighbor.y][neighbor.x] == 1)
				continue;

			if (IsWall(neighbor)) {

				closedSet[neighbor.y][neighbor.x] = 1;
				continue;
			}
			float ngScore = neighbor.level;

			if (openSet[neighbor.y][neighbor.x] == -1.0f || ngScore < openSet[neighbor.y][neighbor.x]) {

				cameFrom[neighbor] = current;
				neighbor.fScore = ngScore + Distance(neighbor, dest) * astar_weight;

				if (openSet[neighbor.y][neighbor.x] == -1.0f) {
					openSet[neighbor.y][neighbor.x] = ngScore;
					openSetQue[osq].push(neighbor);
				}
				else {
					openSet[neighbor.y][neighbor.x] = ngScore;
					while (!(neighbor == openSetQue[osq].top())) {

						openSetQue[1 - osq].push(openSetQue[osq].top());
						openSetQue[osq].pop();
					}
					openSetQue[osq].pop();
					if (openSetQue[osq].size() >= openSetQue[1 - osq].size()) {
						osq = 1 - osq;
					}
					while (!openSetQue[osq].empty()) {
						openSetQue[1 - osq].push(openSetQue[osq].top());
						openSetQue[osq].pop();
					}
					osq = 1 - osq;
					openSetQue[osq].push(neighbor);
				}
			}
		}
		delete neighbors;
	}
	return NoPath;
}