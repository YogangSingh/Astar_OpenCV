#include "Pathfinding.h"
#include "highgui.hpp"
#include <iostream>

#define MUL 20

using namespace std;
using namespace cv;

Pathfinding pathfinding;
Point2i turtPos;
Mat mapimg, turtimg;

void DisplayMap() {

	Mat tmp;
	resize(mapimg, tmp, mapimg.size() * MUL, 0, 0, 0);
	turtimg.copyTo(tmp.rowRange(turtPos.y * MUL, (turtPos.y + 1) * MUL).colRange(turtPos.x * MUL, (turtPos.x + 1) * MUL));
	imshow("Pathfinding turtle", tmp);
}

void MouseDest(int event, int x, int y, int flags, void* userdata) {

	if (event == EVENT_LBUTTONUP) {
		pathfinding.SetDestination(x / MUL, y / MUL);
		cout << "Destination: x = " << x/MUL << " y = " << y/MUL << endl << "Press any key to continue..." << endl;
	}
}

int main() {

	//open and load map and turtle image
	mapimg = imread("kornelmap.tif");
	turtimg = imread("Kornel_sajat_teknose.png");
	resize(turtimg, turtimg, Size(MUL, MUL));
	//turtle initial position
	turtPos = Point2i(2, 1);
	pathfinding.SetMap(mapimg);
	pathfinding.SetWallWeight(0.f, 0);
	pathfinding.SetDiagonal(false);
	DisplayMap();

	while (true) {
		cout << "Click on destination!" << endl;
		pathfinding.SetStart(turtPos.x, turtPos.y);
		setMouseCallback("Pathfinding turtle", MouseDest);
		int key = waitKey();
		if (key == 27)
			break;
		Pathfinding::ErrorCodes result = pathfinding.CalculatePath();
		if (result == Pathfinding::NoError) {

			string path = pathfinding.GetDirections();
			for (int i = 0; i < path.length(); i++) {
				char c = path[i];
				turtPos.x += Pathfinding::nonDiagonalDirX[c - '0'];
				turtPos.y += Pathfinding::nonDiagonalDirY[c - '0'];
				DisplayMap();
				int key = waitKey(50);
				if (key == 27)
					break;
			}
		}
		else {

			cout << "Error! Unsuccessful pathfinding!" << endl;
		}
	}
}