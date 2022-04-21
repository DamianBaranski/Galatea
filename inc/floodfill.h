/*
 * floodfill.h
 *
 *  Created on: 29 lut 2016
 *      Author: zajec
 */

#ifndef APPLICATION_USER_FLOODFILL_H_
#define APPLICATION_USER_FLOODFILL_H_

extern const char NORTH;
extern const char EAST;
extern const char SOUTH;
extern const char WEST;
extern const char STOP;
extern const int WYMX;
extern const int WYMY;

typedef struct {
	int N;
	int M;
	unsigned char Value[16][16];
	unsigned char Wall[16][16];
	unsigned char Action[16][16];
	unsigned short int Visited[16];
	int StartX;
	int StartY;
	int EndX;
	int EndY;
} floodfill;

void floodfill_init(floodfill* FF, int sX, int sY, int eX, int eY);
void addWall(floodfill* FF, int pX, int pY, char wall);
void actValue(floodfill* FF);
void actValue2(floodfill* FF);
void setVisited(floodfill* FF, int pX, int pY);
void changeStartEnd(floodfill* FF, int sX, int sY, int eX, int eY);
char bestDir(floodfill* FF, int pX, int pY, char oldDir);
char bestAction(floodfill* FF, int pX, int pY, char oldDir);
void bestActionforAll(floodfill* FF, int pX, int pY, char sOrient);

#endif /* APPLICATION_USER_FLOODFILL_H_ */
