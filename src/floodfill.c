/*
 * floodfill.c
 *
 *  Created on: 1 mar 2016
 *      Author: zajec
 */

#include "floodfill.h"

const char NORTH=1;
const char EAST=2;
const char SOUTH=4;
const char WEST=8;
const char STOP=0;
const int WYMX=16;
const int WYMY=16;

void floodfill_init(floodfill* FF, int sX, int sY, int eX, int eY) {
	FF->N=WYMX;
	FF->M=WYMY;
	FF->StartX=sX;
	FF->StartY=sY;
	FF->EndX=eX;
	FF->EndY=eY;
	int i,j;
	int tmpmax=255;
	for (i=0;i<FF->N;i++) {
		for (j=0;j<FF->M;j++) {
			FF->Value[i][j]=tmpmax;
			FF->Wall[i][j]=0;
			FF->Action[i][j]='N';
		}
		FF->Visited[i]=0;
	}
	FF->Value[eX][eY]=0;
	for (i=0;i<FF->N;i++) {
		FF->Wall[i][0]|=SOUTH;
		FF->Wall[i][FF->M-1]|=NORTH;
	}
	for (i=0;i<FF->M;i++) {
		FF->Wall[0][i]|=WEST;
		FF->Wall[FF->N-1][i]|=EAST;
	}
}

void addWall(floodfill* FF, int pX, int pY, char wall) {
	FF->Wall[pX][pY]|=wall;
	if ((pX > 0) && (FF->Wall[pX][pY] & WEST)) {
		FF->Wall[pX-1][pY]|=EAST;
	}
	if ((pX < FF->N-1) && (FF->Wall[pX][pY] & EAST)) {
		FF->Wall[pX+1][pY]|=WEST;
	}
	if ((pY > 0) && (FF->Wall[pX][pY] & SOUTH)) {
		FF->Wall[pX][pY-1]|=NORTH;
	}
	if ((pY < FF->M-1) && (FF->Wall[pX][pY] & NORTH)) {
		FF->Wall[pX][pY+1]|=SOUTH;
	}
}

void actValue(floodfill* FF) {
	int i,j;
	char flag;
	int tmpmax=255;
	int tmp;
	for (i=0;i<FF->N;i++) {
		for (j=0;j<FF->M;j++) {
			FF->Value[i][j]=tmpmax;
		}
	}
	FF->Value[FF->EndX][FF->EndY]=0;
	do {
		flag=0;
		for (i=1;i<FF->N-1;i++) {
			for (j=1;j<FF->M-1;j++) {
				tmp=FF->Value[i][j];
				if (!(FF->Wall[i][j] & NORTH)) {
					if (tmp > FF->Value[i][j+1]+1) {
						tmp=FF->Value[i][j+1]+1;
					}
				}
				if (!(FF->Wall[i][j] & SOUTH)) {
					if (tmp > FF->Value[i][j-1]+1) {
						tmp=FF->Value[i][j-1]+1;
					}
				}
				if (!(FF->Wall[i][j] & EAST)) {
					if (tmp > FF->Value[i+1][j]+1) {
						tmp=FF->Value[i+1][j]+1;
					}
				}
				if (!(FF->Wall[i][j] & WEST)) {
					if (tmp > FF->Value[i-1][j]+1) {
						tmp=FF->Value[i-1][j]+1;
					}
				}
				if (tmp!=FF->Value[i][j]) {
					flag=1;
					FF->Value[i][j]=tmp;
				}
			}
		}
		for (i=1;i<FF->N-1;i++) {
			tmp=FF->Value[i][0];
			if (!(FF->Wall[i][0] & NORTH)) {
				if (tmp > FF->Value[i][1]+1) {
					tmp=FF->Value[i][1]+1;
				}
			}
			if (!(FF->Wall[i][0] & EAST)) {
				if (tmp > FF->Value[i+1][0]+1) {
					tmp=FF->Value[i+1][0]+1;
				}
			}
			if (!(FF->Wall[i][0] & WEST)) {
				if (tmp > FF->Value[i-1][0]+1) {
					tmp=FF->Value[i-1][0]+1;
				}
			}
			if (tmp!=FF->Value[i][0]) {
				flag=1;
				FF->Value[i][0]=tmp;
			}
		}
		for (i=1;i<FF->N-1;i++) {
			tmp=FF->Value[i][FF->M-1];
			if (!(FF->Wall[i][FF->M-1] & SOUTH)) {
				if (tmp > FF->Value[i][FF->M-2]+1) {
					tmp=FF->Value[i][FF->M-2]+1;
				}
			}
			if (!(FF->Wall[i][FF->M-1] & EAST)) {
				if (tmp > FF->Value[i+1][FF->M-1]+1) {
					tmp=FF->Value[i+1][FF->M-1]+1;
				}
			}
			if (!(FF->Wall[i][FF->M-1] & WEST)) {
				if (tmp > FF->Value[i-1][FF->M-1]+1) {
					tmp=FF->Value[i-1][FF->M-1]+1;
				}
			}
			if (tmp!=FF->Value[i][FF->M-1]) {
				flag=1;
				FF->Value[i][FF->M-1]=tmp;
			}
		}
		for (j=1;j<FF->M-1;j++) {
			tmp=FF->Value[0][j];
			if (!(FF->Wall[0][j] & EAST)) {
				if (tmp > FF->Value[1][j]+1) {
					tmp=FF->Value[1][j]+1;
				}
			}
			if (!(FF->Wall[0][j] & NORTH)) {
				if (tmp > FF->Value[0][j+1]+1) {
					tmp=FF->Value[0][j+1]+1;
				}
			}
			if (!(FF->Wall[0][j] & SOUTH)) {
				if (tmp > FF->Value[0][j-1]+1) {
					tmp=FF->Value[0][j-1]+1;
				}
			}
			if (tmp!=FF->Value[0][j]) {
				flag=1;
				FF->Value[0][j]=tmp;
			}
		}
		for (j=1;j<FF->M-1;j++) {
			tmp=FF->Value[FF->N-1][j];
			if (!(FF->Wall[FF->N-1][j] & WEST)) {
				if (tmp > FF->Value[FF->N-2][j]+1) {
					tmp=FF->Value[FF->N-2][j]+1;
				}
			}
			if (!(FF->Wall[FF->N-1][j] & NORTH)) {
				if (tmp > FF->Value[FF->N-1][j+1]+1) {
					tmp=FF->Value[FF->N-1][j+1]+1;
				}
			}
			if (!(FF->Wall[FF->N-1][j] & SOUTH)) {
				if (tmp > FF->Value[FF->N-1][j-1]+1) {
					tmp=FF->Value[FF->N-1][j-1]+1;
				}
			}
			if (tmp!=FF->Value[FF->N-1][j]) {
				flag=1;
				FF->Value[FF->N-1][j]=tmp;
			}
		}
		tmp=FF->Value[0][0];
		if (!(FF->Wall[0][0] & EAST)) {
			if (tmp > FF->Value[1][0]+1) {
				tmp=FF->Value[1][0]+1;
			}
		}
		if (!(FF->Wall[0][0] & NORTH)) {
			if (tmp > FF->Value[0][1]+1) {
				tmp=FF->Value[0][1]+1;
			}
		}
		if (tmp!=FF->Value[0][0]) {
			flag=1;
			FF->Value[0][0]=tmp;
		}
		tmp=FF->Value[0][FF->M-1];
		if (!(FF->Wall[0][FF->M-1] & EAST)) {
			if (tmp > FF->Value[1][FF->M-1]+1) {
				tmp=FF->Value[1][FF->M-1]+1;
			}
		}
		if (!(FF->Wall[0][FF->M-1] & SOUTH)) {
			if (tmp > FF->Value[0][FF->M-2]+1) {
				tmp=FF->Value[0][FF->M-2]+1;
			}
		}
		if (tmp!=FF->Value[0][FF->M-1]) {
			flag=1;
			FF->Value[0][FF->M-1]=tmp;
		}
		tmp=FF->Value[FF->N-1][0];
		if (!(FF->Wall[FF->N-1][0] & WEST)) {
			if (tmp > FF->Value[FF->N-2][0]+1) {
				tmp=FF->Value[FF->N-2][0]+1;
			}
		}
		if (!(FF->Wall[FF->N-1][0] & NORTH)) {
			if (tmp > FF->Value[FF->N-1][1]+1) {
				tmp=FF->Value[FF->N-1][1]+1;
			}
		}
		if (tmp!=FF->Value[FF->N-1][0]) {
			flag=1;
			FF->Value[FF->N-1][0]=tmp;
		}
		tmp=FF->Value[FF->N-1][FF->M-1];
		if (!(FF->Wall[FF->N-1][FF->M-1] & WEST)) {
			if (tmp > FF->Value[FF->N-2][FF->M-1]+1) {
				tmp=FF->Value[FF->N-2][FF->M-1]+1;
			}
		}
		if (!(FF->Wall[FF->N-1][FF->M-1] & SOUTH)) {
			if (tmp > FF->Value[FF->N-1][FF->M-2]+1) {
				tmp=FF->Value[FF->N-1][FF->M-2]+1;
			}
		}
		if (tmp!=FF->Value[FF->N-1][FF->M-1]) {
			flag=1;
			FF->Value[FF->N-1][FF->M-1]=tmp;
		}
	}while(flag);
	//}
}

void actValue2(floodfill* FF) {
	int i,j;
	for (i=0;i<FF->N;i++) {
		for (j=0;j<FF->M;j++) {
			if (!(FF->Visited[i] & 1<<j)) {
				addWall(FF,i,j,NORTH | SOUTH | WEST | EAST);
				FF->Value[i][j]=255;
			}
		}
	}
}

void setVisited(floodfill* FF, int pX, int pY) {
	FF->Visited[pX]|=1<<pY;
}

void changeStartEnd(floodfill* FF, int sX, int sY, int eX, int eY) {
	FF->StartX=sX;
	FF->StartY=sY;
	FF->EndX=eX;
	FF->EndY=eY;
}

char bestDir(floodfill* FF, int pX, int pY, char oldDir) {
	if (oldDir == NORTH) {
		if (pY < FF->M-1) {
			if ((FF->Value[pX][pY]-1 == FF->Value[pX][pY+1]) && (!(FF->Wall[pX][pY] & NORTH))) {
				return NORTH;
			}
		}
		if (pX < FF->N-1) {
			if ((FF->Value[pX][pY]-1 == FF->Value[pX+1][pY]) && (!(FF->Wall[pX][pY] & EAST))) {
				return EAST;
			}
		}
		if (pX > 0) {
			if ((FF->Value[pX][pY]-1 == FF->Value[pX-1][pY]) && (!(FF->Wall[pX][pY] & WEST))) {
				return WEST;
			}
		}
		if (pY > 0) {
			if ((FF->Value[pX][pY]-1 == FF->Value[pX][pY-1]) && (!(FF->Wall[pX][pY] & SOUTH))) {
				return SOUTH;
			}
		}
	}
	if (oldDir == EAST) {
		if (pX < FF->N-1) {
			if ((FF->Value[pX][pY]-1 == FF->Value[pX+1][pY]) && (!(FF->Wall[pX][pY] & EAST))) {
				return EAST;
			}
		}
		if (pY > 0) {
			if ((FF->Value[pX][pY]-1 == FF->Value[pX][pY-1]) && (!(FF->Wall[pX][pY] & SOUTH))) {
				return SOUTH;
			}
		}
		if (pY < FF->M-1) {
			if ((FF->Value[pX][pY]-1 == FF->Value[pX][pY+1]) && (!(FF->Wall[pX][pY] & NORTH))) {
				return NORTH;
			}
		}
		if (pX > 0) {
			if ((FF->Value[pX][pY]-1 == FF->Value[pX-1][pY]) && (!(FF->Wall[pX][pY] & WEST))) {
				return WEST;
			}
		}
	}
	if (oldDir == SOUTH) {
		if (pY > 0) {
			if ((FF->Value[pX][pY]-1 == FF->Value[pX][pY-1]) && (!(FF->Wall[pX][pY] & SOUTH))) {
				return SOUTH;
			}
		}
		if (pX > 0) {
			if ((FF->Value[pX][pY]-1 == FF->Value[pX-1][pY]) && (!(FF->Wall[pX][pY] & WEST))) {
				return WEST;
			}
		}
		if (pX < FF->N-1) {
			if ((FF->Value[pX][pY]-1 == FF->Value[pX+1][pY]) && (!(FF->Wall[pX][pY] & EAST))) {
				return EAST;
			}
		}
		if (pY < FF->M-1) {
			if ((FF->Value[pX][pY]-1 == FF->Value[pX][pY+1]) && (!(FF->Wall[pX][pY] & NORTH))) {
				return NORTH;
			}
		}
	}
	if (oldDir == WEST) {
		if (pX > 0) {
			if ((FF->Value[pX][pY]-1 == FF->Value[pX-1][pY]) && (!(FF->Wall[pX][pY] & WEST))) {
				return WEST;
			}
		}
		if (pY < FF->M-1) {
			if ((FF->Value[pX][pY]-1 == FF->Value[pX][pY+1]) && (!(FF->Wall[pX][pY] & NORTH))) {
				return NORTH;
			}
		}
		if (pY > 0) {
			if ((FF->Value[pX][pY]-1 == FF->Value[pX][pY-1]) && (!(FF->Wall[pX][pY] & SOUTH))) {
				return SOUTH;
			}
		}
		if (pX < FF->N-1) {
			if ((FF->Value[pX][pY]-1 == FF->Value[pX+1][pY]) && (!(FF->Wall[pX][pY] & EAST))) {
				return EAST;
			}
		}
	}
	return STOP;
}

char bestAction(floodfill* FF, int pX, int pY, char oldDir) {
	if (pX == FF->EndX && pY == FF->EndY) {
		return 'E';
	}
	/*if (FF->Value[pX][pY] == 255) {
		return 'S';
	}*/
	char dir = bestDir(FF,pX,pY,oldDir);
	if (dir == oldDir) {
		return 'F';
	}
	if (oldDir == NORTH) {
		if (dir == SOUTH) {
			return 'I';
		}
		if (dir == EAST) {
			return 'R';
		}
		if (dir == WEST) {
			return 'L';
		}
	}
	if (oldDir == SOUTH) {
		if (dir == NORTH) {
			return 'I';
		}
		if (dir == EAST) {
			return 'L';
		}
		if (dir == WEST) {
			return 'R';
		}
	}
	if (oldDir == EAST) {
		if (dir == WEST) {
			return 'I';
		}
		if (dir == SOUTH) {
			return 'R';
		}
		if (dir == NORTH) {
			return 'L';
		}
	}
	if (oldDir == WEST) {
		if (dir == EAST) {
			return 'I';
		}
		if (dir == NORTH) {
			return 'R';
		}
		if (dir == SOUTH) {
			return 'L';
		}
	}
}

void bestActionforAll(floodfill* FF, int pX, int pY, char sOrient) {
	char act = 'N';
	do {
		if (sOrient == NORTH) {
			act = bestAction(FF,pX,pY+1,sOrient);
			FF->Action[pX][pY+1] = act;
			pY=pY+1;
		}
		if (sOrient == SOUTH) {
			act = bestAction(FF,pX,pY-1,sOrient);
			FF->Action[pX][pY-1] = act;
			pY=pY-1;
		}
		if (sOrient == EAST) {
			act = bestAction(FF,pX+1,pY,sOrient);
			FF->Action[pX+1][pY] = act;
			pX=pX+1;
		}
		if (sOrient == WEST) {
			act = bestAction(FF,pX-1,pY,sOrient);
			FF->Action[pX-1][pY] = act;
			pX=pX-1;
		}
		if (act == 'R') {
			sOrient = sOrient << 1;
		}
		if (act == 'L') {
			sOrient = sOrient << 3;
		}
		if (act == 'I' || act == 'E') {
			sOrient = sOrient << 2;
		}
		if (sOrient > 8) {
			sOrient = sOrient >> 4;
		}
	}
	while(act != 'E');
	return;
}
