#include <stdio.h>
#include <stdlib.h>



float** createMat(int m, int n);

void destroyMat(float** arr);

float** matFromString(char str_data[],int rows,int cols);
float** matFromArr(int data[],int rows,int cols);
//adds m2 from m1. assumed both are of size rowsxcols
float** addMat(float** m1,float** m2,int rows,int cols);
//subtracts m2 from m1. assumed both are of size rowsxcols
float** subtractMat(float** m1,float** m2,int rows,int cols);
//multiplies two matrixes.assumes m1 is size lxm, m2 is size mxn
//resulting array is size lxn
float** multMat(float** m1,float** m2,int l,int m,int n);
//multiplies matrix by -1.
void negMat(float** m1,int rows,int cols);
void printMat(float** m,int rows,int cols);