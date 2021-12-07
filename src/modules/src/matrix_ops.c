#include <matrix_ops.h>

float** createMat(int m, int n)
{
    float* values = calloc(m*n, sizeof(float));
    float** rows = malloc(m*sizeof(float*));
    for (int i=0; i<m; ++i)
    {
        rows[i] = values + i*n;
    }
    return rows;
}

void destroyMat(float** arr)
{
    free(*arr);
    free(arr);
}
float** matFromString(char str_data[],int rows,int cols){
    float** result = createMat(rows,cols);
    char* p=strtok(str_data," \n\t");
    int ri=0;
    int ci=0;

    while(p!=NULL)
    {
        result[ri][ci]=atof(p);
        p=strtok(NULL, " \n\t");
        ci=ci+1;
        if(ci>=cols){
            ri=ri+1;
            ci=0;
        }
        if(ri==rows){
            return result;
        }
    }
}
float** matFromArr(float data[],int rows,int cols){
    float** result = createMat(rows,cols);
    int ri=0;
    int ci=0;
    int max=rows*cols;
    for(int i=0;i<max;i++)
    {
        result[ri][ci]=data[i];
        ci=ci+1;
        if(ci>=cols){
            ri=ri+1;
            ci=0;
        }
        if(ri==rows){
            return result;
        }
    }
}


//adds m2 from m1. assumed both are of size rowsxcols
float** addMat(float** m1,float** m2,int rows,int cols)
{
    float** result = createMat(rows,cols);
    for(int r=0;r<rows;r++){
        for(int c=0;c<cols;c++){
            result[r][c]=m1[r][c]+m2[r][c];
        }
    }
    return result;
}
//subtracts m2 from m1. assumed both are of size rowsxcols
float** subtractMat(float** m1,float** m2,int rows,int cols)
{
    float** result = createMat(rows,cols);
    for(int r=0;r<rows;r++){
        for(int c=0;c<cols;c++){
            result[r][c]=m1[r][c]-m2[r][c];
        }
    }
    return result;
}
//multiplies two matrixes.assumes m1 is size lxm, m2 is size mxn
//resulting array is size lxn
float** multMat(float** m1,float** m2,int l,int m,int n)
{
    float** result = createMat(l,n);

    for(int li=0;li<l;li++){
        for(int ni=0;ni<n;ni++){
            float sum_i=0.0f;
            for(int mi=0;mi<m;mi++){
                sum_i=sum_i+m1[li][mi]*m2[mi][ni];
            }
            result[li][ni]=sum_i;
        }
    }
    return result;
}
//multiplies matrix by -1.
void negMat(float** m1,int rows,int cols){
    for(int r=0;r<rows;r++){
        for(int c=0;c<cols;c++){
            m1[r][c]=-m1[r][c];
        }
    }
}
void printMat(float** m,int rows,int cols)
{
	int size=rows*cols*10+3*(rows+1)+2;
	char mat_str[size];
	snprintf(mat_str,2,"%s","[");
	for(int r=0;r<rows;r++){
		strcat(mat_str,"[");
		for(int c=0;c<cols;c++){
			char ele[10];
				
			snprintf(ele,8,"%3.3f, ",m[r][c]);
			strcat(mat_str,ele);
		}
		strcat(mat_str,"]\n");
	}
	strcat(mat_str,"]\n");
	//DEBUG_PRINT(mat_str);
}