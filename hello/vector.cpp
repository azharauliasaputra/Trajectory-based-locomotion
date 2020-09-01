//
//  vector.cpp
//  hello
//
//  Created by Azhar Aulia Saputra on 5/26/16.
//  Copyright © 2016 Azhar Aulia Saputra. All rights reserved.
//

#include <fcntl.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "vector.h"

double dotProduct(double v1[], double v2[], int j){
    double v = 0;
    for(int i = 0; i<j; i++){
        v += v1[i]*v2[i];
    }
    return v;
}
double vectorValue(double v1[], int j){
    double v=0,x;
    for(int i = 0; i<j; i++){
        v += v1[i]*v1[i];
    }
    x = sqrt(v);
    return x;
}
double *vectorSubtraction(double v1[], double v2[], int j){
    double *v = (double *) malloc(sizeof (double) * j);
    
    for(int i = 0; i<j; i++)
        v[i] = v1[i] - v2[i];
    return v;
}double *vectorProduct(double v1[], double v2[], int j){
    double *v = (double *) malloc(sizeof (double) * j);
    
    for(int i = 0; i<j; i++)
        v[i] = v1[i] * v2[i];
    return v;
}double *vectorScale(double a, double v2[], int j){
    double *v = (double *) malloc(sizeof (double) * j);
    
    for(int i = 0; i<j; i++){
        v[i] = a * v2[i];
    }
    return v;
}