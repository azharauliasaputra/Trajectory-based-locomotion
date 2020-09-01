//
//  vector.hpp
//  hello
//
//  Created by Azhar Aulia Saputra on 5/26/16.
//  Copyright © 2016 Azhar Aulia Saputra. All rights reserved.
//

#ifndef vector_h
#define vector_h

#include <stdio.h>


extern float moveStick;
extern double xZMP, yZMP;
double dotProduct(double v1[], double v2[], int j);
double vectorValue(double v1[], int j);
double *vectorSubtraction(double v1[], double v2[], int j);
double *vectorProduct(double v1[], double v2[], int j);
double *vectorScale(double a, double v2[], int j);
#endif /* vector_hpp */
