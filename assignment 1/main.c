#include "stdio.h"

// EE2024 Assignment 1
// (C) CK Tham, ECE, NUS, 2014

// Optimization routine written in assembly language
// Input parameters (signed integers):
//     xi : integer version of x
//     a, b, c : integer coefficients of f(x)
// Output : returns solution x (signed integer)
extern int optimize(int xi, int a, int b, int* cnt);

int main(void)
{
    int a=2, b=-3, c=3, xsoli, cnt = 0;
    float fp, x, xprev, xsol, change, lambda=0.2;

    printf("Initial value of x (press [Enter] after keying in): "); // try x0 = -12.35
    scanf("%f", &x);

    //  ARM ASM & Integer version
    printf("ARM ASM & Integer version:\n");

    const int FACTOR = 100;
    xsoli = optimize((int)(x * FACTOR),a,(b * FACTOR),&cnt);
    xsol = xsoli;
    printf("xsol : %f \n",(float)xsol/100);
    printf("number of iterations: %d\n", cnt);

    //  C & Floating Point version
    printf("C & Floating point version:\n",x);
    while (1) {
    	fp = 2*a*x + b;

    	xprev = x;
    	change = -lambda*fp;
    	x += change;

    	printf("x: %f, fp: %f, change: %f\n", x, fp, change);
    	if (x==xprev) break;
    }

    // Enter an infinite loop, just incrementing a counter
    // This is for convenience to allow registers, variables and memory locations to be inspected at the end
    volatile static int loop = 0;
    while (1) {
        loop++;
    }

    return 0;
}

