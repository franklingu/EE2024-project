 	.syntax unified
 	.cpu cortex-m3
 	.thumb
 	.align 2
 	.global	optimize
 	.thumb_func
@  EE2024 Assignment 1 with Bonus
@  Lab Group B1
@  XIE KAI (A0102016)
@  GU JUNCHAO (A0105750)

@ This function takes in x, a, b, and address of counter, then give optimization answer:
@ x is a guess value where we start to calculate the optimization answer, and it's kept updating
@ during the whole process. When the function returns x, it will hold the position where there is
@ an optimization answer.
@ a and b is coefficients for quadratic equation (ax^2 + bx + c).
@ address of counter allows us to modify its value.
optimize:
@ R0 = Factor * x as Integer, AKA scaling the input x
@ R1 = a,
@ R2 = b * Factor, we also scale the answer (normally answer = -b / (2 * a) for quadratic equation)
@ R3 = addr of cnt, we access counter's address to modify its value
@ Initialize, eg push registers, LDR etc
	PUSH {R4-R6}    @ back up other registers R4 ~ R7
	LDR R4, [R3]    @ load cnt's value into R4
	MOV R5, #5      @ R5 = 1 / Lambda
loop:
@ Increment counter by One
	ADD R4, #1      @ ++R4 for every loop
@ Calculating d(f) = 2 * a * x + 100 * b
	MUL R6, R0, R1  @ R6 = R0 * R1, which is R6 = a * x
	ADD R6, R6      @ R6 += R6, which is R6 = 2 * a * x
	ADD R6, R2      @ R6 += R2, which is R6 = d(f) = 2 * a * x + 100 * b
	SDIV R6, R5     @ R6 /= R5, which is d(f) * lambda
@ Comparing d(f) with Zero, if so, go to DONE
	CMP R6, #0      @ Compare R6 with zero
	BEQ DONE        @ if this branch is equal, go to DONE
@ Calculating next x
	SUB R0, R6      @ else R0 -= R6, which is R0 = x` =  x - d(f) * lambda
b loop                  @ while(1), next loop

DONE:
@ Preparing to exit, eg STR, restore registers
	STR R4, [R3]    @ store R4 back to R3's address, which is *cnt = R4
	POP {R4-R6}     @ restore registers R4 ~ R7
	BX LR           @ return R0, AKA xsoli
