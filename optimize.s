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

optimize:
@ at the beginning, R0 = x as Integer, R1 = a, R2 = b, R3 = addr of cnt (counter)
@ Init
	PUSH {R4-R6}	@ back up other registers R4 ~ R7
	MOV R4, #100	@ R4 = Scale Factor
	MOV R5, #5		@ R5 = 1 / Lambda
	MUL R0, R4		@ R0 *= R4, which is x *= 100
	MUL R2, R4		@ R2 *= R4, which is b *= 100

@ now, R0 = 100 * x as Integer, R1 = a, R2 = b * 100, R3 = addr of cnt (counter)
	LDR R4, [R3]	@ load cnt's value into R4
loop:
	ADD R4, #1		@ ++R4 for every loop
	MUL R6, R0, R1	@ R6 = R0 * R1, which is R6 = a * x
	ADD R6, R6		@ R6 += R6, which is R6 = 2 * a * x
	ADD R6, R2		@ R6 += R2, which is R6 = d(f) = 2 * a * x + 100 * b
	SDIV R6, R5		@ R6 /= R5, which is d(f) / lambda
	CMP R6, #0		@ Compare R6 with zero
	BEQ DONE		@ if this branch is equal, go to DONE
	SUB R0, R6		@ else R0 -= R6, which is R0 = x` =  x - d(f) / lambda
b loop				@ while(1), next loop

DONE:
	STR R4, [R3]	@ store R4 back to R3's address, which is *cnt = R4
	POP {R4-R6}		@ restore registers R4 ~ R7
	BX LR			@ return R0, AKA x
