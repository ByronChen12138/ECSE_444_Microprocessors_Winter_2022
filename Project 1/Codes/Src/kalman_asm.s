/*
* kalman_asm.s
*/

//.section .data
.syntax unified
.align 16
.section .text.rodata
.global kalman_asm

/*
* This function update the kalman filter given the measurement
* err equals to -1 if not available; otherwise, return 0
*
* R0: the address to the quintuple(q, r, x, p, k, err) that are six single-precision fp numbers
* 		err stores if the process calculated properly or not
* 		err equals to -1 if not available; otherwise, return 0
* S0: measurement
*/

kalman_asm: // Main kalman label
	// Load the FP numbers from the given address in R0 into seperate fp registers
	VPUSH {S1-S12}
	PUSH {R1-R12, LR}

	// Reset the overflow bit in FPSCR
	VMRS R1, FPSCR
	AND R1, R1, 0xFFFFFFFB
	VMSR FPSCR, R1

							// S0 := measurement
	VLDR S1, [R0]			// S1 := q, process noise covariance
	VLDR S2, [R0, #4]		// S2 := r, measurement noise covariance
	VLDR S3, [R0, #8]		// S3 := x, estimated value
	VLDR S4, [R0, #12]		// S4 := p, estimation error covariance
	VLDR S5, [R0, #16]		// S5 := k, adaptive Kalman filter gain

	// p = p + q
	VADD.F32 S4, S4, S1 	// S4 := p = p + q
	BL is_overflow

	// k = p / (p + r)
	VADD.F32 S5, S4, S2		// S5 := k = p + r
	BL is_overflow

	VCMP.F32 S5, #0.0		// Compare the S5 with zero
	VMRS APSR_nzcv, FPSCR
	BEQ exception			// If (p + r) = 0, branch to label exception

	VDIV.F32 S5, S4, S5		// S5 := k = p / k
	BL is_overflow

	// x = x + k * (measurement - x)
	// Since q is not needed anymore, S1 is used
	VSUB.F32 S1, S0, S3		// S1 := measurement - x
	BL is_overflow

	VMUL.F32 S1, S1, S5		// S1 := k * (S1)
	BL is_overflow

	VADD.F32 S3, S3, S1		// S3 := x = x + S1
	BL is_overflow

	// p = (1 - k) * p
	VMOV.F32 S1, #1			// S1 := 1
	VSUB.F32 S1, S1, S5		// S1 := S1 - k
	VMUL.F32 S4, S1, S4		// S4 := p = S1 * p
	BL is_overflow

	// Update
	VSTR S3, [R0, #8]		// S3 := x, estimated value
	VSTR S4, [R0, #12]		// S4 := p, estimation error covariance
	VSTR S5, [R0, #16]		// S5 := k, adaptive Kalman filter gain

	// Return
	VMOV.F32 S0, #1.0
	POP {R1-R12, LR}
	VPOP {S1-S12}
	BX LR

is_overflow: 				// Check if it is overflow
	VMRS R1, FPSCR			// Store the FPSCR in R1
	AND R1, R1, #4			// Test if it is overflow
	CMP R1, #4				// If overflow, it will equal 4; otherwise, 0
	BEQ exception			// If overflow, branch to label overflow
	BX LR

exception: 					// Exception label
	// Return -1 n the sixth position if anything is overflow or divided by zero
	VMOV.F32 S0, #-1
	VSTR S0, [R0, #20]
	POP {R1-R12, LR}
	VPOP {S1-S12}
	BX LR

