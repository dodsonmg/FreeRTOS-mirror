/*
 * This code must remain standalone and position independent, since it will
 * be copied to a scratch area and run from there to avoid writing over
 * itself. Do not reference any globals here; any required values must be
 * available via the arguments provided (and located in memory that isn't about
 * to be clobbered).
 */
	.option norelax

	/*
	 * Keep start as aligned as the trap vector to ensure copying does not
	 * de-align it.
	 */
	.balign 4
	.global netboot_load_trampoline_start
	.type netboot_load_trampoline_start, @object
netboot_load_trampoline_start:
	/*
	 *  a0 - hartid from previous stage
	 *  a1 - dtb from previous stage
	 *  a2 - dynamic info
	 * ca3 - commands
	 *  a4 - entry point
	 *  a5 - halt before jump
	 * ca6 - almighty RWX capabiliity
	 */
netboot_load_trampoline:
	csetoffset ca6, ca6, zero

	/*
	 * We don't want to trap back to the FreeRTOS handler; the handler and/or
	 * any global state used is about to be clobbered. The reset state also has
	 * mtcc almighty, so we rederive to match. We also don't want to take any
	 * interrupts from now on, so we set mstatus to its reset state early.
	 */
	csrw mstatus, zero
	cllc ct0, netboot_load_trampoline_trap
	csetoffset ct0, ca6, t0
	cspecialw mtcc, ct0

	/*
	 * Iterate through the null-src-terminated list of load commands.
	  */
1:	clc ct0, 0*(__riscv_clen/8)(ca3) /* src */
	beqz t0, 8f
	clc ct1, 1*(__riscv_clen/8)(ca3) /* dst */
#if __riscv_xlen == 32
	clw t2, 2*(__riscv_clen/8)+0(ca3) /* copysz */
	clw t3, 2*(__riscv_clen/8)+4(ca3) /* zerosz */
#else
	cld t2, 2*(__riscv_clen/8)+0(ca3) /* copysz */
	cld t3, 2*(__riscv_clen/8)+8(ca3) /* zerosz */
#endif
	/* Increment early for next iteration */
	cincoffset ca3, ca3, 2*(__riscv_clen/8)+2*(__riscv_xlen/8)
	/* t4 is block mask */
	li t4, (__riscv_clen/8)-1

	/* Copy bytes until src and dst aligned or copysz == 0 */
2:	beqz t2, 2f
	or t5, t0, t1
	and t5, t5, t4
	beqz t5, 3f
	clb t5, 0(ct0)
	csb t5, 0(ct1)
	cincoffset ct0, ct0, 1
	cincoffset ct1, ct1, 1
	addi t2, t2, -1
	j 2b

	/* Copy blocks until copysz <= block mask */
3:	bleu t2, t4, 4f
	clc ct5, 0(ct0)
	csc ct5, 0(ct1)
	cincoffset ct0, ct0, (__riscv_clen/8)
	cincoffset ct1, ct1, (__riscv_clen/8)
	addi t2, t2, -(__riscv_clen/8)
	j 3b

	/* Copy bytes until copysz == 0 */
4:	beqz t2, 2f
	clb t5, 0(ct0)
	csb t5, 0(ct1)
	cincoffset ct0, ct0, 1
	cincoffset ct1, ct1, 1
	addi t2, t2, -1
	j 4b

	/* Zero bytes until dst aligned or zerosz == 0 */
2:	beqz t3, 2f
	and t5, t1, t4
	beqz t5, 3f
	csb zero, 0(ct1)
	cincoffset ct1, ct1, 1
	addi t3, t3, -1
	j 2b

	/* Zero blocks until zerosz <= block mask */
3:	bleu t3, t4, 4f
	csc cnull, 0(ct1)
	cincoffset ct1, ct1, (__riscv_clen/8)
	addi t3, t3, -(__riscv_clen/8)
	j 3b

	/* Zero bytes until zerosz == 0 */
4:	beqz t3, 2f
	csb zero, 0(ct1)
	cincoffset ct1, ct1, 1
	addi t3, t3, -1
	j 4b

	/* Next load command (already incremented after loading this one) */
2:	j 1b

8:	fence rw, rw
	fence.i
	/* Reset mcause */
	csrw mcause, zero
	/* Clear mtdc/mscratchc to match reset state */
	cmove ct0, cnull
	cspecialw mtdc, ct0
	cspecialw mscratchc, ct0
	/* Make ddc/mepcc almighty to match reset state */
	cspecialw ddc, ca6
	cspecialw mepcc, ca6
	/* Prepare jump target*/
	csetoffset ca4, ca6, a4
	csetflags ca4, ca4, zero
	/* Halt if requested */
	beqz a5, 9f
	ebreak
9:	/* Jump to next stage */
	cjr ca4
	.size netboot_load_trampoline, . - netboot_load_trampoline

	.balign 4
netboot_load_trampoline_trap:
1:	j 1b /* Spin rather than ebreak to avoid clobbering mcause/mepcc */

	.global netboot_load_trampoline_end
	.type netboot_load_trampoline_end, @object
netboot_load_trampoline_end:
	.size netboot_load_trampoline_end, 0
	.size netboot_load_trampoline_start, netboot_load_trampoline_end - netboot_load_trampoline_start
