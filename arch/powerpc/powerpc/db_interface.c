/*	$OpenBSD: db_interface.c,v 1.3 1999/07/05 20:23:08 rahnds Exp $	*/

#include <sys/param.h>
#include <sys/proc.h>

#include <machine/db_machdep.h>
void
Debugger()
{
	ddb_trap();
}

int
ddb_trap_glue(frame)
	struct trapframe *frame;
{
	if (!(frame->srr1 & PSL_PR)
	    && (frame->exc == EXC_TRC
		|| (frame->exc == EXC_PGM
		    && (frame->srr1 & 0x20000))
		|| frame->exc == EXC_BPT)) {

		bcopy(frame->fixreg, DDB_REGS->tf.fixreg,
			32 * sizeof(u_int32_t));
		DDB_REGS->tf.srr0 = frame->srr0;
		DDB_REGS->tf.srr1 = frame->srr1;

		db_trap(T_BREAKPOINT, 0);

		bcopy(DDB_REGS->tf.fixreg, frame->fixreg,
			32 * sizeof(u_int32_t));

		return 1;
	}
	return 0;
}
