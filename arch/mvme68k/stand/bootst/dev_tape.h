/*	$OpenBSD: dev_tape.h,v 1.2 2001/07/04 08:06:54 niklas Exp $	*/


int	tape_open(struct open_file *, ...);
int	tape_close(struct open_file *);
int	tape_strategy(void *, int, daddr_t, size_t, void *, size_t *);
int	tape_ioctl();

