/*	$OpenBSD$	*/

/*
 * Copyright (c) 2004 Theo de Raadt <deraadt@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#include <sys/types.h>
#include <fcntl.h>
#include <dev/pci/neoreg.h>

#include "neo-coeff.h"

#define FILENAME "neo-coefficients"

int
main(int argc, char *argv[])
{
	struct neo_firmware nf;
	int fd, i;

	fd = open(FILENAME, O_WRONLY|O_CREAT|O_TRUNC, 0644);
	if (fd == -1)
		err(1, FILENAME);

	bcopy(coefficientSizes, &nf.coefficientSizes,
	    sizeof nf.coefficientSizes);
	bcopy(coefficients, &nf.coefficients,
	    sizeof nf.coefficients);

	write(fd, &nf, sizeof nf);
	printf("created %s length %d\n", FILENAME, sizeof nf);
	close(fd);
	return (0);
}
