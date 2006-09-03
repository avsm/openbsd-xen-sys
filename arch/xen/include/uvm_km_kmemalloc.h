/* $NetBSD: xbd.c,v 1.24 2005/10/18 00:14:43 yamt Exp $ */

/*
 *
 * Copyright (c) 2004 Christian Limpach.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Christian Limpach.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/*
 * XXX move uvm_km_kmemalloc1() to src/sys/uvm/uvm_km.c and replace
 * uvm_km_kmemalloc() with
 *  #define uvm_km_kmemalloc(map, obj, size, flags)	\
 *	uvm_km_kmemalloc1(map, obj, 0, UVM_UNKNOWN_OFFSET, flags)
 *
 * uvm_km_kmemalloc1() is needed to meet the alignment requirements
 *   for the ring buffer. It only differs in the two align and prefer
 *   arguments which are just passed through to uvm_map().
 */

vaddr_t
uvm_km_kmemalloc1(struct vm_map *map, struct uvm_object *obj,
		vsize_t size, vsize_t align, voff_t prefer, int flags);
