/*	$OpenBSD: rf_revent.h,v 1.2 1999/02/16 00:03:25 niklas Exp $	*/
/*	$NetBSD: rf_revent.h,v 1.3 1999/02/05 00:06:17 oster Exp $	*/

/*
 * Copyright (c) 1995 Carnegie-Mellon University.
 * All rights reserved.
 *
 * Author:
 *
 * Permission to use, copy, modify and distribute this software and
 * its documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 *
 * CARNEGIE MELLON ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS"
 * CONDITION.  CARNEGIE MELLON DISCLAIMS ANY LIABILITY OF ANY KIND
 * FOR ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 *
 * Carnegie Mellon requests users of this software to return to
 *
 *  Software Distribution Coordinator  or  Software.Distribution@CS.CMU.EDU
 *  School of Computer Science
 *  Carnegie Mellon University
 *  Pittsburgh PA 15213-3890
 *
 * any improvements or extensions that they make and grant Carnegie the
 * rights to redistribute these changes.
 */

/*********************************************************************
 *
 * rf_revent.h -- Header file for reconstruction event handling code.
 *
 *********************************************************************/

#ifndef	_RF__RF_REVENT_H_
#define	_RF__RF_REVENT_H_

#include "rf_types.h"

int  rf_ConfigureReconEvent(RF_ShutdownList_t **);
RF_ReconEvent_t *rf_GetNextReconEvent(RF_RaidReconDesc_t *,
	RF_RowCol_t, void (*) (void *), void *);
void rf_CauseReconEvent(RF_Raid_t *,
	RF_RowCol_t, RF_RowCol_t, void *, RF_Revent_t);
void rf_FreeReconEventDesc(RF_ReconEvent_t *);

#endif	/* !_RF__RF_REVENT_H_ */
