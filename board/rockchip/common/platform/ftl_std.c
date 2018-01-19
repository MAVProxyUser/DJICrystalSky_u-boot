/*
 * (C) Copyright 2008-2015 Rockchip Electronics
 * Peter, Software Engineering, <superpeter.cai@gmail.com>.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include "../config.h"


/***************************************************************************
��������:
���ú���:
***************************************************************************/
void * ftl_memset(void *s, int c, unsigned int n)
{
	return ((void*)memset(s, c, n));
}

/***************************************************************************
��������:
���ú���:
***************************************************************************/
void* ftl_memcpy(void* pvTo, const void* pvForm, unsigned int size)
{
	if(((unsigned long)pvTo & 0x3)||((unsigned long)pvForm & 0x3)) {
		int i;
		char * pTo = (char *)pvTo;
		char * pForm = (char *)pvForm;

		for(i=0;i<size;i++) {
			*pTo++ = *pForm++;
		}

		return 0;
	}

	return ((void*)memcpy(pvTo, pvForm, size));
}

/***************************************************************************
��������:
���ú���:
***************************************************************************/
int ftl_memcmp(void *str1, void *str2, unsigned int count)
{
	return (memcmp(str1, str2, count));
}

