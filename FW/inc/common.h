/*
 * File:   common.h
 * Author: Kreyl
 *
 * Created on June 9, 2011, 11:11 PM
 */

#ifndef COMMON_H
#define	COMMON_H

typedef struct {
    uint8_t *Buf;
    uint8_t Length;
    uint8_t CurrentItem;
} Buf8_t;
typedef Buf8_t* Buf8_p;

// Functional types
typedef void (*Voidvoid_f)(void);

#endif	/* COMMON_H */

