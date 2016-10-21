#ifndef _VIK_CIRCULAR_BUFFER_H_
#define _VIK_CIRCULAR_BUFFER_H_

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <asm/types.h>

/*
 * Copyright (c) 2005 Casabyte, Inc.  All Rights Reserved
 */

/*
 * NOTES:
 *  maximum buffer capacity = (buffer size - 1) - to detect buffer full conditions
 *  so - buffer size should be wanted size + 1 .
 *
 *  there is no mutex to synchronize - buffer will work only in kernel mode
 */

typedef uint16_t BufferElement;

typedef struct  CircBuffer
{
    BufferElement*      m_Buffer;   // Buffer storage
    uint32_t            m_In;       // Index of added data
    uint32_t            m_Out;      // Index of extracted data
    uint32_t            m_Size;     // number of BufferElement in the buffer
                                    // power of 2
    spinlock_t          m_Lock;     //
}   CircBuffer, *CircBufferHndl;

static inline uint32_t cb_IsEmpty(CircBufferHndl p_BufHndl)
{
    return (p_BufHndl->m_In == p_BufHndl->m_Out);
}

static inline uint32_t cb_GetElementCount(CircBufferHndl p_BufHndl)
{
    return ((p_BufHndl->m_In  + p_BufHndl->m_Size) - p_BufHndl->m_Out) & (p_BufHndl->m_Size - 1);
}
//
//  reset buffer
//
static inline void cb_ClearBuffer(CircBufferHndl p_BufHndl)
{
    p_BufHndl->m_In  = 0;
    p_BufHndl->m_Out = 0;
}

void cb_InitMutex(void);

void cb_ClearBufferLock(CircBufferHndl p_BufHndl);

int  cb_AllocBuffer(CircBufferHndl p_BufHndl, size_t p_Size, gfp_t p_GfpMask);
void cb_DisconnectBuffer(CircBufferHndl p_BufHndl);
uint32_t cb_PutBlock(CircBufferHndl p_BufHndl, BufferElement *p_Block, uint32_t p_BlkSize);
uint32_t cb_PutBlockLock(CircBufferHndl p_BufHndl, BufferElement *p_Block, uint32_t p_BlkSize);
uint32_t cb_PutElement(CircBufferHndl p_BufHndl, const BufferElement p_Val);
uint32_t cb_PutElementLock(CircBufferHndl p_BufHndl, const BufferElement p_Val);

uint32_t cb_GetBlock(CircBufferHndl p_BufHndl, BufferElement *p_Block, uint32_t p_BlkSize);
uint32_t cb_GetBlockLock(CircBufferHndl p_BufHndl, BufferElement *p_Block, uint32_t p_BlkSize);
uint32_t cb_GetElement(CircBufferHndl p_BufHndl, BufferElement *p_Val);
uint32_t cb_GetElementLock(CircBufferHndl p_BufHndl, BufferElement *p_Val);

void cb_PrintCircBuf(CircBufferHndl p_BufHndl, const char *p_pszHeader);

#endif  /*_VIK_CIRCULAR_BUFFER_H_*/
