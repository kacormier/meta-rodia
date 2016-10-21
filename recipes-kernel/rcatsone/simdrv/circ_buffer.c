
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>     // kmalloc()
#include <linux/errno.h>    // error codes
#include <linux/mutex.h>
#include "circ_buffer.h"

// In original version of driver all processing was initiated
// in interrupt handlers therefore the legacy
// spin_lock_irqsave
// <work>
// spin_unlock_irqrestore pattern; replace with mutex

// Our global buffer mutex
struct mutex gBufferMutex;

// empty condition m_In == m_Out
// full condition  mIn+1 == m_Out;
//
// increments m_In counter with wraping
//
static inline void cb_MoveIn(CircBufferHndl p_BufHndl, uint32_t p_Val)
{
    p_BufHndl->m_In = (p_BufHndl->m_In + p_Val) & (p_BufHndl->m_Size-1);
}
//
// increments m_Out counter with wraping
//
static inline void cb_MoveOut(CircBufferHndl p_BufHndl, uint32_t p_Val)
{
    p_BufHndl->m_Out = (p_BufHndl->m_Out + p_Val) & (p_BufHndl->m_Size-1);
}

//
// struct CircBuffer should be allocated outside
// space for data storage will be allocated in this function
//  p_Size - circ. buffer capacity in BufferElement elements
//           advised size will be adjusted to power of 2
//
int cb_AllocBuffer(CircBufferHndl p_BufHndl, size_t p_Size, gfp_t p_GfpMask)
{
    BufferElement  *pBuffer;

    /*
     * round up to the next power of 2, since our 'let the indices
     * wrap' tachnique works only in this case.
     */
    // recalculate p_Size to next power of 2
    if (p_Size & (p_Size - 1))
        p_Size = roundup_pow_of_two(p_Size);

    pBuffer = kmalloc(p_Size*sizeof(BufferElement), p_GfpMask);
    if (!pBuffer)
        return -ENOMEM;

    p_BufHndl->m_Buffer = pBuffer;
    p_BufHndl->m_Size = p_Size;
    p_BufHndl->m_In   = 0;
    p_BufHndl->m_Out  = 0;
    spin_lock_init(&p_BufHndl->m_Lock);

    return 0;
}
//
//  destructor - free all allocated memory
//
void cb_DisconnectBuffer(CircBufferHndl p_BufHndl)
{
    if ( p_BufHndl->m_Buffer != NULL )
        kfree((void*)p_BufHndl->m_Buffer);

    p_BufHndl->m_Buffer = NULL;
    p_BufHndl->m_In     = 0;
    p_BufHndl->m_Out    = 0;
    p_BufHndl->m_Size   = 0;
}
//
//  p_BlkSize - counter in BufferElement elements
//   Funtion returns nuber of elements those was put to the buffer
//
uint32_t cb_PutBlock(CircBufferHndl p_BufHndl, BufferElement *p_Block, uint32_t p_BlkSize)
{
    uint32_t    uLen;

    // we can only put m_Size - 1 element to the buffer
    // this will allow as to distinguish buffer empty and buffer full situations
    p_BlkSize = min(p_BlkSize, (p_BufHndl->m_Size - 1) - cb_GetElementCount(p_BufHndl));

    // put first part of the block
    uLen = min(p_BlkSize, p_BufHndl->m_Size - p_BufHndl->m_In);
    memcpy(&p_BufHndl->m_Buffer[p_BufHndl->m_In], p_Block, uLen * sizeof(BufferElement));
    // put remain block data, if any
    memcpy(p_BufHndl->m_Buffer, &p_Block[uLen], (p_BlkSize - uLen) * sizeof(BufferElement));

    cb_MoveIn(p_BufHndl, p_BlkSize);

    return p_BlkSize;
}
//
//  cb_PutBlock with IRQ save locking
//
uint32_t cb_PutBlockLock(CircBufferHndl p_BufHndl, BufferElement *p_Block, uint32_t p_BlkSize)
{
    unsigned long   ulFlags;
    uint32_t        uRet;

    mutex_lock(&gBufferMutex);
    uRet = cb_PutBlock(p_BufHndl, p_Block, p_BlkSize);
    mutex_unlock(&gBufferMutex);

    return uRet;
}
//
//
//
uint32_t cb_PutElement(CircBufferHndl p_BufHndl, const BufferElement p_Val)
{
    if ( p_BufHndl->m_Size - 1 <= cb_GetElementCount(p_BufHndl))
        return 0;
    p_BufHndl->m_Buffer[p_BufHndl->m_In] = p_Val;
    p_BufHndl->m_In = (p_BufHndl->m_In + 1) & (p_BufHndl->m_Size-1);
/*{
char str[40];
sprintf(str, "cb_PutElement 0x%X", p_Val);
cb_PrintCircBuf(p_BufHndl, str);
}*/
    return 1;
}
//
//
//
uint32_t cb_PutElementLock(CircBufferHndl p_BufHndl, const BufferElement p_Val)
{
    unsigned long   ulFlags;
    uint32_t        uRet;

    mutex_lock(&gBufferMutex);
    uRet = cb_PutElement(p_BufHndl, p_Val);
    mutex_unlock(&gBufferMutex);
    return uRet;
}
//
//  copy pBlkSize or less buffer elements to the pBlock buffer
//
uint32_t cb_GetBlock(CircBufferHndl p_BufHndl, BufferElement *p_Block, uint32_t p_BlkSize)
{
    uint32_t    uLen;

    p_BlkSize = min(p_BlkSize, cb_GetElementCount(p_BufHndl));

    /* first get the data from fifo->out until the end of the buffer */
    uLen = min(p_BlkSize, p_BufHndl->m_Size - p_BufHndl->m_Out);
    memcpy(p_Block, &p_BufHndl->m_Buffer[p_BufHndl->m_Out], uLen * sizeof(BufferElement));

    /* then get the rest (if any) from the beginning of the buffer */
    memcpy(&p_Block[uLen], p_BufHndl->m_Buffer, (p_BlkSize - uLen) * sizeof(BufferElement));

    cb_MoveOut(p_BufHndl, p_BlkSize);

    return p_BlkSize;
}
//
//  cb_GetBlock with IRQ save locking
//
uint32_t cb_GetBlockLock(CircBufferHndl p_BufHndl, BufferElement *p_Block, uint32_t p_BlkSize)
{
    unsigned long   ulFlags;
    uint32_t        uRet;

    mutex_lock(&gBufferMutex);
    uRet = cb_GetBlock(p_BufHndl, p_Block, p_BlkSize);
    mutex_unlock(&gBufferMutex);
    return uRet;
}
//
//
//
uint32_t cb_GetElement(CircBufferHndl p_BufHndl, BufferElement *p_Val)
{
    if ( cb_IsEmpty(p_BufHndl))
    {
        *p_Val = 0;
        return 0;
    }
    *p_Val = p_BufHndl->m_Buffer[p_BufHndl->m_Out];
    p_BufHndl->m_Out = (p_BufHndl->m_Out + 1) & (p_BufHndl->m_Size-1);
/*{
char str[40];
sprintf(str, "cb_GetElement 0x%X", *p_Val);
cb_PrintCircBuf(p_BufHndl, str);
}*/
    return 1;
}
//
//
//
uint32_t cb_GetElementLock(CircBufferHndl p_BufHndl, BufferElement *p_Val)
{
    unsigned long   ulFlags;
    uint32_t        uRet;

    mutex_lock(&gBufferMutex);
    uRet = cb_GetElement(p_BufHndl, p_Val);
    mutex_unlock(&gBufferMutex);
    return uRet;
}
//
// clear buffer with IRQ save locking
//
void cb_ClearBufferLock(CircBufferHndl p_BufHndl)
{
    unsigned long   ulFlags;

    mutex_lock(&gBufferMutex);
    cb_ClearBuffer(p_BufHndl);
    mutex_unlock(&gBufferMutex);
}
//
//  DEBUG print
//
void cb_PrintCircBuf(CircBufferHndl p_BufHndl, const char *p_pszHeader)
{
    printk( "%s: buf=0x%X, size=%d; In=%d; Out=%d; Cnt=%d\n", p_pszHeader,
            (uint32_t)(p_BufHndl->m_Buffer), p_BufHndl->m_Size,
            p_BufHndl->m_In, p_BufHndl->m_Out,
            cb_GetElementCount(p_BufHndl));
}

void
cb_InitMutex(void)
{
    mutex_init(&gBufferMutex);
}