/*
 *
 *  Copyright (c) 2015 RCATSOne Incorporated.  All rights reserved.
 *
 */
#include <ctype.h>
#include "csema.h"
#include "ctask.h"
#include "cproject.h" // CLKTICK
#include <fcntl.h>      // O_RDONLY
#include <sys/ioctl.h>
#include <errno.h>
#include "PhoneSIMClient.h"
#include <netinet/tcp.h>    // TCP_NODELAY
#include "fpga_ph.h"        // fpga_phone_sim_present(), fpga_map_phone_sim()
#include "fpgaRegisterAccess.h"
#include "../../LinuxDrivers/phoenix/sim/simdrv.h"   // assumes LinuxDrivers is checked out under rcats-linux
#include "multi_rcats.h"
#define _SIM_CLIENT_
#include "MISClient.h"
#include "PersistentSettings.h" // MIServerSearchList, local_sim
#include "phone.h" // ph_Phone
#include "util.h" // error
#include "sim_sw.h" // SIM_SW_COM
#include "LKDNetDB.h"
#include "casa_mem_check.h"
#include "CMiDefaultPath.h"
#include "IRemoteSimUser.h"      // uses IRemoteSimUserr

#define WORKING_EF      0
#define INTERNAL_EF     1
#define LINEAR_FIXED    2
#define CYCLIC          6

extern void SIMClientMain(void);                                                        // PhoneSIMClient.c
extern void set_phone_flags(int simLinkSlot, unsigned int flag, BOOL clearFlag);    // PhoneSIMClient.c
extern void ProcessRspToPhone(int simLinkSlot, unsigned char * rspPDU, int rspSize);    // PhoneSIMClient.c
extern int cellPhoneToSimLinkSlot(int phoneID);                 // PhoneSIMClient.c

extern PhoneSession * Phones[];                                                        // PhoneSIMClient.c

// Global Composites for Conditional State Assignments
Span_State  Blocking[] = {Span_Idle, Span_Error};
Span_State  Connecting[] = {Span_ShouldConnect, Span_Connecting};
Span_State  Disconnecting[] = {Span_ShouldDisconnect, Span_Disconnecting};

int  getCmdLen(unsigned char * pCmdPdu);
void invalidateCache(Spanner_Client * Span);

//CRE increased array size  ATR length byte + TS byte + 32 max Tx bytes = 34 (6.1 ISO 7816-3)
unsigned char ATR[34] = {
    0,    // 0    length of ATR
    0x3B, // 1    TS    Fixed ATR Initial Character
    0,    // 2    T0    Will be set to 0x9X, where 9 specifies TA(1) and TD(1) present and X is number of History Bytes (max 15)
    0,    // 3    TA(1) Will be set to 0xFD, where F is the clock rate conversion factor and D is the baud rate adjustment factor
    0x40  // 4    TD(1) Specifies TC(2) present and T=0 Protocol
          // 5    TC(2) Will be set to Work Waiting Time
          // 6-20       Will be set to History Bytes
};

/*
 * Spanner task resources needed under RTXC
 */

//! define priority for spanner tasks
#define SPANNER_TASK_PRIORITY    TASK_PRIORITY_HIGHEST

//! define priority for SIM Client task
#define SIM_CLIENT_TASK_PRIORITY TASK_PRIORITY_HIGHEST

void Spanner_Lock(Spanner_Client *Span)
{
    KS_lockw(Span->Mutex);
}

void Spanner_UnLock(Spanner_Client *Span)
{
    KS_unlock(Span->Mutex);
}

void Spanner_SetState(Spanner_Client * Span, Span_State State)
{
    // int rtn;
    KS_lockw(Span->Mutex);
    ddebug2(DBG2_SIM_SERVER, ("Spanner_SetState: span %p was state %d is now state %d\n", (void *)Span, (int)Span->State, (int)State));
    Span->State = State;
    // rtn = KS_signal(Span->Condition);
    //KS_signal(Span->Condition);   // --> grl, can we reverse this statement and the next one?  This would prevent a little thrashing
    // bugfix 3427 (see our header file):
    pthread_mutex_lock(&Span->condition_mutex);
    pthread_cond_broadcast(&Span->condition);
    pthread_mutex_unlock(&Span->condition_mutex);    // not really how it should work, but we're shoe-horning this in as a fix
    KS_unlock(Span->Mutex);
    // ddebug2(DBG2_SIM_SERVER, ("Spanner_SetState: span %p signal return %d\n", (void *)Span, rtn));
}

/* There is no good way to "watch" the state a-la a debugger and abort an operation */
/*   mid instruction.  Setting the state conditionally relieves a race condition.  In   */
/*   essence, you see an instruction and act on it.  Then, when it comes time to update */
/*   the state, the update only takes if the instruction hadn't changed.  Now a change  */
/*   won't actually abort a connection (for example) but the change from ShouldConnect  */
/*   to something else won't be overwritten by the complete connection switching to     */
/*   the Connected state at the moment of completion.                   */
void Spanner_SetStateIf(Spanner_Client * Span, Span_State NewState, Span_State OldState)
{
    // int rtn;
    KS_lockw(Span->Mutex);
    if (Span->State == OldState)    {
        Span->State = NewState;
        ddebug2(DBG2_SIM_SERVER,("Spanner_SetStateIf: span %p OldState %d NewState %d\n",(void *)Span,(int)OldState,(int)NewState));
    }
    /* I Moved this back out of the conditional.  This "will" cause spurious-but harmless wakeup events */
    /* but it will also allow this routine to "tend to" forward wakups between threads if/when more than    */
    /* one thread ends up waiting on the condition.  This may be safe to move back later.  Rob.     */
    //KS_signal(Span->Condition);
    // bugfix 3427 (see our header file):
    pthread_mutex_lock(&Span->condition_mutex);
    pthread_cond_broadcast(&Span->condition);
    pthread_mutex_unlock(&Span->condition_mutex);    // not really how it should work, but we're shoe-horning this in as a fix
    // rtn = KS_signal(Span->Condition);
    KS_unlock(Span->Mutex);
    // ddebug2(DBG2_SIM_SERVER, ("Spanner_SetStateIf: span %p signal return %d\n", (void *)Span, rtn));
}

void Spanner_SetStateIfAny(Spanner_Client * Span, Span_State NewState, unsigned int state_count, Span_State PredicateStates[])
{
    unsigned int counter;
    unsigned int Match = 0;
    // int rtn;

    KS_lockw(Span->Mutex);
    for (counter = 0; counter < state_count && Match == 0; ++counter)   {
        if (Span->State == PredicateStates[counter])    {
            ++Match;
        }
    }
    if (Match)  {
        Span->State = NewState;
    }
    //KS_signal(Span->Condition);  // See note about unconditional signaling in Spanner_SetStateIf()
    // bugfix 3427 (see our header file):
    pthread_mutex_lock(&Span->condition_mutex);
    pthread_cond_broadcast(&Span->condition);
    pthread_mutex_unlock(&Span->condition_mutex);    // not really how it should work, but we're shoe-horning this in as a fix
    // rtn = KS_signal(Span->Condition);  // See note about unconditional signaling in Spanner_SetStateIf()
    KS_unlock(Span->Mutex);
    // ddebug2(DBG2_SIM_SERVER, ("Spanner_SetStateIfAny: span %p signal return %d\n", (void *)Span, rtn));
}

/* Wait until changes out of this state */
void Spanner_WaitWhileState(Spanner_Client * Span, Span_State State)
{
    KS_lockw(Span->Mutex);
    while (Span->State == State)    {
        // ddebug2(DBG2_SIM_SERVER, ("Spanner_WaitWhileState: span %p Wait State %d Present State %d\n", (void *)Span, (int)State, (int)Span->State));
        KS_unlock(Span->Mutex);
        //KS_wait(Span->Condition);
        // bugfix 3427 (see our header file):
        pthread_mutex_lock(&Span->condition_mutex);
        pthread_cond_wait(&Span->condition, &Span->condition_mutex);
        pthread_mutex_unlock(&Span->condition_mutex);    // not really how it should work, but we're shoe-horning this in as a fix
        KS_lockw(Span->Mutex);
    }
    KS_unlock(Span->Mutex);
    ddebug2(DBG2_SIM_SERVER, ("Spanner_WaitWhileState: span %p state achieved\n", (void *)Span));
}

/* Wait until changes out of this set of states */
void Spanner_WaitWhileStates(Spanner_Client * Span, unsigned int state_count, Span_State State[])
{
    if (state_count)    {
        int Match;
        unsigned int    counter;
        KS_lockw(Span->Mutex);
        do  {
            Match = 0;
            for (counter = 0; (Match == 0) && (counter < state_count); ++counter)   {
                if (Span->State == State[counter])  {
                    ++Match;
                } // else
                    // ddebug2(DBG2_SIM_SERVER, ("Spanner_WaitWhileStates: state %d\n", Span->State));
            }
            if (Match)  {
                KS_unlock(Span->Mutex);
                //KS_wait(Span->Condition);
                // bugfix 3427 (see our header file):
                pthread_mutex_lock(&Span->condition_mutex);
                pthread_cond_wait(&Span->condition, &Span->condition_mutex);
                pthread_mutex_unlock(&Span->condition_mutex);    // not really how it should work, but we're shoe-horning this in as a fix
                KS_lockw(Span->Mutex);
                // ddebug2(DBG2_SIM_SERVER, ("Spanner_WaitWhileStates: state %d after wait\n", Span->State));
            }
        } while(Match);
        KS_unlock(Span->Mutex);
        ddebug2(DBG2_SIM_SERVER, ("Spanner_WaitWhileStates: span %p state achieved\n", (void *)Span));
    }
}

int Spanner_TickCheckState(Spanner_Client * Span)
{
    /* TPF Tick Routine.    */
    if ((Span->State == Span_Connected)
    || (Span->State == Span_Connecting)
    || (Span->State == Span_Disconnecting)) {
        /* Good to block in these states */
        return 0;
    } else  {
        /* Other states are commands just issued so wake up */
        return 1;
    }
}

int Spanner_AckCallback(Spanner_Client * Span)
{
    /* Count acks from MIServer for SIM commands sent by probe.    */
    Span->misToProbeAcks++;
    return 0;
}

SOCKET  GetConnection(char * host)
{
	debug("/-------------GetConnection--------------\\");
    SOCKET          S = t_socket(PF_INET,SOCK_STREAM,0);
    struct sockaddr_in  Address = {0};

    if (S < 0) {
        ddebug2(DBG2_SIM_SERVER, ("GetConnection: could not allocate a socket"));
        return -1;
    }
    outStatus("GetConnection: Trying MIServer \"%s\"", host);
    if (MakeAddress(&Address,host,BRIDGEPORT,0) != 0) {
        ddebug2(DBG2_SIM_SERVER, ("GetConnection: MakeAddress failed"));
        return -1;
    }
    if (t_ConnectTimed(S,(struct sockaddr *)&Address,8,24,"Trying to connect...","Timed out\n") == 0)   {
        int no_nagle = 1;
        setsockopt((int)S,SOL_TCP,TCP_NODELAY,&no_nagle,sizeof(no_nagle));
        debug("\\-------------GetConnection--------------/");
        return S;
    } else  {
        ddebug2(DBG2_SIM_SERVER, ("GetConnection: connect failed"));
        error(eWARN,"MIServer: Couldn't Connect to MIServer \"%s\"",host);
        debug("\\-------------GetConnection--------------/");
        return -1;
    }
}

// warning: these are externed in the header file and used elsewhere
Spanner_Client Spanner[MAX_CLIENT_SESSIONS];

ResultCode connectMIServer(int simLinkSlot, const char *sim)
{
	debug("/---------------connectMIServer-------------\\");
    int cnt;
    Spanner_Client *Span = &Spanner[simLinkSlot];
    Span->Flags &= (~SPAN_FLAG_INHIBIT); // if MIServer disconnects, disconnect SIM from phone
    if (Span->SIMConnectToken)
        free(Span->SIMConnectToken);
    if ((Span->SIMConnectToken = (char *)malloc(strlen(sim)+1)) != NULL) {
        Span->SIMConnectToken[0] = '\0';
        strncat(Span->SIMConnectToken,sim,strlen(sim)+1);
    }
    // The Spanner will disconnect any existing session before trying to create a new session.

    // SpannerThread will see this change in state and try connecting
    debug("| connectMIServer: set state to Span_ShouldConnect...");
    Spanner_SetState(Span,Span_ShouldConnect);

    // delays in MDS may cause problems handing off to SpannerThread
    debug("| connectMIServer: start delay...");
    for (cnt = 0; cnt <10 && ((Span->State == Span_ShouldConnect) || (Span->State == Span_Connecting)); cnt++)
    {
        debug("| connectMIServer: delay # %i ", cnt);
        KS_delay(SELFTASK, 500 / CLKTICK); // Required else the following wait will see the semaphore change and SpannerThread won't
        debug("| connectMIServer: wait for connecting state...");
        Spanner_WaitWhileStates(Span,(sizeof(Connecting)/sizeof(Connecting[0])),Connecting);    // Till SpannerThread connects (or not)
        debug("| connectMIServer: done waiting");
    }
    if (Span->State != Span_Connected)
    {
    	debug("\\---------------connectMIServer (failed to connect!!!) -------------/");
        return MAKE_RCATS_ERROR(ERROR_INVALID_CMD_RESP, "could not acquire SIM from the MI server");
    }
    debug("\\---------------connectMIServer (connected)-------------/");
    return ERROR_OK;
}

void cpPin(char ** dstPtr, char * src)
{
    int len = src[7] ? 8 : strlen(src);
    char *  dst = *dstPtr;
    if(dst)
        free(dst);
    if(len) {
        dst = (char *)malloc(len + 1);
        strncpy(dst, src, len);
        dst[len] = 0;
        *dstPtr = dst;
    } else
        *dstPtr = 0;
}

/*
 * NOTE: redirect must be the address of a pointer, not an array
 *  During a redirect, the contents of the pointer are freed and
 *  a new buffer is allocated.
 */
ConnectionProblemType Span_TryNameOnServer(Spanner_Client *Span, char *Server, char **redirect)
{
	debug("/---------------Span_TryNameOnServer-----------------\\");
	ConnectionProblemType   retval = SBPT_Unknown;
    SOCKET          S = GetConnection(Server);
    unsigned char *     header;

    ddebug2(DBG2_SIM_SERVER, ("| Span_TryNameOnServer: span %p, server %s\n", (void *)Span, (char *)Server));

    if (S >= 0)
    {
        ddebug2(DBG2_SIM_SERVER, ("| Span_TryNameOnServer: span %p, connected to server\n", (void *)Span));
        Span->tfp = TFP_StartProtocol(S,
            SIMB_UVLength,
            SIMB_UVString,
            Span,
            (TFP_Callback)Spanner_TickCheckState,
            (TFP_Callback)Spanner_AckCallback,
            (TFP_MutexProc)Spanner_Lock,
            (TFP_MutexProc)Spanner_UnLock,
            NULL,0);
        if (Span->tfp)
        {
            // jdubovsky note: send "request option"
            unsigned char   Buffer[66];
            Buffer[0] = SBMT_RequestOption;
            Buffer[1] = getDebug2Flags() & DBG2_NO_SIM_CACHING ? 0 : USE_CACHING;
            BOOL        wantUMTS = FALSE;
            BOOL        wantClockStopATR = FALSE;

            casabyte::shared_ptr<casabyte::RcatsPhone> pPhone = DeviceFromIndex(ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID), std::nothrow);      // may be NULL
            casabyte::IRemoteSimUser *pSimUsingPhone = dynamic_cast<casabyte::IRemoteSimUser *>(pPhone.get());       // may be NULL (whether pPhone was or not)
            if (!pSimUsingPhone)
                // assume phone doesn't want UMTS (3G)
                wantUMTS = FALSE;
            else
                // ask the phone if it wants UMTS (3G)
                wantUMTS = pSimUsingPhone->WantUmts();
            debug("| Span_TryNameOnServer: MISClient wantUMTS = %u", wantUMTS);

            if(wantUMTS)
                Buffer[1] |= IS_UMTS_AVAIL;

            if ( pSimUsingPhone && pSimUsingPhone->WantClockStopATR() )
               wantClockStopATR = TRUE;

            char strSiteName[64];
            GetGlobalNVR_string(e_site_name, strSiteName, 64);
            unsigned int strSiteNameLen = strlen(strSiteName) + 1;
            memcpy(&Buffer[2], strSiteName, strSiteNameLen);

            TFP_Packet  Packet;
            TFP_Return  xferstat;
            memset(&Packet,0,sizeof(Packet));
            Packet.Data = Buffer;
            Packet.BufferLength = sizeof(Buffer);
            Packet.SideBand = 1;
            Packet.PacketLength = Packet.DataLength = strSiteNameLen + 2;

            outStatus("| Span_TryNameOnServer: MIServer Session Starting");
            xferstat = TFP_SendPacket(Span->tfp,&Packet);
            Span->probeToMisPkts++;
            // jdubovsky note: if we sent the "connect option" okay, go ahead and request the SIM
            if ((xferstat == TFP_PACKET_OK) || ((xferstat == TFP_PACKET_TIMEOUT) && (Span->State == Span_Connecting)))
            {
                // jdubovsky note: send "connect (to SIM?) request"
                unsigned char * dst = Buffer;
                unsigned char * src = (unsigned char *)Span->SIMName;
                int     resolved = 0;
                //int       connected = 0;
                outStatus("Requesting Sim %s",src);
                *dst++ = SBMT_ConnectRequest;
                while (*src != '\0')    {
                    *dst++ = *src++;
                }
                Packet.PacketLength = Packet.DataLength = dst - Buffer;
                xferstat = TFP_SendPacket(Span->tfp,&Packet);
                Span->probeToMisPkts++;
                // jdubovsky note: until we resolve the SIM (or die), keep trying to receive data from the SIM server
                while (((xferstat == TFP_PACKET_OK)
                    || ((xferstat == TFP_PACKET_TIMEOUT) && (Span->State == Span_Connecting)))
                    && (!resolved))
                {
                	debug("| Span_TryNameOnServer: ... waiting for packet from server...");
                    // jdubovsky note: wait for a packet from the server
                    if ((xferstat = TFP_ReceivePacket(Span->tfp,&Packet)) == TFP_PACKET_OK)
                    {
                    	debug("| Span_TryNameOnServer: received packet OK (packetLength: %lu; dataLen: %i; sideband: %i...",
                    			Packet.PacketLength, Packet.DataLength, Packet.SideBand);
                       // jdubovsky note: is it a control (SideBand) packet?
                        if ((Packet.PacketLength) && (Packet.DataLength) && (Packet.SideBand))
                        {
                        	debug("| Span_TryNameOnServer: check packet data...");
                            unsigned int    ATRHistLength;  //number of Historical Characters (we get these from the MI Server)
                            uint8_t ATRNonHistLength; //number of non-historical characters in ATR
                            switch (Packet.Data[0])
                            {
                            // jdubovsky note: when we get ConnectOK(_UMTS), SIM connection is done (we have all the SIM data, etc.)
                            case SBMT_ConnectOK:
                            case SBMT_ConnectOK_UMTS:
                                ddebug2(DBG2_SIM_SERVER, ("Span_TryNameOnServer: span %p, SBMT_ConnectOK received", (void *)Span));
                                ATRHistLength = (unsigned char)_min(15,_min(Packet.PacketLength-1,Packet.DataLength-1));
                                ATRNonHistLength = 5;
                                ATR[0] = ATRHistLength + ATRNonHistLength;
                                ATR[1] = 0x3B; //TS = Direct Convention
                                ATR[2] = 0x90 | ATRHistLength;
                                memcpy(&ATR[6], &(Packet.Data[1]), ATRHistLength);  //copy Historical Characters we got from MI Server
                                // Any baud rate divisor settings made here are only used after PPS response.
                                // If not set here, baud rate divisor settings in SIMBridge_Init continue to be used.
                                if(Packet.Data[0]==SBMT_ConnectOK_UMTS && wantUMTS) {
                                   if ( wantClockStopATR )
                                   {
                                      //NOTE: this ATR matches the ATR sent by local SIMs we have from AT&T and Rogers
                                      //We're adding TD2 (switch to T=15) and TA3 (Clock Stop Indicator and Class Indicator).  
                                      //The switch to T=15 requires us to add the TCK checksum byte.
                                      //WARNING: It does NOT include the TC2 Work Waiting Integer = 0xFE.  This could cause timeouts 
                                      //because our sim driver may assume it is set!  ISO 7816 specifies that the default is 10.
                                      ATRNonHistLength = 7;
                                      ATR[0] = ATRHistLength + ATRNonHistLength; 
                                      ATR[2] = 0x90 | ATRHistLength;
                                      ATR[3] = 0x95; // F = 512, D = 16 => 102 kbps
                                      //CRE - The MC8700 won't work if we specify the Work Waiting Integer, even though the MC8775V will
                                      //ATR[4] = 0xC0; //TD2 | TC2
                                      //ATR[5] = 0xFE; //Work Waiting Integer
                                      //RCT-69 ATR format for all modules with wantClockStopATR=true needs a change. So below 1 line commented as part of the fix
									  //ATR[4] = 0x80; //TD2
									  //RCT-69 ATR format for all modules with wantClockStopATR=true needs a change. So below 1 line added as part of the fix
									  ATR[4] = 0x40;
                                      ATR[5] = 0x1F; //TA3 / T=15
                                      ATR[6] = 0xC7; //C7 is clock stop supported = no preference and class indicator = A,B,C
                                      memcpy(&ATR[7], &(Packet.Data[1]), ATRHistLength); //we added one Interface Char so we have to recopy Historical chars

                                      //checksum is XOR of sum of bytes T0 through byte before TCK (TCK is required because of the T=15)
                                      uint8_t byTck = 0;
                                      for( uint8_t i=2; i<ATRHistLength+ATRNonHistLength; i++)
                                         byTck ^= ATR[i];
                                      ATR[ATRHistLength+ATRNonHistLength] = byTck;   //add TCK to end of ATR
                                   }
                                   else
                                   {
                                      ATR[3] = 0x95; // F = 512, D = 16 => 102 kbps
                                      ATR[4] = 0x40;
                                      ATR[5] = 0xFE;
                                   }
                                   Phones[Span->simLinkSlot]->OpClass = CLASS_00_MATCH;
                                   Phones[Span->simLinkSlot]->baudRate = Phones[Span->simLinkSlot]->baudRateUmts;
                                } else {
                                    ATR[3] = 0X11;
                                    ATR[4] = 0x40;
                                    ATR[5] = 0xFF;
                                    Phones[Span->simLinkSlot]->OpClass = CLASS_A0_MATCH;
                                    Phones[Span->simLinkSlot]->baudRate = Phones[Span->simLinkSlot]->atrBaudRate;
                                }
                                if(getDebug2Flags() & DBG2_SIM_SIMU)
                                    debugDump(ATR+1, ATR[0], "ATR:", dsONESPACED);
                                if (ioctl(Phones[Span->simLinkSlot]->writeFD, SIMDRV_ATR_FULL, ATR) < 0)
                                    error(eWARN,"Phone %c ATR save failed: errno %i",
                                      (char)('A' + ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID)), errno);

                                retval = Span->Error = SBPT_ALL_OK;
                                resolved=1;
                                //connected=1;
                                break;
                            case SBMT_ConnectProblem:
                                retval = Span->Error = SBPT_Unknown;
                                ddebug2(DBG2_SIM_SERVER, ("| Span_TryNameOnServer: span %p, SBMT_ConnectProblem received", (void *)Span));
                                if (Packet.DataLength > 1)  {
                                    retval = Span->Error = static_cast<ConnectionProblemType>(Packet.Data[1]);
                                    switch (Span->Error)
                                    {
                                    case SBPT_Connect_SIMBusy:
                                        error(eWARN,"MIClient: MIServer \"%s\" Reports All SIMs Named \"%s\" In-Use/Busy",Server,Span->SIMName);
                                        break;
                                    case SBPT_Connect_PinNotSetYet:
                                        error(eWARN,"MIClient: MI Server \"%s\" reports SIM named \"%s\" requires a PIN, but not set yet",Server,Span->SIMName);
                                        break;
                                    case SBPT_Connect_BadPinValue:
                                        error(eWARN,"MIClient: MI Server \"%s\" reports SIM named \"%s\" requires a PIN, but incorrect PIN entered",Server,Span->SIMName);
                                        break;
                                    case SBPT_Connect_SIMBlocked:
                                        error(eWARN,"MIClient: MI Server \"%s\" reports SIM named \"%s\" is blocked because incorrect PIN entered 3 times",Server,Span->SIMName);
                                        break;
                                    case SBPT_Connect_SIMNotFound:
                                        error(eWARN,"MIClient: MIServer \"%s\" Doesn't Recognize SIM Name \"%s\"",Server,Span->SIMName);
                                        break;
                                    case SBPT_Connect_Redirect:
                                        if (*redirect)  {
                                            // shouldn't ever happen , but if it does, it's would be a memory leak, so...
                                            farfree(*redirect);
                                            *redirect = NULL;
                                        }
                                        if (Packet.DataLength > 2)
                                        {
                                            *redirect = (char *)farmalloc(Packet.DataLength - 1);
                                            if (*redirect)  {
                                                memcpy(*redirect,Packet.Data+2,Packet.DataLength - 2);
                                                *(*redirect + Packet.DataLength - 1) = '\0';
                                                outStatus("MIClient: Search for \"%s\" redirected to MIServer \"%s\"",Span->SIMName,*redirect);
                                            } else {
                                                ddebug2(DBG2_SIM_SERVER,("Span_TryNameOnServer: : Out of Heap during Redirect"));
                                                error(eWARN,"MIClient: Out of Heap during Redirect");
                                            }
                                        } else  {
                                            ddebug2(DBG2_SIM_SERVER, ("Span_TryNameOnServer: : Server Error: Redirect to Nowhere"));
                                            error(eWARN,"MIClient: Server Error: Redirect to Nowhere");
                                        }
                                        break;
                                    default:
                                        error(eWARN,"MIClient: Unknown Error occured Attaching to SIM \"%s\" on server \"%s\"",Span->SIMName,Server);
                                        break;
                                    }
                                } else  {
                                    error(eWARN,"MIClient: Unknown Protocol Error occured Attaching to SIM \"%s\" on server \"%s\"",Span->SIMName,Server);
                                    Span->Error = SBPT_Unknown;
                                }
                                TFP_EndProtocol(Span->tfp);
                                Span->tfp = NULL;
                                t_socketclose(S);
                                S = 0;             // so we don't try to close it again before we return
                                resolved = 1;
                                break;
                            default:
                                break;
                            }
                        }
                    }
                    // jdubovsky note: is packet just SIM data (the cache image)?
                    else if ((xferstat == TFP_PACKET_CONT) && (Packet.Data[0] == SBMT_CacheImage))
                    {
                        outStatus("| Span_TryNameOnServer: SIM Cache Download Starting");
                        if (Span->cacheImage) {
                            farfree(Span->cacheImage);
                        }
                        Span->cacheImage = (unsigned char *)farmalloc(Packet.PacketLength - 1);
                        if (Span->cacheImage != NULL)
                        {
                            memcpy(Span->cacheImage, &Packet.Data[1], Packet.DataLength - 1);
                            Packet.Data = Span->cacheImage + Packet.DataLength - 1;
                            Packet.BufferLength = Packet.PacketLength - Packet.DataLength;
                            // jdubovsky note: receive all the SIM data (cache image)
                            while ((xferstat = TFP_ReceivePacketContinue(Span->tfp,&Packet)) == TFP_PACKET_CONT)
                            {
                                Packet.Data += Packet.DataLength;
                                Packet.BufferLength -= Packet.DataLength;
                            }
                            if (xferstat == TFP_PACKET_OK)
                            {
                                 // jdubovsky note: we got all the SIM data
                                outStatus("Span_TryNameOnServer: SIM Cache Download Successful");
                                ddebug2(DBG2_SIM_SERVER, ("| Span_TryNameOnServer: SIM Cache Download successful, retval = %i", retval));
                                Span->cwd = 0;
                                Span->simReadComplete = FALSE;
                                header = Span->cacheImage;
                                if (*(header+SCH_DIR_AREA_OFFSET) == SIM_CACHE_HEADER_SIZE)
                                {
                                    unsigned int    index1;
                                    for(index1=0; index1<32; index1++)
                                        *(header+index1+SCH_PIN) = (*(header+index1+SCH_PIN)>>1)&0x7F;
                                    for(index1=0; index1<4; index1++)
                                    {
                                        unsigned char * src2 = NULL;
                                        char **         dst2 = NULL;
                                        switch(index1) {
                                        case 0:
                                            src2 = header + SCH_PIN;
                                            dst2 = &(Span->pin);
                                            break;
                                        case 1:
                                            src2 = header + SCH_PUK;
                                            dst2 = &(Span->puk);
                                            break;
                                        case 2:
                                            src2 = header + SCH_PIN2;
                                            dst2 = &(Span->pin2);
                                            break;
                                        case 3:
                                            src2 = header + SCH_PUK2;
                                            dst2 = &(Span->puk2);
                                            break;
                                        }
                                        cpPin(dst2, (char *)src2);
                                    }
                                }
                            }
                            else     // jdubovsky note: downloading the SIM data failed
                            {
                                outStatus("| Span_TryNameOnServer: SIM Cache Download Fail");
                                error(eWARN,"MIClient: Cache Download Error for SIM \"%s\" on server \"%s\"",Span->SIMName,Server);
                                farfree(Span->cacheImage);
                                Span->cacheImage = NULL;
                            }
                            Packet.Data = Buffer;
                            Packet.BufferLength = sizeof(Buffer);
                        } else {
                            // jdubovsky note: farmalloc failed
                            error(eWARN,"MIClient: Cache Download Error, No Local Memory");
                            xferstat = TFP_PACKET_OK;
                            // terminate added 2008jul24 by jdubovsky: out of memory, don't return OK and keep running!
                            std::terminate();
                        }
                    }
                    else
                    {
                       // TFP_ReceivePacket (return value now in xferstat) was neither TFP_PACKET_OK nor
                       //  (TFP_PACKET_CONT and the packet was a cache piece).
                       
                       // bugfix 3496 (missing "SIM Attach Successful" Message): if this is really an
                       //  error (not just a timeout while connecting, etc.), fail with a meaningful error.
                       // note: due to a dumb design, our caller does not report our errors up to the user
                       //  (it just loops N times trying for a successful connection and the last error
                       //  wins), so we have to print something here.
                       switch (xferstat)
                       {
                          case TFP_CALLER_ERROR:
                             Span->Error = retval = SBPT_BadState;
                             error(eFATAL, "SIM client had a bad internal state while connecting to the SIM");
                             break;
                          case TFP_PACKET_ERROR:      // not the best error, but we don't have a value for "socket error"
                             // fall through
                          case TFP_PACKET_DISCON:
                             Span->Error = retval = SBPT_LostConnection;
                             error(eFATAL, "SIM client lost connection with SIM server");
                             break;
                          // the following are probably not errors; this entire switch statement is new,
                          //  so these were previously not handled.  It's probably best to leave the
                          //  old behavior alone unless we know its *really* an error.
                          case TFP_PACKET_OK:
                          case TFP_PACKET_CONT:
                          case TFP_PACKET_TIMEOUT:
                             break;
                          // no default case; let the compiler warn us if we forget an enum.
                       }
                    }
                }       // end while loop
            } else {             // jdubovsky note: the "connect option" failed
                Span->Error = retval = SBPT_Unknown;
                error(eFATAL, "SIM client could not send session start (%i, %i)", xferstat, Span->State);
            }
        } else  {                // jdubovsky note: Span->tfp is NULL
            error(eFATAL, "SIM client could not start TFP");
            Span->Error = retval = SBPT_Unknown;
        }
    } else {                     // jdubovsky note: GetConnection failed
        ddebug2(DBG2_SIM_SERVER, ("Span_TryNameOnServer: span %p, failed to connected to server\n", (void *)Span));
        retval = SBPT_Connect_NoConnection; //fix 3386
    }

    debug("| Span_TryNameOnServer: retval=%i", retval);
    if (retval)
    {
       outStatus("| Span_TryNameOnServer: TryName Complete (error %i)", retval);
      // bugfix 3496 (missing "SIM Attach Successful" Message): if TryName fails,
      //  don't leave the connection to the server open!  The server may hang,
      //  thinking the SIM is acquired.
      if (Span->tfp)
      {
         TFP_EndProtocol(Span->tfp);      // note: frees the pointer
         Span->tfp = NULL;
      }
      if (S)
         t_socketclose(S);
    }
    else
       outStatus("| Span_TryNameOnServer: TryName Complete");
	debug("\\---------------Span_TryNameOnServer-----------------/");
    return retval;
}
//
// Function parses last part of "PA SIM [mmmmm/]sss" command
// and returns pointer to server name or NULL
//             and SIM name in Span->SIMName
//
char *ParseSIMConnectToken(Spanner_Client *Span)
{
    char *  CTcursor;
    char *  ServerName  = NULL;

    if (Span->SIMName)
    {
        /* Discard any old information  */
        farfree(Span->SIMName);
        Span->SIMName = NULL;
    }

    CTcursor = strchr(Span->SIMConnectToken, '/');

    if (CTcursor != NULL)
    {       // command has both server name and SIM name parts
        char *src = Span->SIMConnectToken;
        /** bugfix, ysinada 072606, we were 1 byte short in the malloc below, the extra byte is needed to store the NULL terminator */
        char *dst = ServerName = (char *)farmalloc(CTcursor - Span->SIMConnectToken+1);
        if (dst)
        {
           while (src != CTcursor)
               *dst++ = *src++;
           *dst = '\0';

           ++src;
           Span->SIMName = (char *)farmalloc(strlen(src)+1);
           if (Span->SIMName)
               strcpy(Span->SIMName,src);
        }
    }
    else
    {   // command has only SIM name part
        Span->SIMName = (char *)farmalloc(strlen(Span->SIMConnectToken)+1);
        if (Span->SIMName)
           strcpy(Span->SIMName,Span->SIMConnectToken);
    }

    return ServerName;
}

ConnectionProblemType Span_FindConnectSIM(Spanner_Client *Span)
{
	debug("/----------------Span_FindConnectSIM--------------\\");
    ConnectionProblemType   retval = SBPT_Unknown;      // Presume failure, but don't know why yet.
    ConnectionProblemType   lastRealError = SBPT_Unknown;  // we save the last non-"SIM not found" error for the user
    char *  ServerName  = NULL;
    char *  DeferName   = NULL;
    int     Connected   = 0;
    int     Attempts    = 0;
    int     DeferDepth  = 0;
    char    NameBuffer[SERVER_NAME_MAX+1];
    int     dbg2_sim_server = 0;

    if (getDebug2Flags() & DBG2_SIM_SERVER && !(getDebug2Flags() & DBG2_SIM_SERVER_ALWAYS))
    {
        ddebug2(DBG2_SIM_SERVER, ("Span_FindConnectSIM: Turning off DBG2_SIM_SERVER because TFP timing problem\n"));
        setDebug2Flags( getDebug2Flags() & ~DBG2_SIM_SERVER );
        dbg2_sim_server = 1;
    }

    NameBuffer[0] = '\0';
    NameBuffer[SERVER_NAME_MAX] = '\0'; // Guaranteed Terminal Null in that "+ 1" spot

    // A sim change inferrs that any existing chache is meaningless
	debug("| Span_FindConnectSIM: invalidate cache...");
    invalidateCache(Span);
    ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: invalidate cache before connect"));

    ddebug2(DBG2_SIM_SERVER, ("Span_FindConnectSIM: span %p entered\n", (void *)Span));

    // at that moment we parse only MI server SIM - Local SIM already processed outside
    // SIM address parser. Could be 2 forms
    //  - host_name_or_IP/SIM_name
    //  - SIM_name
	debug("| Span_FindConnectSIM: ParseSIMConnectToken...");
    ServerName = ParseSIMConnectToken(Span);

    // at this point Span->SIMName - contains SIM name from the command line
    //               ServerName - server name. If ServerName==NULL - use default servers
    //
    // - build a list of all MI server to checked for the SIM
    CMiDefaultPath  MiPath(ServerName, LastSuccessfulServer, Span->LastServer);
    unsigned int    n_MiPathIterator = 0;

    while (!Connected && Attempts <= 10)
    {
    	debug("| Span_FindConnectSIM: ... in WHILE loop; check state: %i", Span->State);
        if (Span->State != Span_Connecting) {
            ddebug2(DBG2_SIM_SERVER, ("Span_FindConnectSIM: Span->State == %i, != Span_Connecting (%i)", Span->State, Span_Connecting));
            retval = SBPT_Aborted;
            break;
        }
        if (DeferName)      // server was redirected
        {
            ++DeferDepth;
            strncpy_safe(NameBuffer, DeferName, SERVER_NAME_MAX);
            farfree(DeferName);
            DeferName = NULL;
        }
        else        // no DeferName server was received - get name from the list
        {
            string  NextPath;
            n_MiPathIterator = MiPath.GetNextPath(n_MiPathIterator, NextPath);
            if ( n_MiPathIterator == 0 || NextPath.empty() )
                break;              // done with list of allowed MI Servers
                                    // break from the connecting loop
            strncpy_safe(NameBuffer, NextPath.c_str(), sizeof(NameBuffer));
        }

        debug("| calling Span_TryNameOnServer()... name: %s", NameBuffer);
        if ((retval = Span_TryNameOnServer(Span,NameBuffer,&DeferName)) == SBPT_ALL_OK)
        {
            ddebug2(DBG2_SIM_SERVER, ("Span_FindConnectSIM: Span_TryNameOnServer returned %i, so connected=1", retval));
            Connected = 1;
        }
        else
        {
            ddebug2(DBG2_SIM_SERVER, ("Span_FindConnectSIM: Span_TryNameOnServer returned %i; not yet connected", retval));
            if ( retval == SBPT_Connect_NoConnection )
            {
                ddebug2(DBG2_SIM_SERVER, ("Span_FindConnectSIM: no connection, so no more attempts"));
                //Attempts = 20;  //fix 3386 - leave loop immediately
            }
            // part of bugfix 3496 (missing "SIM Attach Successful" Message): if we get any
            //  error other than "SIM not found", save that error for later return.
            //  If we don't find the SIM on a second/third/etc. server, that error
            //  message is very important to convey what went wrong.
            if (retval != SBPT_Connect_SIMNotFound)
            {
               lastRealError = retval;
            }
            ++Attempts;
        }
    }
    debug("| Span_FindConnectSIM: ... done WHILE loop... connected?%i", Connected);
    if (Connected)
    {
        if (Span->LastServer)
            farfree(Span->LastServer);

        Span->LastServer = (char *)farmalloc(strlen(NameBuffer)+1);
        // strncpy_safe checks all parameters for NULL
        strncpy_safe(Span->LastServer,NameBuffer,strlen(NameBuffer)+1);

        LastSuccessfulServer[0] = '\0';
        strncpy_safe(LastSuccessfulServer,NameBuffer,sizeof(LastSuccessfulServer));
        outStatus("SIM Attach Successful [lastServer: %s]", Span->LastServer);
    }
    if (DeferName)  {
        // By now this should be coppied away or of no importance
        // So plug the potential memory leak
        farfree(DeferName);
    }
    ddebug2(DBG2_SIM_SERVER, ("| Span_FindConnectSIM: span %p exited, return value %d", (void *)Span, (int)retval));
    if (dbg2_sim_server) {
        setDebug2Flags(getDebug2Flags() | DBG2_SIM_SERVER);
        ddebug2(DBG2_SIM_SERVER, ("| Span_FindConnectSIM: Turning back on DBG2_SIM_SERVER\n"));
    }
    // part of bugfix 3496 (missing "SIM Attach Successful" Message): if we got an error (TCP
    //  disconnect, etc.), then tried other servers and only saw "SIM not found", return the
    //  first error, as that's the one that means something (the SIM was found, but we couldn't
    //  use it).
    if (retval != SBPT_ALL_OK && lastRealError != SBPT_Unknown)
    {
    	debug("\\----------------Span_FindConnectSIM (Error: %i) --------------/", lastRealError);
       return lastRealError;
    }
    else
    {
    	debug("\\----------------Span_FindConnectSIM--------------/");
    	return retval;
    }
}

void SpannerThread(void)
{
	debug("/------------------SpannerThread-----------------\\");
    Spanner_Client *Span = (Spanner_Client *)KS_inqtask_arg(SELFTASK);
    TFP_Packet  Packet;
    unsigned char   Buffer[270];

    Packet.Data = Buffer;
    Packet.BufferLength = sizeof(Buffer);
    debug("| SpannerThread: start span %p: pid %d", Span, getpid());
    Spanner_SetState(Span,Span_Idle);
    for (;;)    {
        /* Wait for New Orders  */
    	debug("| SpannerThread: Wait for New Orders ...");
        Spanner_WaitWhileStates(Span,(sizeof(Blocking)/sizeof(Blocking[0])),Blocking);
        // This next line is in here twice.  (yep, ugly... 8-)
        // Once here for the initial connect, and once below for a connect
        // attempt that happens when the span is already connected.
        debug("| SpannerThread: change state to Span_Connecting (from Span_ShouldConnect)");
        Spanner_SetStateIf(Span,Span_Connecting,Span_ShouldConnect);
        /* (Re)Initialize for a command attempt pass */
        Span->Error = SBPT_ALL_OK;
        /* Check Orders in State. So Far just ShouldConnect is valid */
        if (Span->State == Span_Connecting) {
         	 debug("| SpannerThread: state = Span_Connecting ...");
            if (Span->SIMConnectToken != NULL)  {
            	debug("| SpannerThread: calling  Span_FindConnectSIM...");
                if ((Span->Error = Span_FindConnectSIM(Span)) == SBPT_ALL_OK)   {
                	debug("| SpannerThread: > Span_FindConnectSIM succeeded...");
                    ddebug2(DBG2_SIM_SERVER, ("Span_FindConnectSIM succeeded!\n"));
                    Spanner_SetStateIf(Span,Span_Connected,Span_Connecting);
                } else  {
                	debug("| SpannerThread: > Span_FindConnectSIM failed...");
                    ddebug2(DBG2_SIM_SERVER, ("Span_FindConnectSIM failed!\n"));
                    Spanner_SetStateIf(Span,Span_Error,Span_Connecting);
                    KS_delay(SELFTASK, 4000 / CLKTICK); // Required else will get to the above Spanner_WaitWhileStates
                                                        // and the one in connectMIServer of the Shell thread won't release
                                                        // See also Bug #3379 for increase from 2 to 4 seconds.
                }
            } else  {
                Span->Error = SBPT_MissingInformation;
                Spanner_SetState(Span,Span_Error);
            }
        } else if (Span->State != Span_ShouldDisconnect) {
        	debug("| SpannerThread: state <> Span_ShouldDisconnect ...");
            Span->Error = SBPT_BadState;
            Spanner_SetState(Span,Span_Error);
        }
        debug("| SpannerThread: state : %02X ...",Span->State);
        ddebug2(DBG2_SIM_SERVER, ("***** Span State: %02X *****\n",Span->State));

        while (Span->State == Span_Connected)   {
        	debug("| SpannerThread:  in CONNNECTED loop... wait for next packet...");
            TFP_Return ProtoStatus = TFP_ReceivePacket(Span->tfp,&Packet);
            debug("| SpannerThread: > TFP_ReceivePacket returned...");
            if (Span->State != Span_Connected)  {
                // A simple early-escape to prevent the Span->State from being changed by an error event
                break;
            }
            if (ProtoStatus == TFP_PACKET_OK)   {
            	debug("| SpannerThread: > packet received ok; Is sideband? %i...", Packet.SideBand);
                if (Packet.SideBand)    {
                    debug("| SpannerThread: > a sideband; package len: %lu", Packet.PacketLength);
                    if (Packet.PacketLength)    {
                        switch (Packet.Data[0]) {
                        case SBMT_Acknowledge:
                        	debug("| SpannerThread: > SBMT ACK");
                            ddebug2(DBG2_SIM_SERVER, ("SBMT ACK\n"));
                            if (Packet.PacketLength > 1)    {
                                switch (Packet.Data[1]) {
                                case SBMT_CommandReset:
                                	debug("| SpannerThread: > SBMT_CommandReset");
                                    set_phone_flags(Span->simLinkSlot, PF_RESET_NEEDACK, TRUE);

#ifdef FIRST_COMMAND_LOCK
                                    KS_signal(Span->RcvCmdReset);
#endif

                                    break;
                                default:
                                    break;
                                }
                            }
                            break;
                        case SBMT_NegativeAcknowledge:
                        	debug("| SpannerThread: > SBMT Negative ACK");
                            break;
                        default:
                            break;
                        }
                    } else  {
                    	debug("| SpannerThread: > packet length = 0...");
                        Packet.SideBand = 0;
                        TFP_SendPacket(Span->tfp,&Packet);
                        Span->probeToMisPkts++;
                    }
                } else  {
                	debug("| SpannerThread: > Inband packet: %lu", Packet.PacketLength);
                    ddebug2(DBG2_SIM_SERVER, ("!!!!!!!!!!!  InBand Packet: %lu !!!!!!!!\n",Packet.PacketLength));
                    if (Packet.PacketLength)    {
                        BOOL badSW = FALSE;
                        if ((Phones[Span->simLinkSlot]->OpClass!=CLASS_A0_MATCH || (Packet.Data[Packet.DataLength - 2] & 0xf0) != 0x90) &&
                            (Phones[Span->simLinkSlot]->OpClass!=CLASS_00_MATCH || ((Packet.Data[Packet.DataLength - 2] & 0xf0) != 0x90 &&
                             Packet.Data[Packet.DataLength - 2] != 0x61 && Packet.Data[Packet.DataLength - 2] != 0x6c &&
                             (Packet.Data[Packet.DataLength - 2] != 0x6a || Packet.Data[Packet.DataLength - 1] != 0x82) &&
                             (Packet.Data[Packet.DataLength - 2] != 0x63 || (Packet.Data[Packet.DataLength - 1]&0xf0) != 0xc0))))
                            badSW = TRUE;

                        debug("| SpannerThread: is badsw: %i", badSW);
                        switch (Packet.Data[0]) {
                        case SBMT_ResponsePDU:
                        	debug("| SpannerThread: Got Response to Command!");
                            ddebug2(DBG2_SIM_SERVER, (" !!!!!!!!!!!!!!!!!!!!!  Got Response to Command!\n"));

                            if (Span->readingSmsState) {
                                if (Span->readingSmsState==1 && Phones[Span->simLinkSlot]->CurrentOpCode==0xc0 &&
                                    Packet.Data[6]==0x3c && Packet.Data[5]==0x6f) {
                                    Span->lastSmsRcd = Packet.Data[15] ? ((Packet.Data[3]<<8)+Packet.Data[4])/Packet.Data[15] : 1;
                                    Span->readingSmsState = 2;
                                    ddebug2(DBG2_SIM_SIMU, ("Phone %c readingSmsState: 2",
                                            (char)('A' + ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID))));
                                } else if (Span->readingSmsState == 4) {
                                    // ugly hack, but this code used to do this by switch-on-type
                                    //  and direct casts (see below); at least this flexes a bit.
                                    casabyte::shared_ptr<casabyte::RcatsPhone> pPhone =
                                        DeviceFromIndex(ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID), std::nothrow);  // may be NULL
                                    casabyte::mc75::GsmPhone *pGsmPhone = dynamic_cast<casabyte::mc75::GsmPhone *>(pPhone.get());
                                    if (pGsmPhone)
                                    {
                                       pGsmPhone->m_bDoneReadingSimSmsFiles = true;
                                    }
                                    else
                                    {
                                       // old method:
                                       switch (ph_phone_type(ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID)))
                                       {
                                       case TYPE_G20:
                                          g20setDoneReadingSimSmsFiles(ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID));
                                          break;
                                       default:
                                          // do nothing
                                          break;
                                       }
                                    }
                                    Span->readingSmsState = 1;
                                    ddebug2(DBG2_SIM_SIMU, ("Phone %c readingSmsState: from 4 to 1",
                                            (char)('A' + ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID))));
                                }
                            }

                            ProcessRspToPhone(Span->simLinkSlot, Packet.Data, Packet.DataLength);
                            if (getDebug2Flags() & DBG2_SIM_SIMU)
                                debugDump(Packet.Data+1, Packet.DataLength-1, "Rsp:", dsONESPACED);
                            if (Packet.DataLength < 3) {
                                error(eWARN,"MIClient: Span: Phone %c: Illegal/Short Response PDU",
                                        (char)('A' + ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID)));
                            } else if (Packet.Data[Packet.DataLength - 2] == 0x91) {
                                Span->lenProactive = Packet.Data[Packet.DataLength - 1];
                                debug("MIClient: Span: Phone %c: Proactive SIM Command Length %02x",
                                    (char)('A' + ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID)),
                                    (int)Packet.Data[Packet.DataLength - 1]);
                            } else if (badSW)
                                debug("MIClient: Span: Phone %c: Response PDU: Error Status [%02x][%02x]",
                                    (char)('A' + ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID)),
                                    (int)Packet.Data[Packet.DataLength - 2],
                                    (int)Packet.Data[Packet.DataLength - 1]);
                            break;
                        case SBMT_ResponsePDUNotToPhone:
                        	debug("| SpannerThread: SBMT_ResponsePDUNotToPhone!");
                            if (badSW) {
                                Span->initWithCacheError++;
                                outStatus("Phone %c SIM initialization aborted: PDU Error Status %02x%02x",
                                    (char)('A' + ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID)),
                                    (int)Packet.Data[Packet.DataLength - 2],
                                    (int)Packet.Data[Packet.DataLength - 1]);
                            }
                        default:
                            break;
                        }
                    }
                }
            } else if (!TFP_Normal(Span->tfp))  {
            	debug("| SpannerThread: > Span: Phone %c: Lost Connection To Server", (char)('A' + ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID)));
                SOCKET  S = TFP_GetSocket(Span->tfp);

            error(eWARN,"MIClient: Span: Phone %c: Lost Connection To Server",(char)('A' + ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID)));
            // error(eWARN,"TFP State Flags: %04Xh ", Span->tfp->Flags);

                // Notify Daughter Card of disconnect.
                if ((Span->Flags & SPAN_FLAG_INHIBIT) == 0) {
                    //   if we are not inhibited...

                    // 20160912 TL - use new function to get phone index to support AMP
                    fpga_phone_sim_present((char) GetPhoneIndexForSIMOperation(Span->simLinkSlot), FALSE);
                    fpga_map_phone_sim((unsigned char) GetPhoneIndexForSIMOperation(Span->simLinkSlot), 0);
                    SetDeviceNVR_long(ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID), e_local_sim, 0);
                }

                debug("| SpannerThread: end TFP protocol; close socket...");
                TFP_EndProtocol(Span->tfp);
                Span->tfp = NULL;
                t_socketclose(S);
                ddebug2(DBG2_SIM_SERVER, ("!!!!!!!!!!  Span is Dead   !!!!!!!!!!\n"));
                Spanner_SetState(Span,Span_Error);
            }
        }

        debug("| SpannerThread: out of CONNECTED loop...");
        debug("| SpannerThread: ***** Span State: %02X *****\n",Span->State);

        // The re-connect job requires the task to take place in two stages.
        // ShouldConnect doesn't block, so the loop above is woken up
        // Connecting does block so the End Protocol and subsequent connect attempts will
        //   block too.
        Spanner_SetStateIf(Span,Span_Connecting,Span_ShouldConnect);
        if (Span->tfp)  {
            Spanner_SetStateIf(Span,Span_Disconnecting,Span_ShouldDisconnect); // In Case We Need To Block in TFP_EndProtocol()
            {
                SOCKET  S = TFP_GetSocket(Span->tfp);
                outStatus("MIClient: Phone %c Disconnecting from Server",(char)('A' + ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID)));

                debug("| SpannerThread: Phone %c Disconnecting from Server",(char)('A' + ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID)));

                // Notify Daughter Card of disconnect.
                // This "ought to be" redundant with the exact same message
                //   being generated in the main command section.  The
                //   extra message is still interesting in case several connection
                //   or data commands have come and gone due to lag or something.
                if ((Span->Flags & SPAN_FLAG_INHIBIT) == 0) {
                    // 20160912 TL - use new function to get phone index to support AMP
                    fpga_phone_sim_present((char) GetPhoneIndexForSIMOperation(Span->simLinkSlot), FALSE);
                    fpga_map_phone_sim((unsigned char) GetPhoneIndexForSIMOperation(Span->simLinkSlot), 0);
                    SetDeviceNVR_long(ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID), e_local_sim, 0);
                }
                debug("| SpannerThread: end TFP protocol; close socket...");
                TFP_EndProtocol(Span->tfp);
                Span->tfp = NULL;
                t_socketclose(S);
            }
        }
        // This prevents a runaway task for disconnections, other states must be preserved.
        Spanner_SetStateIfAny(Span,Span_Idle,(sizeof(Disconnecting)/sizeof(Disconnecting[0])),Disconnecting);
        debug("| SpannerThread: current state before back to top of FOR loop: %02X",Span->State);
    }
	debug("\\------------------SpannerThread-----------------/");
}

//
// SIMClient_GetTimeSinceLastAccess - returns time since last SIM access via spanner in TICKS, note this function
//    calls rcats_get_raw_time which does disable interrupts for a few instructions, so don't call this function
//    in a busy loop.
//

TICKS SIMClient_GetTimeSinceLastAccess(int Target)
{
    if (Spanner[Target].simLinkSlot==Target) {
        return rcats_get_raw_time() - Spanner[Target].LastAccessTimer;
    } else {
        return -1;
    }
}

char *SIMClient_GetAssignedServerName(int Target, char *buf, int buf_length)
{
    if (Spanner[Target].simLinkSlot==Target) {
        snprintf(buf, buf_length, "%s/%s", Spanner[Target].LastServer, Spanner[Target].SIMName);
        return buf;
    } else {
        return NULL;
    }
}

/*
 * Procedure: SIMClient_GetContentOffset
 *  Routine to get file content offset from cache, hiding version based cache structure.
 *  Note the offset increments in SimCacheDirRecord units for directories, byte offset for data files only.
 * Inputs:	pCache		- Pointer to cache header.
 *			pDirRec		- Pointer to directory record
 * Returns offset.
*/
unsigned long SIMClient_GetContentOffset(unsigned char *pCache, unsigned char *pDirRec)
{
	if ((pCache[SCH_DIR_RECORD_SIZE]) == SIM_CACHE_DIR_RECORD_V01_SIZE)
		return( (unsigned long)pDirRec[SCDR_CONTENT_OFFSET_LSB] |
				(unsigned long)pDirRec[SCDR_CONTENT_OFFSET_MSB] << 8 );
	else
		return( (unsigned long)pDirRec[SCDR_CONTENT_OFFSET_LSB] |
				(unsigned long)pDirRec[SCDR_CONTENT_OFFSET_MSB] << 8 |
				(unsigned long)pDirRec[SCDR_CONTENT_OFFSET_HI_LSB] << 16 |
				(unsigned long)pDirRec[SCDR_CONTENT_OFFSET_HI_MSB] << 24 );
}

/*
 * Procedure: SIMClient_GetStatusOffset
 *  Routine to get file status offset from cache, hiding version based cache structure.
 * Inputs:	pCache		- Pointer to cache header.
 *			pDirRec		- Pointer to directory record
 * Returns offset.
*/
unsigned long SIMClient_GetStatusOffset(unsigned char *pCache, unsigned char *pDirRec)
{
	if ((pCache[SCH_DIR_RECORD_SIZE]) == SIM_CACHE_DIR_RECORD_V01_SIZE)
		return( (unsigned long)pDirRec[SCDR_STATUS_OFFSET_LSB] |
				(unsigned long)pDirRec[SCDR_STATUS_OFFSET_MSB] << 8 );
	else
		return( (unsigned long)pDirRec[SCDR_STATUS_OFFSET_LSB] |
				(unsigned long)pDirRec[SCDR_STATUS_OFFSET_MSB] << 8 |
				(unsigned long)pDirRec[SCDR_STATUS_OFFSET_HI_LSB] << 16 |
				(unsigned long)pDirRec[SCDR_STATUS_OFFSET_HI_MSB] << 24 );
}
/*
 * Procedure: SIMClient_SetContentOffset
 *  Routine to set file content offset in cache, hiding version based cache structure.
 * Inputs:	pCache		- Pointer to cache header.
 *			pDirRec		- Pointer to directory record
 *			offset		- content offset
*/
void SIMClient_SetContentOffset(unsigned char *pCache, unsigned char *pDirRec, unsigned long offset)
{
	if ((pCache[SCH_DIR_RECORD_SIZE]) == SIM_CACHE_DIR_RECORD_V01_SIZE)	{
		pDirRec[SCDR_CONTENT_OFFSET_LSB] = LOW_BYTE_OF_16BITS((unsigned short)offset);
		pDirRec[SCDR_CONTENT_OFFSET_MSB] = HIGH_BYTE_OF_16BITS((unsigned short)offset);
	}
	else {
		pDirRec[SCDR_CONTENT_OFFSET_LSB] = BYTE0_OF_DWORD(offset);
		pDirRec[SCDR_CONTENT_OFFSET_MSB] = BYTE1_OF_DWORD(offset);
		pDirRec[SCDR_CONTENT_OFFSET_HI_LSB] = BYTE2_OF_DWORD(offset);
		pDirRec[SCDR_CONTENT_OFFSET_HI_MSB] = BYTE3_OF_DWORD(offset);
	}
}
/*
 * Procedure: SIMClient_SetStatusOffset
 *  Routine to set file status offset in cache, hiding version based cache structure.
 * Inputs:	pCache		- Pointer to cache header.
 *			pDirRec		- Pointer to directory record
 *			offset		- content offset
*/
void SIMClient_SetStatusOffset(unsigned char *pCache, unsigned char *pDirRec, unsigned long offset)
{
	if ((pCache[SCH_DIR_RECORD_SIZE]) == SIM_CACHE_DIR_RECORD_V01_SIZE)	{
		pDirRec[SCDR_STATUS_OFFSET_LSB] = LOW_BYTE_OF_16BITS((unsigned short)offset);
		pDirRec[SCDR_STATUS_OFFSET_MSB] = HIGH_BYTE_OF_16BITS((unsigned short)offset);
	}
	else {
		pDirRec[SCDR_STATUS_OFFSET_LSB] = BYTE0_OF_DWORD(offset);
		pDirRec[SCDR_STATUS_OFFSET_MSB] = BYTE1_OF_DWORD(offset);
		pDirRec[SCDR_STATUS_OFFSET_HI_LSB] = BYTE2_OF_DWORD(offset);
		pDirRec[SCDR_STATUS_OFFSET_HI_MSB] = BYTE3_OF_DWORD(offset);
	}
}

#define CMD_SELECT  0xA4
#define CMD_STATUS  0xF2
#define CMD_READBIN 0xB0
#define CMD_UPDBIN  0xD6
#define CMD_READREC 0xB2
#define CMD_UPDREC  0xDC
#define CMD_CALCKC  0x88
#define CMD_GETRSP  0xC0
#define CMD_SEARCH  0xA2
#define CMD_FETCH   0x12
#define CMD_VERIFYPIN   0x20
#define CMD_UNBLKPIN    0x2C
#define CMD_ENVELOPE    0xC2

#define RDRECMODE_NEXT  2
#define RDRECMODE_PREV  3
#define RDRECMODE_ABCU  4   // Absolute or Current

void deletePar(ParDir * parDirPtr)
{
    if (parDirPtr->ParDirPtr) {
        deletePar(parDirPtr->ParDirPtr);
        farfree(parDirPtr->ParDirPtr);
        parDirPtr->ParDirPtr = 0;
    }
}

void invalidateCache(Spanner_Client * Span)
{
    if (Span->cacheImage) {
        farfree(Span->cacheImage);
    }
    Span->cacheImage = 0;
    if (Span->parDirPtr) {
        deletePar(Span->parDirPtr);
        farfree(Span->parDirPtr);
    }
    Span->parDirPtr = 0;
    if (Span->aidPtr)
        free(Span->aidPtr);
    Span->aidPtr = 0;
}

void reinitializeCache(int simLinkSlot)
{
    Spanner_Client *        Span = &Spanner[simLinkSlot];
    int simId;
    ResultCode result = sim_get_id2(simLinkSlot, &simId);
    if (result.fail())
    {
       error(eFATAL, "unable to get SIM ID from physical slot");
       return;
    }
    if (IS_REMOTE_SIM(simId))
    {
        if (!Span->cacheImage) {
            outStatus("Not Using SIM Cache");
            return;
        }
        if (Span->cwd)
            debug("Phone %c previously used SIM cache reinitialized",(char)('A' + ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID)));
        Span->cwd = Span->cacheImage + *(Span->cacheImage+SCH_DIR_AREA_OFFSET);
        if (Span->parDirPtr) {
            deletePar(Span->parDirPtr);
            farfree(Span->parDirPtr);
        }
        Span->parDirPtr = 0;
        Span->cwf = 0;
        Span->curRec = 0;
        Span->prevCmd = 0;
        Span->resetCnt = 0;
        Span->misToProbeAcks = 0;
        Span->probeToMisPkts = 0;
        Span->probeToMisPktsAtLastReset = 0;
        Span->initWithCacheError = 0;
        Span->curEntryState = CurEntry_IsInCache;
        Span->lenProactive = 0;
#if defined(SUP_N6650) || defined(SUP_G20) || defined(SUP_MC75)
        if (Span->readingSmsState) {
            Span->readingSmsState = 1;
            ddebug2(DBG2_SIM_SIMU, ("Phone %c readingSmsState: 1",(char)('A' + ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID))));
        }
#endif
    }
}

void newDirInit(Spanner_Client * Span, unsigned char * curEntry)
{
    ParDir *tmp = (ParDir*)farmalloc(sizeof(ParDir));
    if (tmp) {
        tmp->Parent = Span->cwd;
        tmp->ParDirPtr = Span->parDirPtr ? Span->parDirPtr : 0;
        Span->parDirPtr = tmp;
        Span->cwd = curEntry;
        Span->cwf = 0;
        Span->curRec = 0;
        Span->contentPtrEFarr = 0;
    }
}

unsigned char * findInDir(int fileId, unsigned char * dirEntry, int dirSz, unsigned char * cHdr)
{
    int     numEntries = LOWHIGHBYTE_TO_16BITS(*(dirEntry+SCDR_CONTENT_LENGTH_LSB),*(dirEntry+SCDR_CONTENT_LENGTH_MSB));
    // ddebug2(DBG2_SIM_SIMU, ("findInDir: numEntries %d", numEntries));
    if (numEntries) {
        unsigned char * curEntry = dirEntry + (dirSz * SIMClient_GetContentOffset(cHdr, dirEntry));
        int         count;
        for (count = 0; count < numEntries; count++) {
            // ddebug2(DBG2_SIM_SIMU, ("findInDir: fileID %x", LOWHIGHBYTE_TO_16BITS(*(curEntry+SCDR_FILE_ID_LSB),*(curEntry+SCDR_FILE_ID_MSB))));
            if (LOWHIGHBYTE_TO_16BITS(*(curEntry+SCDR_FILE_ID_LSB),*(curEntry+SCDR_FILE_ID_MSB)) == fileId)
                return curEntry;
            curEntry += dirSz;
        }
    }
    return 0;
}

IsRspFromCmdToMIS SIMSimulator(Spanner_Client * Span, unsigned char * inputCmdPdu, unsigned char * simResponse, int * rspSizePtr)
{
	debug("/---------------SIMSimulator--------------------\\");
    IsRspFromCmdToMIS       isRspFromCmdToMIS;
    BOOL                    isUMTS;
    unsigned char *         header;
    unsigned char *         dataArea;
    int                     cmd;

    // debug("SIMSimulator phone %d", Span->PhoneID);
    if (!Span->cacheImage) {
    	debug("| SIMSimulator: no cache image!!!");
        if (getDebug2Flags() & DBG2_SIM_SIMU)
            debugDump(inputCmdPdu, getCmdLen(inputCmdPdu), "Cmd to MIS:", dsONESPACED);
        debug("\\---------------SIMSimulator--------------------/");
        return rspFromMIS;
    }

    if (Span->initWithCacheError) {
    	debug("| SIMSimulator: initWithCacheError!!!");
        *rspSizePtr = 3;
        simResponse[1] = 0x6F;
        simResponse[2] = 0;
        debug("\\---------------SIMSimulator--------------------/");
        return rspSuppliedCmdToMIS;
    }

    isRspFromCmdToMIS = rspSuppliedCmdToMIS;
    isUMTS = (Phones[Span->simLinkSlot]->OpClass == CLASS_00_MATCH);
    header = Span->cacheImage;
    dataArea = Span->cacheImage + LOWHIGHBYTE_TO_16BITS(*(header+SCH_DATA_AREA_OFFSET_LSB),*(header+SCH_DATA_AREA_OFFSET_MSB));
    cmd = inputCmdPdu[TC_OpCode];
    debug("| SIMSimulator: cmd = %i", cmd);
    switch (cmd) {
    case CMD_SELECT:    {
    	debug("| SIMSimulator: CMD_SELECT");
        unsigned char *     curEntry = NULL;
        unsigned int        fileId = 0;
        int         P1 = inputCmdPdu[TC_P1];
        CurEntry                previousEntryState = Span->curEntryState;
        Span->curEntryState = CurEntry_IsInCache;
        if (!(P1&0xC)) {                                    // not by AID and not by path, so by fileId
            fileId = (inputCmdPdu[5] << 8) + inputCmdPdu[6];
            if (!P1) {                                            // DF, EF or MF by fileId
                /*
                 * In addition to finding the file or dir in the cwd, SIM search rules allow
                 * finding a directory (i.e., changing directory) in several other odd places
                 */
                if (fileId == 0x3F00 || !inputCmdPdu[TC_Length]) {                          // MD
                    Span->cwd = curEntry = Span->cacheImage + *(header+SCH_DIR_AREA_OFFSET);
                    if (Span->parDirPtr) {
                        deletePar(Span->parDirPtr);
                        free(Span->parDirPtr);
                    }
                    Span->parDirPtr = 0;
                    Span->cwf = 0;
                    Span->curRec = 0;
                    Span->contentPtrEFarr = 0;
                } else if (fileId == (unsigned int)LOWHIGHBYTE_TO_16BITS(*(Span->cwd+SCDR_FILE_ID_LSB),*(Span->cwd+SCDR_FILE_ID_MSB))) { // cwd
                    curEntry = Span->cwd;
                    Span->cwf = 0;
                    Span->curRec = 0;
                    Span->contentPtrEFarr = 0;
                } else if (Span->parDirPtr && fileId == (unsigned int)LOWHIGHBYTE_TO_16BITS(*(Span->parDirPtr->Parent+SCDR_FILE_ID_LSB), // cwdPar
                                                  *(Span->parDirPtr->Parent+SCDR_FILE_ID_LSB))) {
                    Span->cwd = curEntry = Span->parDirPtr->Parent;
                    if (Span->parDirPtr->ParDirPtr) {
                        ParDir *tmp = Span->parDirPtr;
                        Span->parDirPtr = Span->parDirPtr->ParDirPtr;
                        free(tmp);
                    } else {
                        farfree(Span->parDirPtr);
                        Span->parDirPtr = 0;
                    }
                    Span->cwf = 0;
                    Span->curRec = 0;
                    Span->contentPtrEFarr = 0;
                } else if ((curEntry = findInDir(fileId, Span->cwd, *(header+SCH_DIR_RECORD_SIZE), header)) != 0) {     // in cwd
                                        if (*(curEntry+SCDR_FLAGS)&STATUS_VALID && !(*(curEntry+SCDR_STATUS_LENGTH))) {
                                                *rspSizePtr = 3;
                                                if (isUMTS) {
                                                        simResponse[1] = 0x6A;
                                                        simResponse[2] = 0x82;
                                                } else {
                                                        simResponse[1] = 0x94;
                                                        simResponse[2] = 0x04;
                                                }
                                                break;
                                        }
                    if (*(curEntry+SCDR_FLAGS)&IS_DIR)
                        newDirInit(Span, curEntry);
                    else
                        Span->cwf = curEntry;
                } else if (Span->parDirPtr && (curEntry = findInDir(fileId, Span->parDirPtr->Parent, *(header+SCH_DIR_RECORD_SIZE), header))  // dir in cwdPar
                        && (*(curEntry+SCDR_FLAGS)&IS_DIR)) {
                                        if (*(curEntry+SCDR_FLAGS)&STATUS_VALID && !(*(curEntry+SCDR_STATUS_LENGTH))) {
                                                *rspSizePtr = 3;
                                                if (isUMTS) {
                                                        simResponse[1] = 0x6A;
                                                        simResponse[2] = 0x82;
                                                } else {
                                                        simResponse[1] = 0x94;
                                                        simResponse[2] = 0x04;
                                                }
                                                break;
                                        }
                    Span->cwd = curEntry;
                    Span->cwf = 0;
                    Span->curRec = 0;
                    Span->contentPtrEFarr = 0;
                                } else if ((curEntry=findInDir(fileId,Span->cacheImage+*(header+SCH_DIR_AREA_OFFSET),*(header+SCH_DIR_RECORD_SIZE), header))!=0) { // in MD
                                        if (*(curEntry+SCDR_FLAGS)&STATUS_VALID && !(*(curEntry+SCDR_STATUS_LENGTH))) {
                                                *rspSizePtr = 3;
                                                if (isUMTS) {
                                                        simResponse[1] = 0x6A;
                                                        simResponse[2] = 0x82;
                                                } else {
                                                        simResponse[1] = 0x94;
                                                        simResponse[2] = 0x04;
                                                }
                                                break;
                                        }
                                        if (Span->parDirPtr) {
                                                deletePar(Span->parDirPtr);
                                                free(Span->parDirPtr);
                        Span->parDirPtr = 0;
                                        }
                                        Span->cwd = Span->cacheImage + *(header+SCH_DIR_AREA_OFFSET);
                                        if (*(curEntry+SCDR_FLAGS)&IS_DIR)
                                                newDirInit(Span, curEntry);
                                        else
                                                Span->cwf = curEntry;
                                        Span->curRec = 0;
                } else
                    isRspFromCmdToMIS = rspFromMIS;
            } else if (P1 == 0x1) {                                             // in cwd
                if ((curEntry = findInDir(fileId, Span->cwd, *(header+SCH_DIR_RECORD_SIZE), header)) != 0) {
                                        if (*(curEntry+SCDR_FLAGS)&STATUS_VALID && !(*(curEntry+SCDR_STATUS_LENGTH))) {
                                                *rspSizePtr = 3;
                                                if (isUMTS) {
                                                        simResponse[1] = 0x6A;
                                                        simResponse[2] = 0x82;
                                                } else {
                                                        simResponse[1] = 0x94;
                                                        simResponse[2] = 0x04;
                                                }
                                                break;
                                        }
                    if (*(curEntry+SCDR_FLAGS)&IS_DIR)
                        newDirInit(Span, curEntry);
                    else
                        Span->cwf = curEntry;
                } else
                    isRspFromCmdToMIS = rspFromMIS;
            } else if (P1 == 0x3) {                                             // cwdPar
                if (Span->parDirPtr) {
                    Span->cwd = curEntry = Span->parDirPtr->Parent;
                    if (Span->parDirPtr->ParDirPtr) {
                        ParDir *tmp = Span->parDirPtr;
                        Span->parDirPtr = Span->parDirPtr->ParDirPtr;
                        free(tmp);
                    } else {
                        free(Span->parDirPtr);
                        Span->parDirPtr = 0;
                    }
                    Span->cwf = 0;
                    Span->curRec = 0;
                    Span->contentPtrEFarr = 0;
                } else
                    isRspFromCmdToMIS = rspFromMIS;
            } else
                isRspFromCmdToMIS = rspFromMIS;
        } else if (P1 == 0x8) {                                                 // path from MF
            int  pathIdx;
            BOOL doesNotExist = FALSE;
            BOOL specialCase = FALSE;
            BOOL specialCase2 = FALSE;

            casabyte::shared_ptr<casabyte::RcatsPhone> pPhone = DeviceFromIndex(ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID), std::nothrow);      // may be NULL
            casabyte::IRemoteSimUser *pSimUsingPhone = dynamic_cast<casabyte::IRemoteSimUser *>(pPhone.get());       // may be NULL (whether pPhone was or not)
			if (pSimUsingPhone)
			{
				specialCase = pSimUsingPhone->Allow7FxxSubDirectories();
				specialCase2 = pSimUsingPhone->Allow6F16ReadWithoutPermission();
			}

            Span->cwd = Span->cacheImage + *(header+SCH_DIR_AREA_OFFSET);
            if (Span->parDirPtr) {
                deletePar(Span->parDirPtr);
                free(Span->parDirPtr);
            }
            Span->parDirPtr = 0;
            Span->cwf = 0;
            Span->curRec = 0;
            Span->contentPtrEFarr = 0;
            for (pathIdx = 0; pathIdx < inputCmdPdu[TC_Length]; pathIdx += 2) {
                fileId = (inputCmdPdu[5+pathIdx] << 8) + inputCmdPdu[6+pathIdx];
                if (specialCase && fileId==0x7f66 && *(Span->cwd+SCDR_FILE_ID_MSB)==0x7f && *(Span->cwd+SCDR_FILE_ID_LSB)==0xff)
                    // Special case for certain phones that considers 7f66 in the 7fff directory even though a 7fxx directory
                    // must be a subdirectory of the MF (3f00), but the MC8775V needs a SW 0x6A82 returned in this case
                    Span->cwd = Span->cacheImage + *(header+SCH_DIR_AREA_OFFSET);
                else if (specialCase && fileId==0x7f40 && *(Span->cwd+SCDR_FILE_ID_MSB)==0x7f && *(Span->cwd+SCDR_FILE_ID_LSB)==0xff)
                    // Special case for certain phones that considers 7f40 in the 7fff directory even though a 7fxx directory
                    // must be a subdirectory of the MF (3f00).
                    Span->cwd = Span->cacheImage + *(header+SCH_DIR_AREA_OFFSET);
                else if (specialCase2 && fileId==0x6f16 && *(Span->cwd+SCDR_FILE_ID_MSB)==0x7f && *(Span->cwd+SCDR_FILE_ID_LSB)==0x20 &&
                    (curEntry = findInDir(0x6f06, Span->cwd, *(header+SCH_DIR_RECORD_SIZE), header)) != 0 &&
                    *(curEntry+SCDR_FLAGS)&STATUS_VALID && *(curEntry+SCDR_STATUS_LENGTH) && *(curEntry+SCDR_FLAGS)&CONTENTS_VALID &&
                    (*(curEntry+SCDR_CONTENT_LENGTH_LSB) || *(curEntry+SCDR_CONTENT_LENGTH_MSB))) {
                    // Special case for MC8775V that asks for the 7f20/6f16 file even though it does not have permission to
                    // read it.  Need the 7f20/6f06 (arr EF) file to determine the permission, so can avoid going to MI Server,
                    // which happens at the time the AT+CIMI is sent to the phone, but the phone won't answer until get the
                    // response from the SIM, which requires a SIM cache response to avoid AT+CIMI timeout.
                    unsigned char *    cPtr = dataArea + SIMClient_GetStatusOffset(header, curEntry);
                    if (*cPtr++ == 0x62) {
                        int    lengthFCP = *cPtr++;
                        for (int current = 0; current < lengthFCP; ) {
                            switch (*cPtr) {
                            case 0x82:
                                Span->recLenEFarr = *(cPtr+5);
                                Span->numRecEFarr = *(cPtr+6);
                                break;
                            case 0x80:
                            case 0x81:
                            case 0x83:
                            case 0x84:
                            case 0x88:
                            case 0x8A:
                            case 0x8B:
                            case 0x8C:
                            case 0xA5:
                            case 0xAB:
                            case 0xC6:
                                break;
                            default:
                                ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Bad tag for ARR %x", *cPtr));
                                break;
                            }
                            current += *(cPtr+1) + 2;
                            cPtr += *(cPtr+1) + 2;
                        }
                        if (Span->recLenEFarr && Span->numRecEFarr)
                            Span->contentPtrEFarr = dataArea + SIMClient_GetContentOffset(header, curEntry);
                        else
                            ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Zero ARR rec len"));
                    } else
                        ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Bad initial tag for ARR %x", *(cPtr-1)));
                }
                if ((curEntry = findInDir(fileId, Span->cwd, *(header+SCH_DIR_RECORD_SIZE), header)) != 0) {
                    if (*(curEntry+SCDR_FLAGS)&STATUS_VALID && !(*(curEntry+SCDR_STATUS_LENGTH))) {
                        *rspSizePtr = 3;
                        if (isUMTS) {
                                simResponse[1] = 0x6A;
                                simResponse[2] = 0x82;
                        } else {
                                simResponse[1] = 0x94;
                                simResponse[2] = 0x04;
                        }
                        doesNotExist = TRUE;
                        break;
                    }
                    if (*(curEntry+SCDR_FLAGS)&IS_DIR)
                        newDirInit(Span, curEntry);
                    else
                        Span->cwf = curEntry;
                } else
                    isRspFromCmdToMIS = rspFromMIS;
            }
            if (doesNotExist)
                break;
        } else if (P1 == 0x9) {                                                 // path from current DF
            int     pathIdx;
            BOOL            doesNotExist = FALSE;
            for (pathIdx = 0; pathIdx < inputCmdPdu[TC_Length]; pathIdx += 2) {
                fileId = (inputCmdPdu[5+pathIdx] << 8) + inputCmdPdu[6+pathIdx];
                if ((curEntry = findInDir(fileId, Span->cwd, *(header+SCH_DIR_RECORD_SIZE), header)) != 0) {
                    if (*(curEntry+SCDR_FLAGS)&STATUS_VALID && !(*(curEntry+SCDR_STATUS_LENGTH))) {
                            *rspSizePtr = 3;
                            if (isUMTS) {
                                    simResponse[1] = 0x6A;
                                    simResponse[2] = 0x82;
                            } else {
                                    simResponse[1] = 0x94;
                                    simResponse[2] = 0x04;
                            }
                            doesNotExist = TRUE;
                            break;
                    }
                    if (*(curEntry+SCDR_FLAGS)&IS_DIR)
                        newDirInit(Span, curEntry);
                    else
                        Span->cwf = curEntry;
                } else
                    isRspFromCmdToMIS = rspFromMIS;
            }
            if (doesNotExist)
                break;
        } else if (P1 == 0x4 && (curEntry = findInDir(0x7fff,Span->cacheImage+*(header+SCH_DIR_AREA_OFFSET),*(header+SCH_DIR_RECORD_SIZE), header)) != 0) { // AID (Application ID)
            unsigned char * cPtr;
            int lengthFCP;
            int current;
            int aidSize = 0;
            if (!(*(curEntry+SCDR_FLAGS) & STATUS_VALID)) {
                ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Flags for AID entry not valid"));
                isRspFromCmdToMIS = rspFromMIS;
                break;
            }
            if (!(*(curEntry+SCDR_STATUS_LENGTH))) {
                *rspSizePtr = 3;
                if (isUMTS) {
                    simResponse[1] = 0x6A;
                    simResponse[2] = 0x82;
                } else {
                    simResponse[1] = 0x94;
                    simResponse[2] = 0x04;
                }
                break;
            }
            if (Span->parDirPtr) {
                deletePar(Span->parDirPtr);
                free(Span->parDirPtr);
                Span->parDirPtr = 0;
            }
            Span->cwd = Span->cacheImage + *(header+SCH_DIR_AREA_OFFSET);
            cPtr = dataArea + SIMClient_GetStatusOffset(header, curEntry);
            if (*cPtr++ != 0x62) {
                ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Bad initial tag for AID %x", *(cPtr-1)));
                isRspFromCmdToMIS = rspFromMIS;
                break;
            }
            lengthFCP = *cPtr++;
            for (current = 0; current < lengthFCP; ) {
                switch (*cPtr) {
                case 0x84:
                    if (aidSize) {
                        ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: More than one AID found"));
                    } else {
                        aidSize = *(cPtr+1) + 2;
                        Span->aidPtr = (unsigned char *)farmalloc(aidSize);
                        memcpy(Span->aidPtr, cPtr, aidSize);
                    }
                    break;
                case 0x80:
                case 0x81:
                case 0x82:
                case 0x83:
                case 0x88:
                case 0x8A:
                case 0x8B:
                case 0x8C:
                case 0xA5:
                case 0xAB:
                case 0xC6:
                    break;
                default:
                    ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Bad tag for AID %x", *cPtr));
                    isRspFromCmdToMIS = rspFromMIS;
                    break;
                }
                current += *(cPtr+1) + 2;
                cPtr += *(cPtr+1) + 2;
            }
            if (isRspFromCmdToMIS == rspFromMIS)
                break;
            if (!aidSize) {
                ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: AID not found"));
                isRspFromCmdToMIS = rspFromMIS;
                break;
            }
            newDirInit(Span, curEntry);
        } else {
            Span->curEntryState = CurEntry_IsDirNotInCache;
            ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Directory 7fff not in cache or unknown P1 parameter %x", P1));
            isRspFromCmdToMIS = rspFromMIS;
            break;
        }

        if (isRspFromCmdToMIS != rspFromMIS) {
            *rspSizePtr = 3;
            if (isUMTS && (inputCmdPdu[TC_P2] == 0xC)) {                    // No data returned
                simResponse[1] = 0x90;
                simResponse[2] = 0x00;
            } else  {
                simResponse[1] = isUMTS ? 0x61 : 0x9F;
                simResponse[2] = *(curEntry+SCDR_STATUS_LENGTH);
            }
        } else {
            unsigned int    firstByteFileId = fileId & 0xFF00;
            if (firstByteFileId == 0x7F00 || firstByteFileId == 0x5F00 || previousEntryState == CurEntry_IsDirNotInCache) {
                Span->curEntryState = CurEntry_IsDirNotInCache;
                ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Directory/File %x not in cache", fileId));
            } else {
                Span->curEntryState = CurEntry_IsFileNotInCache;
                // RCATSP-1867: Files in SIM cache may be falsely invalidated.
                // Since file isn't cached, clear pointer to current working file.
                Span->cwf = 0;
                ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: File %x not in cache", fileId));
            }
        }
        }
        break;
    case CMD_CALCKC:
    	debug("| SIMSimulator: CMD_CALCKC");

        ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Cmd 88 not supported")); // May fail if auth cnt exceeded; for UMTS, GETRSP size can be 0xE, 0x10 or 0x35
        isRspFromCmdToMIS = rspFromMIS;
        break;
    case CMD_GETRSP:
    	debug("| SIMSimulator: CMD_GETRSP");
        // RCATSP-1861: Rogers LTE SIM on MC7700 fails; Envelope command response indicated more data, so get from SIM.
        if (Span->prevCmd == CMD_CALCKC || Span->prevCmd == CMD_ENVELOPE || Span->curEntryState != CurEntry_IsInCache) {
            ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Get response for file not in cache or for CMDs CALCKC/ENVELOPE"));
            isRspFromCmdToMIS = rspFromMIS;
            break;
        }
                if (Span->prevCmd == CMD_VERIFYPIN) {
                        if (Span->curEntryState == CurEntry_IsInCache) {
                                *rspSizePtr = 3;
                                simResponse[1] = 0x6F;          // Negative acknowledgement: i.e. no PIN
                                simResponse[2] = 0;
                        } else {
                ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Get response for CMD VERIFYPIN"));
                                isRspFromCmdToMIS = rspFromMIS;
                        }
                        break;
                }
        if (Span->prevCmd == CMD_SEARCH) {
            ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Get response for CMD SEARCH"));
            isRspFromCmdToMIS = rspFromMIS;
            break;
        }
        // fall thru to CMD_STATUS case
    case CMD_STATUS:    {
    	debug("| SIMSimulator: CMD_STATUS");
        unsigned char * curEntry;
        int             len = inputCmdPdu[TC_Length];
        int             P2 = inputCmdPdu[TC_P2];
        if (cmd == CMD_STATUS) {
            if (P2 == 0x1) {                    // current application
                if (!Span->aidPtr) {
                    ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: AID not set"));
                    isRspFromCmdToMIS = rspFromMIS;
                    break;
                }
                if (len == 0xFF) {              // return AID length
                    *rspSizePtr = 3;
                    simResponse[1] = 0x6C;
                    simResponse[2] = *(Span->aidPtr + 1) + 2;
                    break;
                } else if (len == *(Span->aidPtr + 1) + 2) {    // return AID
                    *rspSizePtr = len + 3;
                    memcpy (&simResponse[1], Span->aidPtr, len);
                    simResponse[*rspSizePtr - 2] = Span->lenProactive ? 0x91 : 0x90;
                    simResponse[*rspSizePtr - 1] = Span->lenProactive;
                    break;
                }
                ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: AID length %x does not match requested %x", *(Span->aidPtr + 1) + 2, len));
                isRspFromCmdToMIS = rspFromMIS;
                break;
            }
            /*
             * To solve Issue 2480 (SMS trickle down one every 15 min) don't invalidate cache, so that cache can be reused upon
             * killcall/restore or recycling phone power.  Alternative would be to download a new cache, but that would require
             * a new MI Server version.  Should be no harm in keeping cache active, since files are invalidated upon write.
             *
            if(Span->prevCmd == CMD_STATUS && Span->cacheImage) {
                invalidateCache (Span);
                ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: 2 STATUSs invalidate cache"));
                isRspFromCmdToMIS = rspFromMIS;
                break;
            }
             *
             */
            if (! Span->simReadComplete) {
                casabyte::shared_ptr<casabyte::RcatsPhone> pPhone =
                    DeviceFromIndex(ph_ConvertCellPhoneToPhoneIndex(Span->PhoneID), std::nothrow);  // may be NULL
                casabyte::mc75::GsmPhone *pGsmPhone = dynamic_cast<casabyte::mc75::GsmPhone *>(pPhone.get());
                if (!pGsmPhone || pGsmPhone->GetUserSimStatus() == casabyte::mc75::SIM_STATUS_READY)
                    Span->simReadComplete = TRUE;
                //RCATSP-1864 - MC7700 needs to get status responses from the SIM
                casabyte::IRemoteSimUser *pSimUsingPhone = dynamic_cast<casabyte::IRemoteSimUser *>(pPhone.get());       // may be NULL (whether pPhone was or not)
                if (pSimUsingPhone && pSimUsingPhone->AlwaysGetSIMStatusFromServer())
                    isRspFromCmdToMIS = rspFromMIS;
            }
            if (Span->simReadComplete) {
                ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: STATUS cmds after SIM read complete sent to MIS so SIM can request proactive cmds"));
                isRspFromCmdToMIS = rspFromMIS;
                break;
            }
            if (P2 == 0xC) {                    // no data returned
                *rspSizePtr = 3;
                simResponse[1] = Span->lenProactive ? 0x91 : 0x90;
                simResponse[2] = Span->lenProactive;
                break;
            }
            if (Span->curEntryState == CurEntry_IsDirNotInCache) {
                ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Status cmd when directory not in cache"));
                isRspFromCmdToMIS = rspFromMIS;
                break;
            }
            if (len == 0xFF) {      // return length (only known use is STATUS cmd that the phone sends after ATD (dial))
                *rspSizePtr = 3;
                simResponse[1] = 0x6C;
                simResponse[2] = *(Span->cwd + SCDR_STATUS_LENGTH);
                break;
            }
            curEntry = Span->cwd;
        } else
            curEntry = Span->cwf ? Span->cwf : Span->cwd;
        if (!(*(curEntry+SCDR_FLAGS) & STATUS_VALID)) {
            ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Flags for current entry not valid"));
            isRspFromCmdToMIS = rspFromMIS;
            break;
        }
        if (Span->contentPtrEFarr) {
            Span->accessRuleRecNum = 0;
            unsigned char *    cPtr = dataArea + SIMClient_GetStatusOffset(header, curEntry);
            if (*cPtr++ == 0x62) {
                int    lengthFCP = *cPtr++;
                for (int current = 0; current < lengthFCP; ) {
                    switch (*cPtr) {
                    case 0x8B:
                        if (*(cPtr+1) == 3 && *(cPtr+2) == 0x6f && *(cPtr+3) == 0x06)
                            Span->accessRuleRecNum = *(cPtr+4);
                        break;
                    case 0x80:
                    case 0x81:
                    case 0x82:
                    case 0x83:
                    case 0x84:
                    case 0x88:
                    case 0x8A:
                    case 0x8C:
                    case 0xA5:
                    case 0xAB:
                    case 0xC6:
                        break;
                    default:
                        ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Bad tag for 6F16 %x", *cPtr));
                        break;
                    }
                    current += *(cPtr+1) + 2;
                    cPtr += *(cPtr+1) + 2;
                }
                if (!Span->accessRuleRecNum)
                    ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Zero 6F16 rec num"));
            } else
                ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Bad initial tag for 6F16 %x", *(cPtr-1)));
        }
        *rspSizePtr = len + 3;
        memcpy (&simResponse[1], dataArea + SIMClient_GetStatusOffset(header, curEntry), len);
        simResponse[*rspSizePtr - 2] = Span->lenProactive ? 0x91 : 0x90;
        simResponse[*rspSizePtr - 1] = Span->lenProactive;
        }
        break;
    case CMD_READBIN:   {
    	debug("| SIMSimulator: CMD_READBIN");
        int         len;
        isRspFromCmdToMIS = rspSuppliedNotToMIS;
        if (Span->curEntryState != CurEntry_IsInCache) {
            ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: READBIN for file not in Cache"));
            isRspFromCmdToMIS = rspFromMIS;
            break;
        }
        if (!Span->cwf) {
            ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: READBIN no Cache CWF"));
            /** Cache can be wrong (i.e. 7f66 dir), so don't respond with error.
            *rspSizePtr = 3;
            if (isUMTS) {
                simResponse[1] = 0x69;
                simResponse[2] = 0x86;
            } else {
                simResponse[1] = 0x94;
                simResponse[2] = 0;
            }
            **/
            // No working file, so forward to MIS.
            isRspFromCmdToMIS = rspFromMIS;
            break;
        }
        if (Span->contentPtrEFarr && Span->accessRuleRecNum > 0 && Span->accessRuleRecNum <= Span->numRecEFarr) {
            unsigned char * cPtr = Span->contentPtrEFarr + Span->recLenEFarr*(Span->accessRuleRecNum - 1);
            if (*cPtr == 0x80 && *(cPtr+1) == 1 && *(cPtr+2)&0x1 && *(cPtr+3) == 0x97) {    // Read of contents never allowed
                *rspSizePtr = 3;
                simResponse[1] = 0x69;
                simResponse[2] = 0x82;
                break;
            }
        }
        if (!(*(Span->cwf+SCDR_FLAGS) & CONTENTS_VALID)) {
        	//NOTE: Is this a new device that needs a specialCase in the CMD_SELECT handler?
            ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: READBIN contents not valid"));
            isRspFromCmdToMIS = rspFromMIS;
            break;
        }
        if (isUMTS && (inputCmdPdu[TC_P1]&0x80)) {
            ddebug2(DBG2_SIM_SIMU,("SIMSimulator: READBIN P1 %x requests Short File Identifier referencing",inputCmdPdu[TC_P1]));
            isRspFromCmdToMIS = rspFromMIS;
            break;
        }
        // *(Span->cwf+SCDR_FLAGS) &= ~CONTENTS_VALID; removed so can reuse cache upon reinit - doing this originally overly conservative
        len = inputCmdPdu[TC_Length];
        if (!len)
            len = 256;
        *rspSizePtr = len + 3;
        int offset = ((inputCmdPdu[TC_P1] & 0x7F) << 8) + inputCmdPdu[TC_P2];
        memcpy (&simResponse[1], dataArea
            + SIMClient_GetContentOffset(header, Span->cwf)
            + offset, len);
        // Dump ICCID file contents. 
        if (LOWHIGHBYTE_TO_16BITS(*(Span->cwf+SCDR_FILE_ID_LSB),*(Span->cwf+SCDR_FILE_ID_MSB)) == 0x2FE2) {
            debugDump(&simResponse[1], len, "SIM ICCID File: ", dsONESPACED);
        }
        simResponse[*rspSizePtr - 2] = 0x90;
        simResponse[*rspSizePtr - 1] = 0;
        /** WGK - Started coding large transparent file response like Rogers SIM, but didn't find behavior in specs, so exclude for now.
		 ** Note: Not done yet are changes to keep current file offset for use in Get Response processing.
        int remaining = LOWHIGHBYTE_TO_16BITS(*(Span->cwf+SCDR_CONTENT_LENGTH_LSB),*(Span->cwf+SCDR_CONTENT_LENGTH_MSB)) - (offset + len);
        if (remaining) {
            simResponse[*rspSizePtr - 2] = 0x61;
            simResponse[*rspSizePtr - 1] = remaining & 0xFF;
        }
        ***/
        }
        break;
    case CMD_READREC:   {
    	debug("| SIMSimulator: CMD_READREC");
        int     rec;
        int     error1 = 0;
        isRspFromCmdToMIS = rspSuppliedNotToMIS;
        if (Span->curEntryState != CurEntry_IsInCache) {
            // ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: READREC for file not in Cache"));
            isRspFromCmdToMIS = rspFromMIS;
            break;
        }
        if (!Span->cwf) {
            *rspSizePtr = 3;
            if (isUMTS) {
                simResponse[1] = 0x69;
                simResponse[2] = 0x86;
            } else {
                simResponse[1] = 0x94;
                simResponse[2] = 0;
            }
            break;
        }
        if (!inputCmdPdu[TC_Length]) { // at least the Datang DTM6106 phone considers this a query for the record length
            unsigned char   recLen = 0;
            unsigned char * cPtr = dataArea + SIMClient_GetStatusOffset(header, Span->cwf);
            if (isUMTS) {
                if (*cPtr++ != 0x62) {
                    ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Bad initial tag for FCP %x", *(cPtr-1)));
                    isRspFromCmdToMIS = rspFromMIS;
                    break;
                }
                int lengthFCP = *cPtr++;
                for (int current = 0; current < lengthFCP; ) {
                    switch (*cPtr) {
                    case 0x82:
                        {
                        unsigned char   dirFileType = (*(cPtr+2) & 0x38) >> 3;
                        if (dirFileType == WORKING_EF || dirFileType == INTERNAL_EF) {
                            unsigned char   fileType = *(cPtr+2) & 0x07;
                            if (fileType == LINEAR_FIXED || fileType == CYCLIC) {
                                if (*(cPtr+1) != 5) {
                                    ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Bad length for 82 FCP tag %x", *(cPtr+1)));
                                    isRspFromCmdToMIS = rspFromMIS;
                                    break;
                                }
                                recLen = *(cPtr+5);
                            }
                        }
                        }
                        break;
                    case 0x80:
                    case 0x81:
                    case 0x83:
                    case 0x84:
                    case 0x88:
                    case 0x8A:
                    case 0x8B:
                    case 0x8C:
                    case 0xA5:
                    case 0xAB:
                    case 0xC6:
                        break;
                    default:
                        ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Bad tag for FCP %x", *cPtr));
                        isRspFromCmdToMIS = rspFromMIS;
                        break;
                    }
                    if (isRspFromCmdToMIS == rspFromMIS || recLen)
                        break;
                    current += *(cPtr+1) + 2;
                    cPtr += *(cPtr+1) + 2;
                }
            } else
                recLen = cPtr[14];
            if (!recLen)
                isRspFromCmdToMIS = rspFromMIS;
            if (isRspFromCmdToMIS != rspFromMIS) {
                *rspSizePtr = 3;
                simResponse[1] = 0x6c;
                simResponse[2] = recLen;
            } else
                ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: READREC query for record length, but not set"));
            break;
        }
        if (!(*(Span->cwf+SCDR_FLAGS) & CONTENTS_VALID)) {
            ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: READREC contents not valid"));
            isRspFromCmdToMIS = rspFromMIS;
            break;
        }
        switch ((int)(inputCmdPdu[TC_P2]&0x7)) {
        case RDRECMODE_ABCU:
            if (inputCmdPdu[TC_P1])     // ABSOLUTE
                rec = inputCmdPdu[TC_P1];
            else if (Span->curRec)      // CURRENT
                rec = Span->curRec;
            else
                error1 = 1;
            break;
        case RDRECMODE_NEXT:    {
            int numRec = LOWHIGHBYTE_TO_16BITS(*(Span->cwf+SCDR_CONTENT_LENGTH_LSB),*(Span->cwf+SCDR_CONTENT_LENGTH_MSB))/inputCmdPdu[TC_Length];
            rec = Span->curRec + 1;
            if (rec > numRec) {
                int cyclic = isUMTS ? 6 : 3;
                if ((*(Span->cwf+SCDR_FLAGS) & TYPE_MASK) == cyclic)
                    rec = 1;
                else {                   // linear or bad file type
                    error1 = 1;
                    break;
                }
            }
            Span->curRec = rec;
            }
            break;
        case RDRECMODE_PREV:    {
            int numRec = LOWHIGHBYTE_TO_16BITS(*(Span->cwf+SCDR_CONTENT_LENGTH_LSB),*(Span->cwf+SCDR_CONTENT_LENGTH_MSB))/inputCmdPdu[TC_Length];
            rec = Span->curRec ? Span->curRec : numRec + 1;
            rec--;
            if (!rec) {
                int cyclic = isUMTS ? 6 : 3;
                if ((*(Span->cwf+SCDR_FLAGS) & TYPE_MASK) == cyclic)
                    rec = numRec;
                else {                   // linear or bad file type
                    error1 = 1;
                    break;
                }
            }
            Span->curRec = rec;
            }
            break;
        default:
            error1 = 1;
        }
        if (error1) {
            *rspSizePtr = 3;
            if (isUMTS) {
                simResponse[1] = 0x6A;
                simResponse[2] = 0x83;
            } else {
                simResponse[1] = 0x94;
                simResponse[2] = 0x02;
            }
            break;
        }
        *rspSizePtr = inputCmdPdu[TC_Length] + 3;
        rec--;
        memcpy (&simResponse[1], dataArea +
            SIMClient_GetContentOffset(header, Span->cwf)
            + rec*inputCmdPdu[TC_Length], inputCmdPdu[TC_Length]);
        simResponse[*rspSizePtr - 2] = 0x90;
        simResponse[*rspSizePtr - 1] = 0;
        }
        break;
    case CMD_UPDBIN:
    	debug("| SIMSimulator: CMD_UPDBIN");
        *rspSizePtr = 3;
        // RCATSP-2011: MC7700 (often at AT&T) binary update has zero length, so reply with wrong error and don't forward to MI Server.
        if (inputCmdPdu[TC_Length] == 0) {
            ddebug2(DBG2_SIM_SIMU,("SIMSimulator: Wrong length (%x) in update binary", inputCmdPdu[TC_Length]));
            simResponse[1] = 0x67;
            simResponse[2] = 0;
            isRspFromCmdToMIS = rspSuppliedNotToMIS;
        }
        else {
            if (Span->cwf)
                *(Span->cwf+SCDR_FLAGS) &= ~CONTENTS_VALID;
            simResponse[1] = 0x90;
            simResponse[2] = 0;
        }
        break;
    case CMD_UPDREC:
    	debug("| SIMSimulator: CMD_UPDREC");
        if (Span->cwf)
            *(Span->cwf+SCDR_FLAGS) &= ~CONTENTS_VALID;
        *rspSizePtr = 3;
        simResponse[1] = 0x90;
        simResponse[2] = 0;
        break;
        case CMD_VERIFYPIN:
        case CMD_UNBLKPIN:
        	debug("| SIMSimulator: CMD_VERIFYPIN/CMD_UNBLKPIN");
                if (isUMTS) {
                        if ((inputCmdPdu[TC_P2]&0x1F) && inputCmdPdu[TC_Length] == 0) { // Reset the retry PIN/PUK count
                                Span->curEntryState = CurEntry_IsInCache;
                                *rspSizePtr = 3;
                                simResponse[1] = 0x63;
                                simResponse[2] = cmd==CMD_VERIFYPIN ? 0xC3 : 0xCA;         // to max value: 3 or 10 (hex A)
                                break;
                        }
            ddebug2(DBG2_SIM_SIMU,("SIMSimulator: Cmd %x P2 %x and/or Length %x not handled",cmd,inputCmdPdu[TC_P2],inputCmdPdu[TC_Length]));
                } else
            ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Cmd %x not known", cmd));
                Span->curEntryState = CurEntry_IsFileNotInCache;
                isRspFromCmdToMIS = rspFromMIS;
                break;
    case CMD_FETCH:
    	debug("| SIMSimulator: CMD_FETCH");
        Span->lenProactive = 0;
        // fall through
    default:
        ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Cmd %x not known", cmd));
        isRspFromCmdToMIS = rspFromMIS;
    }
    Span->prevCmd = (unsigned char)(cmd & 0xFF);
    if (getDebug2Flags() & DBG2_SIM_SIMU) {
        debugDump(inputCmdPdu, getCmdLen(inputCmdPdu), "Cmd:", dsONESPACED);
        if (isRspFromCmdToMIS == rspFromMIS) {
            ddebug2(DBG2_SIM_SIMU, ("SIMSimulator: Sent to MIServer (%d/%d)", Span->probeToMisPkts, Span->misToProbeAcks));
        } else
            debugDump(simResponse+1, *rspSizePtr-1, "Rsp:", dsONESPACED);
    }
    debug("| return : %i", isRspFromCmdToMIS);
    debug("\\---------------SIMSimulator--------------------/");
    return isRspFromCmdToMIS;
}

void SIMBridge_Init(void)
{
	debug("/----------------SIMBridge_Init---------------------\\");
    int counter;
    TASK Task;

    //RCATSP-1151 - AMPs now run in their own RCATS instance, but don't yet support remote SIM

    // 20160820
    //
    int noOfSession = MAX_CLIENT_SESSIONS;
    bool ampSession = IsAmpTestSession();
    if (ampSession)
    {
    	noOfSession = 1;
    }

    for (counter = 0; counter < noOfSession; ++counter) {
        int SimLinkSlot;
        if (IsMultiRcatsSlave() && !IsDeviceIndexWeControl(ph_ConvertCellPhoneToPhoneIndex(counter)))
            continue;
        // 20160913 TL : for amp, SimLinkSlot is 0
        SimLinkSlot = (ampSession? 0: cellPhoneToSimLinkSlot(counter));
	debug("| SIMBridge_Init() for AMP, set SimLinkSlot to zero");
        debug("| SIMBridge_Init() SimLinkSlot=%d", SimLinkSlot);
        Spanner[SimLinkSlot].simLinkSlot = SimLinkSlot;
        Spanner[SimLinkSlot].PhoneID = counter;
        Spanner[SimLinkSlot].State = Span_NotRunning;
        switch (SimLinkSlot) {
        case 0:
            Spanner[SimLinkSlot].Mutex = PHASPRES; // Phone A spanner resource
            Spanner[SimLinkSlot].NetSemaphore = PHASPNTS; // Phone A spanner net bind semaphore
#ifdef FIRST_COMMAND_LOCK
            Spanner[SimLinkSlot].RcvCmdReset = PHARSTRE; // Phone A Receive Command Reset resource
#endif
            break;
        case 1:
            Spanner[SimLinkSlot].Mutex = PHBSPRES; // Phone B spanner resource
            Spanner[SimLinkSlot].NetSemaphore = PHBSPNTS; // Phone B spanner net bind semaphore
#ifdef FIRST_COMMAND_LOCK
            Spanner[SimLinkSlot].RcvCmdReset = PHBRSTRE; // Phone B Receive Command Reset resource
#endif
            break;
#if MAX_CLIENT_SESSIONS > 2
        case 2:
            Spanner[SimLinkSlot].Mutex = PHCSPRES; // Phone C spanner resource
            Spanner[SimLinkSlot].NetSemaphore = PHCSPNTS; // Phone C spanner net bind semaphore
#  ifdef FIRST_COMMAND_LOCK
            Spanner[SimLinkSlot].RcvCmdReset = PHCRSTRE; // Phone C Receive Command Reset resource
#  endif
         break;
#  endif
        default:
            error(eFATAL, "Cannot initialize spanner mutex for phone %c in SIMBridge_Init", (char)('A' + ph_ConvertCellPhoneToPhoneIndex(counter)));
        }
#if defined(SUP_N6650) || defined(SUP_G20) || defined(SUP_MC75)
        debug("| based on phone type (%i)...", ph_phone_type(ph_ConvertCellPhoneToPhoneIndex(counter)));
        switch (ph_phone_type(ph_ConvertCellPhoneToPhoneIndex(counter))) {
        case TYPE_N6650:
        case TYPE_G20:
        case TYPE_MC75:
        	debug("| > phonetype=MC75...");
            Spanner[SimLinkSlot].readingSmsState = 1;
            break;
        default:
        	debug("| > DEFAULT...");
            Spanner[SimLinkSlot].readingSmsState = 0;
            break;
        }
#endif
        Spanner[SimLinkSlot].LastAccessTimer = rcats_get_raw_time();     // Initialize the LastAccessTimer

        debug("| creating Spanner Thread...");
        Task = KS_alloc_task();
        if (Task) {
         char name[9];
         snprintf(name, sizeof name, "SPANNER%c", (char)('A' + ph_ConvertCellPhoneToPhoneIndex(counter)));
//#if defined(SUP_G20_AND_T725) || defined(SUP_G20_AND_N6650) || defined(SUP_XSCALE)
         KS_set_dynamic_task_name(Task, name);
//#endif
            // bugfix 3527 (Turning on three G24s in rapid succession will usually cause one
            //  of them to lock up/turn off (stick in status 6)): for historical reasons,
            //  KS_deftask supplied a priority that wasn't used; KS_deftask2 does, because
            //  the spanner task needs to be a high priority so that SIM traffic will be
            //  handled in a quasi-realtime fashion.
            KS_deftask2(Task,
                SPANNER_TASK_PRIORITY,
                SpannerThread);
            KS_deftask_arg(Task, &Spanner[SimLinkSlot]);
            ddebug2(DBG2_SIM_SERVER, ("| Spanner Thread: Task %d, span %p\n",Task, &Spanner[SimLinkSlot]));
            KS_execute(Task);
        } else {
            error(eFATAL, "Could not allocate a task for phone %c spanner task", (char)('A' + ph_ConvertCellPhoneToPhoneIndex(counter)));
        }
    }
	debug("\\----------------SIMBridge_Init---------------------/");
}


ResultCode SIMClientStartup(BOOL delayAfterStart)
{
debug("/-------------SIMClientStartup---------------\\");
debug("|SIMClientStartup:  IsMultiRcatsSlave() returns %i", IsMultiRcatsSlave());

    TASK Task;
    int phoneID;
    int fd;
    char buf;

    static int initialized = FALSE;

    int noOfPhone = MAX_CLIENT_SESSIONS;

    // 20160816
    bool ampSession = IsAmpTestSession();
    if ( ampSession )
    {
    	noOfPhone = 1;
	}

    // check if the SIMClient has already been started up, if so just exit silently ...
    if (initialized) { debug("| SIMClientStartup: already init"); debug("\\-------------SIMClientStartup---------------/"); return ERROR_OK; }
    initialized = TRUE;

    debug("|SIMClientStartup: calling GlobalPhoneDataInit()...");
    GlobalPhoneDataInit(ampSession);

    //RCATSP-1151 - AMPs now run in their own RCATS instance, but don't yet support remote SIM
    if ( ampSession )
    {
       debug("|for AMP!");
//       debug("\\-------------SIMClientStartup---------------/");
//       return ERROR_OK;
    }
    else
    {
    	debug("|for Probe:  open /dev/fpga0");
		if ((fd = open("/dev/fpga0", O_RDONLY)) == -1) {
			int old_errno = errno;
			debug("ERROR: Failed to open /dev/fpga0 errno=%d\n", errno);
			ostringstream ostr;
			ostr << "cannot open /dev/fpga0: errno " << old_errno;
			return MAKE_RCATS_ERROR(ERROR_RESOURCE_UNAVAILABLE, ostr.str());
		}
    	debug("|for Probe:  read from /dev/fpga0");
		if (read(fd, &buf, 1) != 1) {
			int old_errno = errno;
			debug("|ERROR: Failed to read /dev/fpga0 errno=%d\n", errno);
			ostringstream ostr;
			ostr << "FPGA not loaded: errno " << old_errno;
			return MAKE_RCATS_ERROR(ERROR_INVALID_CMD_RESP, ostr.str());
		}

		for(phoneID=0; phoneID < noOfPhone; phoneID++) {
			if (IsMultiRcatsSlave() && !IsDeviceIndexWeControl(ph_ConvertCellPhoneToPhoneIndex(phoneID)))
				continue;
			debug("|for Probe:  calling fpga_map_phone_sim for phone #%i", phoneID);
			// 20160912 TL - use new function to get phone index to support AMP
			ResultCode r = fpga_map_phone_sim( (unsigned char)GetPhoneIndexForSIMOperation(cellPhoneToSimLinkSlot(phoneID)), GetDeviceNVR_long(ph_ConvertCellPhoneToPhoneIndex(phoneID), e_local_sim) );
			if (r.fail())
			{
				debug("|NVR must be bad!!!");
				debug("\\-------------SIMClientStartup---------------/");
				return r;  // NVR must be bad
			}
		}
	}

    debug("|SIMClientStartup: calling SIMBridge_Init()...");
    SIMBridge_Init();

    debug("|SIMClientStartup: Setup & Run Task for SIMClientMain() for each phone...");
    for(phoneID=0; phoneID < noOfPhone; phoneID++) {
        int simLinkSlot;
        if (IsMultiRcatsSlave() && !IsDeviceIndexWeControl(ph_ConvertCellPhoneToPhoneIndex(phoneID)))
            continue;
        // 20160913 TL : for amp, simLinkSlot is 0
        simLinkSlot = (ampSession? 0: cellPhoneToSimLinkSlot(phoneID));
        debug("|SIMClientStartup: simLinkSlot=%i", simLinkSlot);
        Task = KS_alloc_task();
        if (Task) {
            char name[9];
            snprintf(name, sizeof name, "SIMCLI%c", (char)('A' + ph_ConvertCellPhoneToPhoneIndex(phoneID)));
            debug("|SIMClientStartup: name=%s", name);
            KS_set_dynamic_task_name(Task, name);
            // bugfix 3527 (Turning on three G24s in rapid succession will usually cause one
            //  of them to lock up/turn off (stick in status 6)): for historical reasons,
            //  KS_deftask supplied a priority that wasn't used; KS_deftask2 does, because
            //  the spanner task needs to be a high priority so that SIM traffic will be
            //  handled in a quasi-realtime fashion.
            KS_deftask2(Task, SIM_CLIENT_TASK_PRIORITY, SIMClientMain);
            KS_deftask_arg(Task, Phones[simLinkSlot]);
            debug("|SIMClientStartup: (%i) Starting SIMClient: Task %d, phSes %p\n", phoneID, Task, Phones[simLinkSlot]);
            KS_execute(Task);
        } else {
			debug("|unable to create SIM client task!!!");
        	debug("\\-------------SIMClientStartup---------------/");
            return MAKE_RCATS_ERROR(ERROR_RESOURCE_UNAVAILABLE, "unable to create SIM client task");
        }
    }

   //celder 2007Feb15 added because first preload SIM assigment always fails without small delay here.
   if (delayAfterStart)
      KS_delay(SELFTASK, 7000 / CLKTICK);

   debug("\\-------------SIMClientStartup---------------/");
    return ERROR_OK;
}

//
// Function: getCmdLen 
// Description: Determines the command w/ data length based on the
// command, as TC_Length specifies the length for command or response data.
// Returns: length of command (5 bytes) and command data.
//
int getCmdLen(unsigned char * pCmdPdu)
{
    int data_len = pCmdPdu[TC_Length]+5;

    switch (pCmdPdu[TC_OpCode])
    {
       // Only dump 5 command bytes for those without data.
       case CMD_STATUS:
       case CMD_READBIN:
       case CMD_READREC:
       case CMD_FETCH:
       case CMD_GETRSP:
           data_len = 5;
           break;
       //case CMD_SELECT:
       //case CMD_UPDBIN:
       //case CMD_UPDREC:
       //case CMD_SEARCH:
       //case CMD_VERIFYPIN:
       //case CMD_UNBLKPIN:
       //case CMD_CALCKC:
       default:
           break;
    }
    return data_len;
}
