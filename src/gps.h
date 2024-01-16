/*
 * GPS class
 *
 * (c) 2024 Erik Tkal
 *
 */

#pragma once

#include <pico/stdlib.h>
#include <hardware/uart.h>
#include <string>
#include <vector>
#include <map>
#include <memory>

#include "lwip/tcp.h"

class SatInfo
{
public:
    SatInfo(uint num = 0, uint el = 0, uint az = 0, uint rssi = 0)
    {
        m_num  = num;
        m_el   = el;
        m_az   = az;
        m_rssi = rssi;
    }
    ~SatInfo()
    {
    }

    uint m_num;
    uint m_el;
    uint m_az;
    uint m_rssi;
};

typedef std::map<uint, SatInfo> SatList;
typedef std::vector<uint> UsedList;

class GPSData
{
public:
    typedef std::shared_ptr<GPSData> Shared;

    GPSData()  = default;
    ~GPSData() = default;

    std::string strLatitude;
    std::string strLongitude;
    std::string strAltitude;
    std::string strNumSats;
    std::string strGPSTime;
    std::string strMode3D;
    std::string strSpeedKts;
    SatList mSatList;
    UsedList vUsedList;
};

typedef void (*sentenceCallback)(void* pCtx, std::string strSentence);
typedef void (*gpsDataCallback)(void* pCtx, GPSData::Shared spGPSData);

class GPS
{
public:
    typedef std::shared_ptr<GPS> Shared;

    GPS();
    ~GPS();

    void SetSentenceCallback(void* pCtx, sentenceCallback pCB);
    void SetGpsDataCallback(void* pCtx, gpsDataCallback pCB);
    void Run();
    bool HasPosition()
    {
        return m_bFixPos;
    }
    bool ExternalAntenna()
    {
        return m_bExternalAntenna;
    }

    // Static callbacks for lwIP
    static err_t TCP_connected(void* arg, struct tcp_pcb* pcb, err_t err);
    static err_t TCP_poll(void* arg, struct tcp_pcb* pcb);
    static err_t TCP_sent(void* arg, struct tcp_pcb* pcb, u16_t len);
    static err_t TCP_recv(void* arg, struct tcp_pcb* pcb, struct pbuf* p, err_t err);
    static void TCP_err(void* arg, err_t err);

private:
    void processSentence(std::string strSentence);
    bool validateSentence(std::string& strSentence);
    std::string checkSum(const std::string& strSentence);
    std::string convertToDegrees(std::string strRaw, int width);

    void tcp_write_string(const std::string& str);

    // Member callbacks for lwIP
    err_t tcp_connected(struct tcp_pcb* pcb, err_t err);
    err_t tcp_poll(struct tcp_pcb* pcb);
    err_t tcp_sent(struct tcp_pcb* pcb, u16_t len);
    err_t tcp_recv(struct tcp_pcb* pcb, struct pbuf* p, err_t err);
    void tcp_err(err_t err);

    bool m_bExit;
    bool m_bFixTime;
    bool m_bFixPos;
    bool m_bExternalAntenna;
    bool m_bGSVInProgress;
    std::string m_strNumGSV;
    uint64_t m_nSatListTime;
    GPSData::Shared m_spGPSData;
    SatList m_mSatListPersistent;

    sentenceCallback m_pSentenceCallBack;
    void* m_pSentenceCtx;
    gpsDataCallback m_pGpsDataCallback;
    void* m_pGpsDataCtx;

    struct tcp_pcb* m_pTcpPcb;
    ip_addr_t m_remoteAddr;
};
