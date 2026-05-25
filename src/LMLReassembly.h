#pragma once
#include "LMLTypes.h"

// Reassembles DATA fragments into a complete LMLMessage.
// Not thread-safe — only call from poll().
class LMLReassembly {
public:
    LMLReassembly() { reset(); }

    // Returns OK with out.len > 0 when message is complete, OK with out.len==0
    // while still accumulating, or BAD_FRAG / BAD_LENGTH on structural error.
    LMLError feed(const LMLPacket &pkt, LMLMessage &out) {
        out.len = 0;

        if (_total == 0 || pkt.seq != _seq) {
            reset();
            _seq   = pkt.seq;
            _total = pkt.total;
        }

        if (pkt.total != _total || pkt.frag >= _total)
            return LMLError::BAD_FRAG;

        if (_received & (1u << pkt.frag))
            return LMLError::OK; // duplicate

        const size_t offset = pkt.frag * LML_MAX_PAYLOAD;
        if (offset + pkt.len > LML_MAX_MESSAGE)
            return LMLError::BAD_LENGTH;

        memcpy(_buf + offset, pkt.payload, pkt.len);
        _received |= (1u << pkt.frag);

        if (pkt.frag == _total - 1)
            _len = offset + pkt.len;

        const uint8_t fullMask = static_cast<uint8_t>((1u << _total) - 1u);
        if ((_received & fullMask) == fullMask && _len > 0) {
            out.len          = _len;
            out.timestamp_ms = static_cast<uint32_t>(millis());
            out.rssi         = _rssi;
            memcpy(out.data, _buf, _len);
            reset();
        }

        return LMLError::OK;
    }

    void setRSSI(float rssi) { _rssi = rssi; }
    bool inProgress() const  { return _total > 0; }

    void reset() {
        _seq      = 0;
        _total    = 0;
        _received = 0;
        _len      = 0;
        _rssi     = 0.0f;
    }

private:
    uint8_t _seq;
    uint8_t _total;
    uint8_t _received;
    size_t  _len;
    float   _rssi;
    uint8_t _buf[LML_MAX_MESSAGE];
};
