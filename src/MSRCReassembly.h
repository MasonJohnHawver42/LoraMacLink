#pragma once
#include "MSRCTypes.h"

// Reassembles DATA fragments into a complete MSRCMessage.
// Not thread-safe — only poll() touches this.
class MSRCReassembly {
public:
    MSRCReassembly() { reset(); }

    // Feed one DATA packet. Returns:
    //   OK        — fragment accepted, message not yet complete
    //   (out set) — message is complete; caller should push to ring buffer
    //   BAD_FRAG  — fragment does not belong to the current sequence
    MSRCError feed(const MSRCPacket &pkt, MSRCMessage &out) {
        // New sequence or first fragment resets state
        if (_total == 0 || pkt.seq != _seq) {
            reset();
            _seq   = pkt.seq;
            _total = pkt.total;
        }

        if (pkt.total != _total || pkt.frag >= _total)
            return MSRCError::BAD_FRAG;

        if (_received & (1u << pkt.frag))
            return MSRCError::OK;   // duplicate, silently accept

        const size_t offset = pkt.frag * MSRC_MAX_PAYLOAD;
        if (offset + pkt.len > MSRC_MAX_MESSAGE)
            return MSRCError::BAD_LENGTH;

        memcpy(_buf + offset, pkt.payload, pkt.len);
        _received |= (1u << pkt.frag);

        // Track total assembled length using the last fragment's end position
        if (pkt.frag == _total - 1)
            _len = offset + pkt.len;

        const uint8_t fullMask = static_cast<uint8_t>((1u << _total) - 1u);
        if ((_received & fullMask) == fullMask && _len > 0) {
            out.len          = _len;
            out.timestamp_ms = static_cast<uint32_t>(millis());
            out.rssi         = _rssi;
            memcpy(out.data, _buf, _len);
            reset();
            return MSRCError::OK;   // caller checks out.len > 0
        }

        return MSRCError::OK;
    }

    void setRSSI(float rssi)  { _rssi = rssi; }
    bool inProgress() const   { return _total > 0; }
    void reset() {
        _seq      = 0;
        _total    = 0;
        _received = 0;
        _len      = 0;
        _rssi     = 0.0f;
    }

private:
    uint8_t  _seq;
    uint8_t  _total;
    uint8_t  _received;   // bitmask of received fragments (max 8)
    size_t   _len;
    float    _rssi;
    uint8_t  _buf[MSRC_MAX_MESSAGE];
};
