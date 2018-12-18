/* Imported from the dvbstream-0.2 project
 *
 * Modified for use with MPlayer, for details see the changelog at
 * http://svn.mplayerhq.hu/mplayer/trunk/
 * $Id: rtp.c 29305 2009-05-13 02:58:57Z diego $
 */

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <ctype.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>

/* MPEG-2 TS RTP stack */

#define DEBUG        1
#include "rtp.h"

#define LOG(format,...)  printf( "####### [(%s):%d] "format, __FUNCTION__, __LINE__, ##__VA_ARGS__)

// RTP reorder routines
// Also handling of repeated UDP packets (a bug of ExtremeNetworks switches firmware)
// rtpreord procedures
// write rtp packets in cache
// get rtp packets reordered

#define MAXRTPPACKETSIN 32   // The number of max packets being reordered
#define STREAM_BUFFER_MIN 2048
#define STREAM_BUFFER_SIZE (2*STREAM_BUFFER_MIN) // must be at least 2*STREAM_BUFFER_MIN

struct rtpbits {
  unsigned int v:2;           /* version: 2 */
  unsigned int p:1;           /* is there padding appended: 0 */
  unsigned int x:1;           /* number of extension headers: 0 */
  unsigned int cc:4;          /* number of CSRC identifiers: 0 */
  unsigned int m:1;           /* marker: 0 */
  unsigned int pt:7;          /* payload type: 33 for MPEG2 TS - RFC 1890 */
  unsigned int sequence:16;   /* sequence number: random */
};

struct rtpheader {	/* in network byte order */
  struct rtpbits b;
  int timestamp;	/* start: random */
  int ssrc;		/* random */
};

struct rtpbuffer
{
	unsigned char  data[MAXRTPPACKETSIN][STREAM_BUFFER_SIZE];
	unsigned short  seq[MAXRTPPACKETSIN];
	unsigned short  len[MAXRTPPACKETSIN];
	unsigned short first;
};
static struct rtpbuffer rtpbuf;

static int getrtp2(int fd, struct rtpheader *rh, char** data, int* lengthData);

// RTP Reordering functions
// Algorithm works as follows:
// If next packet is in sequence just copy it to buffer
// Otherwise copy it in cache according to its sequence number
// Cache is a circular array where "rtpbuf.first" points to next sequence slot
// and keeps track of expected sequence

// Initialize rtp cache
static void rtp_cache_reset(unsigned short seq)
{
	int i;

	rtpbuf.first = 0;
	rtpbuf.seq[0] = ++seq;

	for (i=0; i<MAXRTPPACKETSIN; i++) {
		rtpbuf.len[i] = 0;
	}
}

// Write in a cache the rtp packet in right rtp sequence order
static int rtp_cache(int fd, char *buffer, int length)
{
	struct rtpheader rh;
	int newseq;
	char *data;
	unsigned short seq;
	static int is_first = 1;

	getrtp2(fd, &rh, &data, &length);
	if(!length)
		return 0;
	seq = rh.b.sequence;

	newseq = seq - rtpbuf.seq[rtpbuf.first];

	if ((newseq == 0) || is_first)
	{
		is_first = 0;

		LOG("RTP (seq[%d]=%d seq=%d, newseq=%d)\n", rtpbuf.first, rtpbuf.seq[rtpbuf.first], seq, newseq);
		rtpbuf.first = ( 1 + rtpbuf.first ) % MAXRTPPACKETSIN;
		rtpbuf.seq[rtpbuf.first] = ++seq;
		goto feed;
	}

	if (newseq > MAXRTPPACKETSIN)
	{
		LOG( "Overrun(seq[%d]=%d seq=%d, newseq=%d)\n", rtpbuf.first, rtpbuf.seq[rtpbuf.first], seq, newseq);
		rtp_cache_reset(seq);
		goto feed;
	}

	if (newseq < 0)
	{
		int i;

		// Is it a stray packet re-sent to network?
		for (i=0; i<MAXRTPPACKETSIN; i++) {
			if (rtpbuf.seq[i] == seq) {
				LOG("Stray packet (seq[%d]=%d seq=%d, newseq=%d found at %d)\n", rtpbuf.first, rtpbuf.seq[rtpbuf.first], seq, newseq, i);
				return  0; // Yes, it is!
			}
		}
		// Some heuristic to decide when to drop packet or to restart everything
		if (newseq > -(3 * MAXRTPPACKETSIN)) {
			LOG("Too Old packet (seq[%d]=%d seq=%d, newseq=%d)\n", rtpbuf.first, rtpbuf.seq[rtpbuf.first], seq, newseq);
			return  0; // Yes, it is!
		}

		LOG( "Underrun(seq[%d]=%d seq=%d, newseq=%d)\n", rtpbuf.first, rtpbuf.seq[rtpbuf.first], seq, newseq);

		rtp_cache_reset(seq);
		goto feed;
	}

	LOG("Out of Seq (seq[%d]=%d seq=%d, newseq=%d)\n", rtpbuf.first, rtpbuf.seq[rtpbuf.first], seq, newseq);
	newseq = ( newseq + rtpbuf.first ) % MAXRTPPACKETSIN;
	memcpy (rtpbuf.data[newseq], data, length);
	rtpbuf.len[newseq] = length;
	rtpbuf.seq[newseq] = seq;

	return 0;

feed:
	memcpy (buffer, data, length);
	return length;
}

// Get next packet in cache
// Look in cache to get first packet in sequence
static int rtp_get_next(int fd, char *buffer, int length)
{
	int i;
	unsigned short nextseq;

	// If we have empty buffer we loop to fill it
	for (i=0; i < MAXRTPPACKETSIN -3; i++) {
		if (rtpbuf.len[rtpbuf.first] != 0) break;

		length = rtp_cache(fd, buffer, length) ;

		// returns on first packet in sequence
		if (length > 0) {
			LOG("Getting rtp [%d] %hu\n", i, rtpbuf.first);
			return length;
		} else if (length < 0) break;

		// Only if length == 0 loop continues!
	}

	i = rtpbuf.first;
	while (rtpbuf.len[i] == 0) {
		LOG( "Lost packet %hu\n", rtpbuf.seq[i]);
		i = ( 1 + i ) % MAXRTPPACKETSIN;
		if (rtpbuf.first == i) break;
	}
	rtpbuf.first = i;

	// Copy next non empty packet from cache
	LOG("Getting rtp from cache [%d] %hu\n", rtpbuf.first, rtpbuf.seq[rtpbuf.first]);
	memcpy (buffer, rtpbuf.data[rtpbuf.first], rtpbuf.len[rtpbuf.first]);
	length = rtpbuf.len[rtpbuf.first]; // can be zero?

	// Reset fisrt slot and go next in cache
	rtpbuf.len[rtpbuf.first] = 0;
	nextseq = rtpbuf.seq[rtpbuf.first];
	rtpbuf.first = ( 1 + rtpbuf.first ) % MAXRTPPACKETSIN;
	rtpbuf.seq[rtpbuf.first] = nextseq + 1;

	return length;
}


// Read next rtp packet using cache
int read_rtp_from_server(int fd, char *buffer, int length) {
	// Following test is ASSERT (i.e. uneuseful if code is correct)
	if(buffer==NULL || length<STREAM_BUFFER_SIZE) {
		LOG("RTP buffer invalid; no data return from network\n");
		return 0;
	}

	// loop just to skip empty packets
	while ((length = rtp_get_next(fd, buffer, length)) == 0) {
		LOG("Got empty packet from RTP cache!?\n");
	}

	return length;
}

static int getrtp2(int fd, struct rtpheader *rh, char** data, int* lengthData) {
  static char buf[1600];
  unsigned int intP;
  char* charP = (char*) &intP;
  int headerSize;
  int lengthPacket;
  lengthPacket=recv(fd,buf,1590,0);
  if (lengthPacket<0)
    LOG("rtp: socket read error\n");
  else if (lengthPacket<12)
    LOG("rtp: packet too small (%d) to be an rtp frame (>12bytes)\n", lengthPacket);
  if(lengthPacket<12) {
    *lengthData = 0;
    return 0;
  }
  rh->b.v  = (unsigned int) ((buf[0]>>6)&0x03);
  rh->b.p  = (unsigned int) ((buf[0]>>5)&0x01);
  rh->b.x  = (unsigned int) ((buf[0]>>4)&0x01);
  rh->b.cc = (unsigned int) ((buf[0]>>0)&0x0f);
  rh->b.m  = (unsigned int) ((buf[1]>>7)&0x01);
  rh->b.pt = (unsigned int) ((buf[1]>>0)&0x7f);
  intP = 0;
  memcpy(charP+2,&buf[2],2);
  rh->b.sequence = ntohl(intP);
  intP = 0;
  memcpy(charP,&buf[4],4);
  rh->timestamp = ntohl(intP);

  headerSize = 12 + 4*rh->b.cc; /* in bytes */
  LOG("recv pack len =%d,head len =%d\n",lengthPacket,headerSize);
  *lengthData = lengthPacket - headerSize;
  *data = (char*) buf + headerSize;

  LOG("Reading rtp: v=%x p=%x x=%x cc=%x m=%x pt=%x seq=%x ts=%x lgth=%d\n",rh->b.v,rh->b.p,rh->b.x,rh->b.cc,rh->b.m,rh->b.pt,rh->b.sequence,rh->timestamp,lengthPacket);

  return 0;
}


#define AV_RB16(x)  ((((const uint8_t*)(x))[0] << 8) | ((const uint8_t*)(x))[1])


#define COUNT_NAL_TYPE(data, nal) do { } while (0)
#define NAL_COUNTERS NULL
#define NAL_MASK 0x1f


static const uint8_t start_sequence[] = { 0x00, 0x00, 0x00, 0x01 };

int ff_h264_handle_frag_packet(uint8_t * h264date, const uint8_t *buf, int len,
                               int start_bit, const uint8_t *nal_header,
                               int nal_header_len)
{
    int ret;
    int tot_len = len;
    int pos = 0;
    if (start_bit)
        tot_len += sizeof(start_sequence) + nal_header_len;
    if (NULL == h264date)
        return 0;
    if (start_bit) {
        memcpy(h264date + pos, start_sequence, sizeof(start_sequence));
        pos += sizeof(start_sequence);
        memcpy(h264date + pos, nal_header, nal_header_len);
        pos += nal_header_len;
    }
    memcpy(h264date + pos, buf, len);
    return len + pos;
}


static int h264_handle_packet_fu_a( uint8_t * h264date,
                                   const uint8_t *buf, int len,
                                   int *nal_counters, int nal_mask)
{
    uint8_t fu_indicator, fu_header, start_bit, nal_type, nal;

    if (len < 3) {
        LOG( "Too short data for FU-A H264 RTP packet\n");
        return 0;
    }

    fu_indicator = buf[0];
    fu_header    = buf[1];
    start_bit    = fu_header >> 7;
    nal_type     = fu_header & 0x1f;
    nal          = fu_indicator & 0xe0 | nal_type;

    // skip the fu_indicator and fu_header
    buf += 2;
    len -= 2;

    if (start_bit && nal_counters)
        nal_counters[nal_type & nal_mask]++;
    return ff_h264_handle_frag_packet(h264date, buf, len, start_bit, &nal, 1);
}

int ff_h264_handle_aggregated_packet(uint8_t * h264date,
													 const uint8_t *buf, int len,
													 int skip_between, int *nal_counters,
													 int nal_mask)
{
    int pass         = 0;
    int total_length = 0;
    uint8_t *dst     = NULL;
    int ret;

    // first we are going to figure out the total size
    for (pass = 0; pass < 2; pass++) {
        const uint8_t *src = buf;
        int src_len        = len;

        while (src_len > 2) {
            uint16_t nal_size = AV_RB16(src);

            // consume the length of the aggregate
            src     += 2;
            src_len -= 2;

            if (nal_size <= src_len) {
                if (pass == 0) {
                    // counting
                    total_length += sizeof(start_sequence) + nal_size;
                } else {
                    // copying
                    memcpy(dst, start_sequence, sizeof(start_sequence));
                    dst += sizeof(start_sequence);
                    memcpy(dst, src, nal_size);
                    if (nal_counters)
                        nal_counters[(*src) & nal_mask]++;
                    dst += nal_size;
                }
            } else {
                LOG("nal size exceeds length: %d %d\n", nal_size, src_len);
                return 0;
            }

            // eat what we handled
            src     += nal_size + skip_between;
            src_len -= nal_size + skip_between;
        }

        if (pass == 0) {
            /* now we know the total size of the packet (with the
             * start sequences added) */
            if (NULL == h264date)
                return 0;
            dst = h264date;
        }
    }

    return total_length;
}


int decrtph264(const uint8_t *rtpdate,int rtpdatelen,uint8_t * h264date)
{
	uint8_t nal;
	uint8_t type;
	int result = 0;

	if (!rtpdatelen) {
		LOG( "Empty H264 RTP packet\n");
		return 0;
	}
	nal  = rtpdate[0];
	type = nal & 0x1f;

	/* Simplify the case (these are all the nal types used internally by
	 * the h264 codec). */
	if (type >= 1 && type <= 23)
		type = 1;
	switch (type) {
	case 0: 				   // undefined, but pass them through
	case 1:
		if (NULL == h264date)
		{
			LOG("h264date NULL\n");
			return 0;
		}
		memcpy(h264date, start_sequence, sizeof(start_sequence));
		memcpy(h264date + sizeof(start_sequence), rtpdate, rtpdatelen);
		result = rtpdatelen + sizeof(start_sequence);
		//COUNT_NAL_TYPE(data, nal);
		break;

	case 24:				   // STAP-A (one packet, multiple nals)
		// consume the STAP-A NAL
		rtpdate++;
		rtpdatelen --;
		result = ff_h264_handle_aggregated_packet(h264date, rtpdate, rtpdatelen, 0,
												  NAL_COUNTERS, NAL_MASK);
		break;

	case 25:				   // STAP-B
	case 26:				   // MTAP-16
	case 27:				   // MTAP-24
	case 29:				   // FU-B
		LOG("Unhandled type (%d) (See RFC for implementation details)\n",type);
		result = 0;
		break;

	case 28:				   // FU-A (fragmented nal)
		result = h264_handle_packet_fu_a( h264date, rtpdate, rtpdatelen,
										 NAL_COUNTERS, NAL_MASK);
		break;

	case 30:				   // undefined
	case 31:				   // undefined
	default:
		LOG("Undefined type (%d)\n", type);
		result = 0;
		break;
	}

	//pkt->stream_index = st->index;

	return result;
}
