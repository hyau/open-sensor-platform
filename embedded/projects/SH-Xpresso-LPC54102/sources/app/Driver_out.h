#ifndef _DRIVER_OUT_H_
#define _DRIVER_OUT_H_

/* Initial approximation of a CMSIS style OSP data output driver.
 * Other methods may be defined later if desired for close CMSIS style
 * adherence.
 *
 * Expected driver functionality:
 * Driver MUST define a global symbol of type struct _Driver_OUT. It MUST
 *   be named Driver_OUT.
 * Driver MUST pre-initialize the global struct with entry for its entry point.
 *   All unused entry point must be one of 2 things:
 *   1. Mandatory points must return an error.
 *   2. Optional ones may either return an error or be set to NULL.
 *
 * Driver's initialize entry point will be called prior to any other usage.
 *    The initialize function is called with a parameter containing a callback
 *    for receive. When the driver receives a HIF packet, it should call
 *    it with the address and length. Upon return from the callback, the buffer
 *    can be reused as the driver sees fit. (i.e. the callback is responsible
 *    for copying any data it wants to use later).
 * 
 * GetBuffer should return a suitably aligned buffer of a size to hold the 
 *    largest HIF packet. It should be capable of being called more then once.
 *    It should support up to 3 calls without an intervening PutBuffer.
 *    Driver MUST NOT make any assumptions on the buffer once it is returned.
 *
 * PutBuffer should take the buffer previously returned by GetBuffer and do
 *    whatever it is needed to transmit out of the system. The length of valid
 *    data to transmit is provided as the 2nd argument. Once PutBuffer
 *    is called, the pointer previously returned by GetBuffer SHALL be invalid
 *    as far as the driver user is concerned. 
 *
 *
 * Notes:
 *    1. The buffers will be filled with HIF data in a ready to transmit
 *    format. Copying can be minimized if the provided buffer is a buffer
 *    matching the underlying transport.
 *    2. The requirement for up to 3 calls is to allow for error handling.
 *    It avoids the need to "abort" a buffer in the event packetization 
 *    errors occur. In the event of an error, the buffer is merely held for 
 *    the next use.
 */

struct _Driver_OUT {
	void (initialize)(void (*cb)(void *, int);
	void *(*GetBuffer)(void);
	void (*PutBuffer)(void *, int);
};
extern struct _Driver_OUT Driver_OUT;

#endif
