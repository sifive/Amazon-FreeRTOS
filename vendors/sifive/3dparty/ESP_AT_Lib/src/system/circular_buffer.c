#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>
#include <FreeRTOS.h>
#include "portmacro.h"

#include "circular_buffer.h"

void __attribute((section(".data.advance_pointer")))
advance_pointer(cbuf_handle_t cbuf)
{
    configASSERT(cbuf);

	if(cbuf->full)
    {
		configPRINT_STRING(("Circular buffer overflowed!\n"));
        cbuf->tail = (cbuf->tail + 1) % cbuf->max;
    }

	cbuf->head = (cbuf->head + 1) % cbuf->max;

	// We mark full because we will advance tail on the next time around
	cbuf->full = (cbuf->head == cbuf->tail);
}

static void retreat_pointer(cbuf_handle_t cbuf)
{
    configASSERT(cbuf);

	cbuf->full = false;
	cbuf->tail = (cbuf->tail + 1) % cbuf->max;
}

cbuf_handle_t circular_buf_init(uint8_t* buffer, size_t size)
{
    configASSERT(buffer && size);

	cbuf_handle_t cbuf = pvPortMalloc(sizeof(circular_buf_t));
    configASSERT(cbuf);

	cbuf->buffer = buffer;
	cbuf->max = size;
	circular_buf_reset(cbuf);

    configASSERT(circular_buf_empty(cbuf));

	return cbuf;
}

void circular_buf_free(cbuf_handle_t cbuf)
{
    configASSERT(cbuf);
    vPortFree(cbuf);
}

void circular_buf_reset(cbuf_handle_t cbuf)
{
    configASSERT(cbuf);

    cbuf->head = 0;
    cbuf->tail = 0;
    cbuf->full = false;
}

size_t circular_buf_size(cbuf_handle_t cbuf)
{
    configASSERT(cbuf);

	size_t size = cbuf->max;

	if(!cbuf->full)
	{
		if(cbuf->head >= cbuf->tail)
		{
			size = (cbuf->head - cbuf->tail);
		}
		else
		{
			size = (cbuf->max + cbuf->head - cbuf->tail);
		}

	}

	return size;
}

size_t circular_buf_capacity(cbuf_handle_t cbuf)
{
    configASSERT(cbuf);

	return cbuf->max;
}

void __attribute((section(".data.circular_buf_put")))
circular_buf_put(cbuf_handle_t cbuf, uint8_t data)
{
    configASSERT(cbuf && cbuf->buffer);

    cbuf->buffer[cbuf->head] = data;

    advance_pointer(cbuf);
}

int circular_buf_put2(cbuf_handle_t cbuf, uint8_t data)
{
    int r = -1;

    configASSERT(cbuf && cbuf->buffer);

    if(!circular_buf_full(cbuf))
    {
        cbuf->buffer[cbuf->head] = data;
        advance_pointer(cbuf);
        r = 0;
    }

    return r;
}

int circular_buf_get(cbuf_handle_t cbuf, uint8_t * data)
{
    configASSERT(cbuf && data && cbuf->buffer);

    int r = -1;

    if(!circular_buf_empty(cbuf))
    {
        *data = cbuf->buffer[cbuf->tail];
        retreat_pointer(cbuf);

        r = 0;
    }

    return r;
}

uint8_t *circular_buf_front(cbuf_handle_t cbuf, size_t *len)
{
    configASSERT(cbuf && cbuf->buffer);

    if(!circular_buf_empty(cbuf))
    {
    	size_t size = 0;

		if(cbuf->head >= cbuf->tail)
		{
			size = (cbuf->head - cbuf->tail);
		}
		else
		{
			size = (cbuf->max - cbuf->tail);
		}

		*len = size;

        return &cbuf->buffer[cbuf->tail];
    }

    if(len)
    	*len = 0;

    return 0;
}

void circular_buf_pop(cbuf_handle_t cbuf, size_t len)
{
    configASSERT(cbuf && cbuf->buffer);

    if(!circular_buf_empty(cbuf))
    {
    	while(len--)
    		retreat_pointer(cbuf);
    }
}

bool circular_buf_empty(cbuf_handle_t cbuf)
{
    configASSERT(cbuf);

    return (!cbuf->full && (cbuf->head == cbuf->tail));
}

bool circular_buf_full(cbuf_handle_t cbuf)
{
    configASSERT(cbuf);

    return cbuf->full;
}

bool circular_buf_peek_and_compare(cbuf_handle_t cbuf, const uint8_t *data, size_t len)
{
    configASSERT(cbuf);
    size_t pointer = cbuf -> tail;
    if (len > circular_buf_size(cbuf)) {
        return false;
    }
    bool first_byte_is_head = cbuf->full; // When the buffer is full head = tail so we need to skip first checking first time. Can be refactored, I think
    for (size_t i = 0; i < len; ++i, pointer = (pointer + 1) % cbuf->max){
        if ((first_byte_is_head ? false : pointer == cbuf->head) || cbuf->buffer[pointer] != data[i]){
            return false;
        }
        first_byte_is_head = false;
    }
    return true;
}
