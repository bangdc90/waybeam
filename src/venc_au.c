#include "venc_au.h"

#include <stdint.h>

int venc_au_publish(venc_frame_ring_t *ring, const VencFrameMeta *meta,
	const VencAuPackView *packs, size_t pack_count)
{
	size_t total = 0;
	size_t max_payload;

	if (!ring || !ring->hdr || !meta || !packs || pack_count == 0 ||
	    pack_count > VENC_AU_MAX_PACKS ||
	    ring->slot_data_size < VENC_FRAME_META_SIZE)
		return -1;
	max_payload = ring->slot_data_size - VENC_FRAME_META_SIZE;
	for (size_t i = 0; i < pack_count; i++) {
		const VencAuPackView *pack = &packs[i];
		if (!pack->data || pack->length == 0 || pack->offset > pack->data_size ||
		    pack->length > pack->data_size - pack->offset ||
		    pack->length > max_payload - total)
			return -1;
		total += pack->length;
	}
	if (total == 0 || total > UINT32_MAX)
		return -1;
	if (venc_frame_ring_begin_write(ring, meta) != 0)
		return -1;
	for (size_t i = 0; i < pack_count; i++) {
		const VencAuPackView *pack = &packs[i];
		if (venc_frame_ring_append(ring, pack->data + pack->offset,
		    (uint32_t)pack->length) != 0) {
			venc_frame_ring_abort_write(ring);
			return -1;
		}
	}
	venc_frame_ring_commit_write(ring);
	return 0;
}