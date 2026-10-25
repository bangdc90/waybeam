#ifndef VENC_AU_H
#define VENC_AU_H

#include "venc_frame_ring.h"

#include <stddef.h>

#define VENC_AU_MAX_PACKS 32

typedef struct {
	const unsigned char *data;
	size_t data_size;
	size_t offset;
	size_t length;
} VencAuPackView;

/* Publish one complete Annex-B access unit assembled from bounded pack views.
 * Returns 0 on publication and -1 for malformed input or ring failure. */
int venc_au_publish(venc_frame_ring_t *ring, const VencFrameMeta *meta,
	const VencAuPackView *packs, size_t pack_count);

#endif