/*
 * Copyright 2016 - 2017 Karsten Schmidt - Apache Software License 2.0
 * Modified 2024 by John Lindgren
 */
#include "tinyalloc.h"
#include <errno.h>
#include <stdint.h>
#include <string.h>

typedef struct Block Block;

struct Block {
    void *addr;
    Block *next;
    size_t size;
};

typedef struct {
    Block *free;   // first free block
    Block *used;   // first used block
    Block *fresh;  // first available blank block
    size_t top;    // top free addr
} Heap;

/**
 * Inserts block into free list, sorted by addr.
 */
static void insert_block(Heap *heap, Block *block) {
    Block *ptr  = heap->free;
    Block *prev = NULL;
    while (ptr != NULL) {
        if ((size_t)block->addr <= (size_t)ptr->addr) {
            break;
        }
        prev = ptr;
        ptr  = ptr->next;
    }
    if (prev != NULL) {
        prev->next = block;
    } else {
        heap->free = block;
    }
    block->next = ptr;
}

static void release_blocks(Heap *heap, Block *scan, Block *to) {
    Block *scan_next;
    while (scan != to) {
        scan_next   = scan->next;
        scan->next  = heap->fresh;
        heap->fresh = scan;
        scan->addr  = 0;
        scan->size  = 0;
        scan        = scan_next;
    }
}

static void compact(Heap *heap) {
    Block *ptr = heap->free;
    Block *prev;
    Block *scan;
    while (ptr != NULL) {
        prev = ptr;
        scan = ptr->next;
        while (scan != NULL &&
               (size_t)prev->addr + prev->size == (size_t)scan->addr) {
            prev = scan;
            scan = scan->next;
        }
        if (prev != ptr) {
            size_t new_size =
                (size_t)prev->addr - (size_t)ptr->addr + prev->size;
            ptr->size   = new_size;
            Block *next = prev->next;
            // make merged blocks available
            release_blocks(heap, ptr->next, prev->next);
            // relink
            ptr->next = next;
        }
        ptr = ptr->next;
    }
}

void ta_init(const ta_cfg_t *cfg) {
    Heap *heap  = (Heap *)cfg->base;
    heap->free  = NULL;
    heap->used  = NULL;
    heap->fresh = (Block *)(heap + 1);
    heap->top   = (size_t)(heap->fresh + cfg->max_blocks);

    Block *block = heap->fresh;
    size_t i     = cfg->max_blocks - 1;
    while (i--) {
        block->next = block + 1;
        block++;
    }
    block->next = NULL;
}

bool ta_free(const ta_cfg_t *cfg, void *ptr) {
    if (ptr == NULL) {
        return false;
    }
    Heap *heap   = (Heap *)cfg->base;
    Block *block = heap->used;
    Block *prev  = NULL;
    while (block != NULL) {
        if (ptr == block->addr) {
            if (prev) {
                prev->next = block->next;
            } else {
                heap->used = block->next;
            }
            insert_block(heap, block);
            compact(heap);
            return true;
        }
        prev  = block;
        block = block->next;
    }
    return false;
}

static Block *alloc_block(const ta_cfg_t *cfg, size_t num) {
    Heap *heap  = (Heap *)cfg->base;
    Block *ptr  = heap->free;
    Block *prev = NULL;
    size_t top  = heap->top;
    if (num > -cfg->alignment) {
        return NULL;  // prevent overflow
    }
    num = (num + cfg->alignment - 1) & -cfg->alignment;
    if (num == 0) {
        num = cfg->alignment;  // prevent zero-size block
    }
    while (ptr != NULL) {
        const int is_top = ((size_t)ptr->addr + ptr->size >= top) &&
                           (num <= (size_t)cfg->limit - (size_t)ptr->addr);
        if (is_top || ptr->size >= num) {
            if (prev != NULL) {
                prev->next = ptr->next;
            } else {
                heap->free = ptr->next;
            }
            ptr->next  = heap->used;
            heap->used = ptr;
            if (is_top) {
                ptr->size = num;
                heap->top = (size_t)ptr->addr + num;
            } else if (heap->fresh != NULL) {
                size_t excess = ptr->size - num;
                if (excess >= cfg->split_thresh) {
                    ptr->size    = num;
                    Block *split = heap->fresh;
                    heap->fresh  = split->next;
                    split->addr  = (void *)((size_t)ptr->addr + num);
                    split->size  = excess;
                    insert_block(heap, split);
                    compact(heap);
                }
            }
            return ptr;
        }
        prev = ptr;
        ptr  = ptr->next;
    }
    // no matching free blocks
    // see if any other blocks available
    if (heap->fresh != NULL && (num <= (size_t)cfg->limit - top)) {
        ptr         = heap->fresh;
        heap->fresh = ptr->next;
        ptr->addr   = (void *)top;
        ptr->next   = heap->used;
        ptr->size   = num;
        heap->used  = ptr;
        heap->top   = top + num;
        return ptr;
    }
    return NULL;
}

void *ta_alloc(const ta_cfg_t *cfg, size_t num) {
    Block *block = alloc_block(cfg, num);
    if (block != NULL) {
        return block->addr;
    }
    errno = ENOMEM;
    return NULL;
}

void *ta_calloc(const ta_cfg_t *cfg, size_t num, size_t size) {
    size_t orig = num;
    num *= size;
    // check for overflow
    if (size == 0 || num / size == orig) {
        Block *block = alloc_block(cfg, num);
        if (block != NULL) {
            memset(block->addr, 0, block->size);
            return block->addr;
        }
    }
    errno = ENOMEM;
    return NULL;
}

size_t ta_getsize(const ta_cfg_t *cfg, void *ptr) {
    if (ptr == NULL) {
        return 0;
    }
    Heap *heap   = (Heap *)cfg->base;
    Block *block = heap->used;
    while (block != NULL) {
        if (ptr == block->addr) {
            return block->size;
        }
        block = block->next;
    }
    return 0;
}

void *ta_realloc(const ta_cfg_t *cfg, void *ptr, size_t num) {
    if (ptr == NULL) {
        return ta_alloc(cfg, num);
    } else if (num == 0) {
        ta_free(cfg, ptr);
        return NULL;
    }
    size_t size = ta_getsize(cfg, ptr);
    if (num <= size && size - num <= cfg->split_thresh) {
        return ptr;  // keep current block
    }
    Block *block = alloc_block(cfg, num);
    if (block != NULL) {
        if (size > num) {
            size = num;
        }
        memcpy(block->addr, ptr, size);
        bool free_rt = ta_free(cfg, ptr);
        return block->addr;
    }
    errno = ENOMEM;
    return NULL;
}

static size_t count_blocks(Block *ptr) {
    size_t num = 0;
    while (ptr != NULL) {
        num++;
        ptr = ptr->next;
    }
    return num;
}

size_t ta_num_free(const ta_cfg_t *cfg) {
    Heap *heap = (Heap *)cfg->base;
    return count_blocks(heap->free);
}

size_t ta_num_used(const ta_cfg_t *cfg) {
    Heap *heap = (Heap *)cfg->base;
    return count_blocks(heap->used);
}

size_t ta_num_fresh(const ta_cfg_t *cfg) {
    Heap *heap = (Heap *)cfg->base;
    return count_blocks(heap->fresh);
}

bool ta_check(const ta_cfg_t *cfg) {
    return cfg->max_blocks ==
           ta_num_free(cfg) + ta_num_used(cfg) + ta_num_fresh(cfg);
}
