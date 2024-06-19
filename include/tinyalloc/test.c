#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* tinyalloc */
#include "tinyalloc.h"

static char heap[256 * 1024 * 1024];
static const ta_cfg_t cfg = {
    .base         = heap,
    .limit        = &heap[sizeof(heap)],
    .max_blocks   = 256 * 1024,
    .split_thresh = 16,
    .alignment    = 16,
};

#define malloc(size) ta_alloc(&cfg, size)
#define calloc(n, size) ta_calloc(&cfg, n, size)
#define free(p) ta_free(&cfg, p)
#define realloc(p, size) ta_realloc(&cfg, p, size)

#define MAX_TEST_BLOCKS (100000)
#define NUM_RANDOM_ALLOCS (100000)
#define REQUIRED_ALIGN (16)

typedef struct {
    uint8_t *mem;
    size_t size;
    uint8_t fill;
} TestBlock;

static TestBlock test_blocks[MAX_TEST_BLOCKS];

static size_t num_blocks;
static size_t total_bytes;
static size_t num_success;
static size_t num_failed;
static size_t smallest_fail = SIZE_MAX;

static bool each_equal(uint8_t *mem, uint8_t val, size_t size) {
    for (size_t i = 0; i < size; i++) {
        if (mem[i] != val) {
            return false;
        }
    }
    return true;
}

static size_t rand_bias_low(void) {
    int bits    = rand() % 32;
    size_t mask = ((size_t)1 << bits) * 2 - 1;
    return (size_t)rand() & mask;
}

static bool test_malloc(size_t idx, size_t size, uint8_t fill) {
    assert(test_blocks[idx].mem == NULL);
    uint8_t *mem = malloc(size);
    if (mem == NULL) {
        size_t fail_size = total_bytes + size;
        if (fail_size > total_bytes /* check overflow */ &&
            fail_size < smallest_fail) {
            smallest_fail = fail_size;
        }
        num_failed++;
        return false;
    }

    assert((uintptr_t)mem % REQUIRED_ALIGN == 0);
    memset(mem, fill, size);

    test_blocks[idx].mem  = mem;
    test_blocks[idx].size = size;
    test_blocks[idx].fill = fill;

    num_blocks++;
    total_bytes += size;
    num_success++;
    return true;
}

static bool test_calloc(size_t idx, size_t num, size_t size, uint8_t fill) {
    assert(test_blocks[idx].mem == NULL);
    size_t num_x_size = num * size;
    uint8_t *mem      = calloc(num, size);
    if (mem == NULL) {
        size_t fail_size = total_bytes + num_x_size;
        if ((size == 0 || num_x_size / size == num) /* check overflow */ &&
            fail_size > total_bytes /* check overflow */ &&
            fail_size < smallest_fail) {
            smallest_fail = fail_size;
        }
        num_failed++;
        return false;
    }

    assert((uintptr_t)mem % REQUIRED_ALIGN == 0);
    assert(each_equal(mem, 0, num_x_size));
    memset(mem, fill, num_x_size);

    test_blocks[idx].mem  = mem;
    test_blocks[idx].size = num_x_size;
    test_blocks[idx].fill = fill;

    num_blocks++;
    total_bytes += num_x_size;
    num_success++;
    return true;
}

static bool test_realloc(size_t idx, size_t size, uint8_t fill) {
    uint8_t *oldmem = test_blocks[idx].mem;
    size_t oldsize  = (oldmem == NULL) ? 0 : test_blocks[idx].size;
    uint8_t oldfill = test_blocks[idx].fill;
    if (oldmem != NULL) {
        assert(each_equal(oldmem, oldfill, oldsize));
    }

    uint8_t *mem = realloc(oldmem, size);
    if (oldmem != NULL && size == 0) {
        assert(mem == NULL);
        test_blocks[idx].mem = NULL;
        num_blocks--;
        total_bytes -= oldsize;
        return true;
    }

    if (mem == NULL) {
        size_t fail_size = total_bytes - oldsize + size;
        if (fail_size > total_bytes - oldsize /* check overflow */ &&
            fail_size < smallest_fail) {
            smallest_fail = fail_size;
        }
        num_failed++;
        return false;
    }

    assert((uintptr_t)mem % REQUIRED_ALIGN == 0);
    size_t minsize = (size < oldsize) ? size : oldsize;
    assert(each_equal(mem, oldfill, minsize));
    memset(mem, fill, size);

    test_blocks[idx].mem  = mem;
    test_blocks[idx].size = size;
    test_blocks[idx].fill = fill;

    if (oldmem == NULL) {
        num_blocks++;
    }
    total_bytes = total_bytes - oldsize + size;
    num_success++;
    return true;
}

static void test_free(size_t idx) {
    uint8_t *mem = test_blocks[idx].mem;
    size_t size  = (mem == NULL) ? 0 : test_blocks[idx].size;
    assert(each_equal(mem, test_blocks[idx].fill, size));

    free(mem);
    test_blocks[idx].mem = NULL;

    if (mem != NULL) {
        num_blocks--;
    }
    total_bytes -= size;
}

static void print_stats(void) {
    printf(
        "%zu blocks, %zu total bytes, %zu success, %zu failed, "
        "smallest_fail = %zu\033[0K\r",
        num_blocks, total_bytes, num_success, num_failed, smallest_fail);
    fflush(stdout);
}

int main(void) {
    srand(0x77777777);

    /* tinyalloc */
    ta_init(&cfg);

    assert(ta_check(&cfg));

    /* test 1-byte allocations */
    for (size_t idx = 0; idx < MAX_TEST_BLOCKS; idx++) {
        if (!test_malloc(idx, 1, rand())) {
            break;
        }
        print_stats();
    }

    assert(ta_check(&cfg));

    /* random free/malloc */
    for (size_t n = 0; n < NUM_RANDOM_ALLOCS; n++) {
        size_t idx = (size_t)rand() % MAX_TEST_BLOCKS;
        test_free(idx);
        test_malloc(idx, rand_bias_low(), rand());
        print_stats();
    }

    assert(ta_check(&cfg));

    /* random free/calloc */
    for (size_t n = 0; n < NUM_RANDOM_ALLOCS; n++) {
        size_t idx = (size_t)rand() % MAX_TEST_BLOCKS;
        test_free(idx);
        test_calloc(idx, rand_bias_low(), rand_bias_low(), rand());
        print_stats();
    }

    assert(ta_check(&cfg));

    /* random realloc */
    for (size_t n = 0; n < NUM_RANDOM_ALLOCS; n++) {
        size_t idx = (size_t)rand() % MAX_TEST_BLOCKS;
        test_realloc(idx, rand_bias_low(), rand());
        print_stats();
    }

    assert(ta_check(&cfg));

    /* free remaining */
    for (size_t idx = 0; idx < MAX_TEST_BLOCKS; idx++) {
        test_free(idx);
        print_stats();
    }

    assert(ta_check(&cfg));

    printf("\n");
    return 0;
}
