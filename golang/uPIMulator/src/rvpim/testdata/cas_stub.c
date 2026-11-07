/* TEST-ONLY stand-in for domain_specific_proc/sw/sim/cas.c. It is NOT a core model.
 *
 * Implements cas.h just far enough to develop and check the uPIMulator RV-PIM
 * co-simulation (src/rvpim) without the real CAS. cas_load_elf() reads a text script
 * instead of an ELF, one op per line ('#' starts a comment, numbers take 0x):
 *   C n                compute n cycles (n retired instructions)
 *   L wram mram size   issue MRAM -> WRAM DMA in the next cycle (edge), the command's cycle
 *   S wram mram size   issue WRAM -> MRAM DMA in the next cycle
 *   W tag              wait for tag: served at edge max(now + 1, done) (CAS semantics);
 *                      unknown completion -> CAS_BLOCKED with no state change
 *   A                  wait for every issued tag
 *   E code             exit with tohost = code
 *
 * Build:  gcc -O2 -I<domain_specific_proc>/sw/sim -c cas_stub.c -o cas_stub.o
 *         ar rcs <dir>/libcas.a cas_stub.o      (then link rvpim with -L<dir>)
 */
#include "cas.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct { char op; uint32_t a, b, c; } op_t;

struct cas {
    cas_config_t cfg;
    uint8_t *mem[8];
    op_t *ops;
    int nops, pc, exited;
    uint64_t cyc, retired, left;
    cas_dma_t q[64];               /* issued, not popped yet */
    int qn;
    int64_t *done;                 /* completion cycle per tag, -1 = in flight */
    uint32_t ntag, wait_tag, exit_code;
    char err[160];
};

void cas_default_config(cas_config_t *cfg) {
    memset(cfg, 0, sizeof *cfg);
    cfg->i_latency = cfg->d_latency = 1;
    cfg->reset_pc = 0x80000000u;
    cfg->nregions = 1;
    cfg->regions[0].base = 0x80000000u;
    cfg->regions[0].size = 16u << 20;
}

cas_t *cas_create(const cas_config_t *cfg) {
    cas_t *c = calloc(1, sizeof *c);
    c->cfg = *cfg;
    for (int i = 0; i < cfg->nregions; i++) c->mem[i] = calloc(1, cfg->regions[i].size);
    return c;
}

void cas_destroy(cas_t *c) {
    for (int i = 0; i < c->cfg.nregions; i++) free(c->mem[i]);
    free(c->ops); free(c->done); free(c);
}

int cas_load_elf(cas_t *c, const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) { snprintf(c->err, sizeof c->err, "cannot open %s", path); return -1; }
    char line[256];
    while (fgets(line, sizeof line, f)) {
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (!*p || *p == '\n' || *p == '#') continue;
        op_t o = {*p++, 0, 0, 0};
        o.a = strtoul(p, &p, 0); o.b = strtoul(p, &p, 0); o.c = strtoul(p, &p, 0);
        c->ops = realloc(c->ops, (c->nops + 1) * sizeof *c->ops);
        c->ops[c->nops++] = o;
    }
    fclose(f);
    return 0;
}

uint8_t *cas_ptr(cas_t *c, uint32_t addr, uint32_t len) {
    for (int i = 0; i < c->cfg.nregions; i++) {
        cas_region_t r = c->cfg.regions[i];
        if (addr >= r.base && (uint64_t)addr + len <= (uint64_t)r.base + r.size) return c->mem[i] + (addr - r.base);
    }
    return NULL;
}

uint8_t *cas_mram(cas_t *c) { (void)c; return NULL; }

static int fail(cas_t *c, const char *msg) { snprintf(c->err, sizeof c->err, "%s (op %d)", msg, c->pc); return CAS_ERROR; }

static void retire_wait(cas_t *c, int64_t d) {
    c->cyc = (uint64_t)d > c->cyc + 1 ? (uint64_t)d : c->cyc + 1;
    c->retired++; c->pc++;
}

int cas_run(cas_t *c, uint64_t limit) {
    for (;;) {
        if (c->exited) return CAS_EXIT;
        if (c->cyc >= limit) return CAS_LIMIT;
        if (c->pc >= c->nops) return fail(c, "script ended without E");
        op_t *o = &c->ops[c->pc];
        switch (o->op) {
        case 'C': {
            if (!c->left) c->left = o->a;
            uint64_t n = c->left < limit - c->cyc ? c->left : limit - c->cyc;
            c->cyc += n; c->retired += n; c->left -= n;
            if (!c->left) c->pc++;
            break;
        }
        case 'L': case 'S': {
            if (c->qn == 64) return fail(c, "host did not pop DMA commands");
            c->cyc++; c->retired++; c->pc++;
            c->q[c->qn++] = (cas_dma_t){c->ntag, o->a, o->b, o->c, o->op == 'S', c->cyc};
            c->done = realloc(c->done, (c->ntag + 1) * sizeof *c->done);
            c->done[c->ntag++] = -1;
            return CAS_DMA;
        }
        case 'W':
            if (o->a >= c->ntag) return fail(c, "wait on a tag never issued");
            if (c->done[o->a] < 0) { c->wait_tag = o->a; return CAS_BLOCKED; }
            retire_wait(c, c->done[o->a]);
            break;
        case 'A': {
            int64_t d = 0;
            for (uint32_t t = 0; t < c->ntag; t++) {
                if (c->done[t] < 0) { c->wait_tag = t; return CAS_BLOCKED; }
                if (c->done[t] > d) d = c->done[t];
            }
            retire_wait(c, d);
            break;
        }
        case 'E':
            c->exit_code = o->a; c->exited = 1; c->cyc++; c->retired++;
            return CAS_EXIT;
        default:
            return fail(c, "unknown op");
        }
    }
}

uint64_t cas_cycles(const cas_t *c) { return c->cyc; }
uint64_t cas_retired(const cas_t *c) { return c->retired; }
uint32_t cas_exit_code(const cas_t *c) { return c->exit_code; }
const char *cas_error(const cas_t *c) { return c->err; }

int cas_dma_pop(cas_t *c, cas_dma_t *out) {
    if (!c->qn) return 0;
    *out = c->q[0];
    memmove(c->q, c->q + 1, --c->qn * sizeof *c->q);
    return 1;
}

void cas_dma_done(cas_t *c, uint32_t tag, uint64_t cycle) { if (tag < c->ntag) c->done[tag] = (int64_t)cycle; }
uint32_t cas_dma_wait_tag(const cas_t *c) { return c->wait_tag; }

void cas_set_commit_log(cas_t *c, const char *path) { (void)c; (void)path; }
void cas_dump_signature(cas_t *c, const char *path) { (void)c; (void)path; }

void cas_write_stats(cas_t *c, const char *path) {
    FILE *f = fopen(path, "w");
    if (!f) return;
    fprintf(f, "{\"stub\": true, \"cycles\": %llu, \"retired\": %llu}\n",
            (unsigned long long)c->cyc, (unsigned long long)c->retired);
    fclose(f);
}
