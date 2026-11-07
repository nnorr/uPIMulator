//go:build rvpim

// Package rvpim runs an RV32 core model (CAS, domain_specific_proc/sw/sim/cas.h) in
// place of the UPMEM DPU logic, on top of uPIMulator's MRAM/DMA memory model
// (src/simulator/dpu/dram). Enabled by --rv_elf; see main.go for the other rv_* options.
//
// Build (cgo; cas.h and libcas.a from domain_specific_proc/sw/sim):
//
//	CAS=$HOME/dev/11_crv/domain_specific_proc/sw/sim
//	CGO_CFLAGS="-I$CAS" \
//	CGO_LDFLAGS="-L$CAS -L$HOME/tools/riscv/lib -Wl,-rpath,$HOME/tools/riscv/lib" \
//	go build -tags rvpim -o build/rvpim ./src
//
// Co-simulation. Memory-side logic cycle t runs exactly what Dpu.Cycle() does around the
// DMA unit: commands the core issued in cycle t enter the DMA input queue, then
// Dma.Cycle() (one command into the memory controller, one completion out of it), then
// the same number of memory-controller cycles. The core runs ahead with cas_run() until
// an event; the memory side then catches up to the core's cycle. The core sees memory
// only through DMA: a store's WRAM bytes are read at issue (when cas_run returns), and a
// completion is visible to the core only at a wait, where it either has been reported
// already (memory caught up past it) or the core blocks and memory runs until it is.
// Memory never steps a cycle before the core has issued everything for it (a command
// issued in an already-simulated cycle is an error), so the result is cycle-exact;
// --rv_lockstep 1 syncs every cycle as a reference.
package rvpim

/*
#cgo LDFLAGS: -lcas -lsoftfloat -lm
#include <stdlib.h>
#include "cas.h"
*/
import "C"

import (
	"errors"
	"fmt"
	"os"
	"strconv"
	"strings"
	"sync"
	"time"
	"uPIMulator/src/abi/encoding"
	"uPIMulator/src/misc"
	"uPIMulator/src/simulator/dpu/dram"
	"unsafe"
)

// RV-PIM memory map, domain_specific_proc/sw/runtime/pim.h and link_pim.ld.
const (
	iramBase = 0x80000000
	wramBase = 0x80010000
	ramSize  = 0x10000
	dmaBase  = 0x80020000
)

type dma struct {
	tag, wram, size uint32
	toMram          bool
	issue           int64
	cmd             *dram.DmaCommand
}

type dpu struct {
	c          *C.cas_t
	mc         *dram.MemoryController
	mramOffset int64
	mramSize   int64
	ratio      float64
	lockstep   bool

	t        int64                     // memory-side logic cycle; cycles [0, t) are done
	issued   []*dma                    // popped from the core, waiting for their issue cycle
	inputQ   []*dram.DmaCommand        // logic.Dma input_q
	inflight map[*dram.DmaCommand]*dma // DmaCommand -> tag (instead of its DPU instruction)
	done     []bool                    // by tag
	pending  int

	num, bytes, latSum, latMax [2]int64 // [load, store]
	blocked, stallEnd          int64    // core cycles stalled on DMA waits; end of the last stall
}

func (d *dpu) ptr(addr, n uint32) []byte {
	p := C.cas_ptr(d.c, C.uint32_t(addr), C.uint32_t(n))
	if p == nil {
		return nil
	}
	return unsafe.Slice((*byte)(unsafe.Pointer(p)), n)
}

// pop takes the commands the core has issued; a store snapshots its WRAM bytes now, at
// issue, like Dma.TransferFromWram does for SDMA.
func (d *dpu) pop() error {
	var x C.cas_dma_t
	for C.cas_dma_pop(d.c, &x) != 0 {
		m := &dma{tag: uint32(x.tag), wram: uint32(x.wram), size: uint32(x.size), toMram: x.to_mram != 0, issue: int64(x.cycle)}
		mram := int64(x.mram)
		buf := d.ptr(m.wram, m.size)
		switch {
		case m.issue < d.t:
			return fmt.Errorf("DMA tag %d issued at cycle %d, memory already at %d", m.tag, m.issue, d.t)
		case int(m.tag) != len(d.done):
			return fmt.Errorf("DMA tag %d out of order (expected %d)", m.tag, len(d.done))
		case buf == nil || mram+int64(m.size) > d.mramSize:
			return fmt.Errorf("DMA tag %d out of range: wram 0x%x mram 0x%x size %d", m.tag, m.wram, mram, m.size)
		}
		m.cmd = new(dram.DmaCommand)
		if m.toMram {
			bs := new(encoding.ByteStream)
			bs.Init()
			for _, b := range buf {
				bs.Append(b)
			}
			m.cmd.InitWriteToMram(d.mramOffset+mram, int64(m.size), bs)
		} else {
			m.cmd.InitReadFromMram(d.mramOffset+mram, int64(m.size))
		}
		d.inflight[m.cmd] = m
		d.issued = append(d.issued, m)
		d.done = append(d.done, false)
		d.pending++
	}
	return nil
}

// step simulates logic cycle t in Dpu.Cycle() order.
func (d *dpu) step() {
	for len(d.issued) > 0 && d.issued[0].issue == d.t { // Logic.Cycle: DMA issue
		d.inputQ = append(d.inputQ, d.issued[0].cmd)
		d.issued = d.issued[1:]
	}
	if len(d.inputQ) > 0 && d.mc.CanPush() { // Dma.ServiceInputQ
		d.mc.Push(d.inputQ[0])
		d.inputQ = d.inputQ[1:]
	}
	if d.mc.CanPop() { // Dma.ServiceReadyQ: the load's data lands in WRAM this cycle
		d.complete(d.mc.Pop())
	}
	n := int(d.ratio*float64(d.t) - d.ratio*float64(d.t-1)) // same expression as Dpu.Cycle()
	for i := 0; i < n; i++ {
		d.mc.Cycle()
	}
	d.t++
}

func (d *dpu) complete(cmd *dram.DmaCommand) {
	m := d.inflight[cmd]
	delete(d.inflight, cmd)
	k := 1
	if !m.toMram {
		k = 0
		bs := cmd.ByteStream(cmd.MramAddress(), cmd.Size())
		buf := d.ptr(m.wram, m.size)
		for i := range buf {
			buf[i] = bs.Get(i)
		}
	}
	C.cas_dma_done(d.c, C.uint32_t(m.tag), C.uint64_t(d.t))
	d.done[m.tag] = true
	d.pending--
	lat := d.t - m.issue
	d.num[k]++
	d.bytes[k] += int64(m.size)
	d.latSum[k] += lat
	d.latMax[k] = max(d.latMax[k], lat)
}

// advance runs the memory side up to (not including) logic cycle target. With nothing in
// the DMA input queue or the memory controller, a memory cycle changes no state: every
// timing countdown (tRCD, tRAS, tCL, tBL, tRP) is the timer of an entry in one of the
// controller's queues, and IsEmpty() covers all of them (the open row just stays open).
// Such stretches are skipped up to the next issue.
func (d *dpu) advance(target int64) {
	for d.t < target {
		if !d.lockstep && len(d.inputQ) == 0 && d.mc.IsEmpty() {
			d.t = target
			if len(d.issued) > 0 {
				d.t = min(target, d.issued[0].issue)
			}
			if d.t == target {
				return
			}
		}
		d.step()
	}
}

func (d *dpu) run(maxCycles uint64) error {
	lastBlocked := int64(-1)
	for {
		limit := maxCycles
		if d.lockstep {
			limit = min(maxCycles, uint64(C.cas_cycles(d.c))+1)
		}
		ev := C.cas_run(d.c, C.uint64_t(limit))
		if err := d.pop(); err != nil {
			return err
		}
		now := int64(C.cas_cycles(d.c))
		d.advance(now)
		switch ev {
		case C.CAS_DMA:
		case C.CAS_BLOCKED:
			tag := int(C.cas_dma_wait_tag(d.c))
			start := d.t
			switch {
			case tag < len(d.done) && !d.done[tag]:
				for !d.done[tag] {
					d.step()
				}
			case now != lastBlocked:
				// it completed while memory caught up; the core sees that when it resumes
			case d.pending > 0:
				// blocked again on a completed or unknown tag: DMA_WAIT_ALL
				for d.pending > 0 {
					d.step()
				}
			default:
				return fmt.Errorf("core waits on DMA tag %d at cycle %d with nothing in flight", tag, now)
			}
			// As CAS counts wait stalls: the wait could be served at edge now+1 and is served at
			// the completion edge d.t-1; a WAIT_ALL can block again at the same edge.
			if from := max(now+1, d.stallEnd); d.t > start && d.t-1 > from {
				d.blocked += d.t - 1 - from
				d.stallEnd = d.t - 1
			}
			lastBlocked = now
		case C.CAS_EXIT:
			for d.pending > 0 || !d.mc.IsEmpty() { // stores still in flight reach MRAM
				d.step()
			}
			return nil
		case C.CAS_LIMIT:
			if uint64(now) >= maxCycles {
				return fmt.Errorf("cycle limit %d reached", maxCycles)
			}
		default:
			return fmt.Errorf("cas_run event %d at cycle %d: %s", int(ev), now, C.GoString(C.cas_error(d.c)))
		}
	}
}

type dump struct {
	off, n int64
	file   string
}

// perDpu names a per-DPU output file when there are several DPUs.
func perDpu(file string, id, n int) string {
	if n > 1 {
		return fmt.Sprintf("%s.dpu%d", file, id)
	}
	return file
}

func simulate(id, n int, p *misc.CommandLineParser, image []byte, dumps []dump) (string, error) {
	var cfg C.cas_config_t
	C.cas_default_config(&cfg)
	cfg.reset_pc = iramBase
	cfg.nregions = 2
	cfg.regions[0] = C.cas_region_t{base: iramBase, size: ramSize}
	cfg.regions[1] = C.cas_region_t{base: wramBase, size: ramSize}
	cfg.dma_base = dmaBase
	cfg.dma_external = 1
	switch p.StringParameter("rv_core") {
	case "baseline":
	case "rv32f":
		cfg.rv32f = 1
	default:
		return "", fmt.Errorf("--rv_core %q: baseline or rv32f", p.StringParameter("rv_core"))
	}
	c := C.cas_create(&cfg)
	if c == nil {
		return "", errors.New("cas_create failed")
	}
	defer C.cas_destroy(c)
	elf := C.CString(p.StringParameter("rv_elf"))
	defer C.free(unsafe.Pointer(elf))
	if C.cas_load_elf(c, elf) != 0 {
		return "", fmt.Errorf("cas_load_elf: %s", C.GoString(C.cas_error(c)))
	}

	config_loader := new(misc.ConfigLoader)
	config_loader.Init()
	d := &dpu{
		c:          c,
		mramOffset: config_loader.MramOffset(),
		mramSize:   config_loader.MramSize(),
		ratio:      float64(p.IntParameter("memory_frequency")) / float64(p.IntParameter("logic_frequency")),
		lockstep:   p.IntParameter("rv_lockstep") != 0,
		inflight:   make(map[*dram.DmaCommand]*dma),
	}
	mram := new(dram.Mram)
	mram.Init(p)
	d.mc = new(dram.MemoryController)
	d.mc.Init(0, 0, id, p)
	d.mc.ConnectMram(mram)
	if int64(len(image)) > d.mramSize {
		return "", fmt.Errorf("MRAM image is %d B, MRAM is %d B", len(image), d.mramSize)
	}
	if len(image) > 0 { // functional preload, as host.Load() -> Dma.TransferToMram
		bs := new(encoding.ByteStream)
		bs.Init()
		for _, b := range image {
			bs.Append(b)
		}
		d.mc.Write(d.mramOffset, bs.Size(), bs)
	}

	start := time.Now()
	err := d.run(uint64(p.IntParameter("rv_max_cycles")))
	wall := time.Since(start)
	if err != nil {
		return "", err
	}

	d.mc.Flush() // write the open row back, as Dma.TransferFromMram does
	for _, x := range dumps {
		if x.off < 0 || x.n < 0 || x.off+x.n > d.mramSize {
			return "", fmt.Errorf("MRAM dump %d:%d out of range", x.off, x.n)
		}
		bs := d.mc.Read(d.mramOffset+x.off, x.n)
		buf := make([]byte, x.n)
		for i := range buf {
			buf[i] = bs.Get(i)
		}
		if err := os.WriteFile(perDpu(x.file, id, n), buf, 0o644); err != nil {
			return "", err
		}
	}
	if f := p.StringParameter("rv_stats"); f != "" {
		cs := C.CString(perDpu(f, id, n))
		C.cas_write_stats(c, cs)
		C.free(unsafe.Pointer(cs))
	}
	if f := p.StringParameter("rv_signature"); f != "" {
		cs := C.CString(perDpu(f, id, n))
		C.cas_dump_signature(c, cs)
		C.free(unsafe.Pointer(cs))
	}

	cycles := int64(C.cas_cycles(c))
	retired := int64(C.cas_retired(c))
	code := uint32(C.cas_exit_code(c))
	mhz := float64(p.IntParameter("logic_frequency"))
	var b strings.Builder
	fmt.Fprintf(&b, "DPU %d: exit_code %d\n", id, code)
	fmt.Fprintf(&b, "  core_cycles         %d (%.3f us at %.0f MHz)\n", cycles, float64(cycles)/mhz, mhz)
	fmt.Fprintf(&b, "  retired             %d (IPC %.3f)\n", retired, float64(retired)/float64(max(cycles, 1)))
	for k, name := range []string{"dma_load ", "dma_store"} {
		fmt.Fprintf(&b, "  %s           %d cmds, %d B, latency avg %.1f max %d cycles\n", name,
			d.num[k], d.bytes[k], float64(d.latSum[k])/float64(max(d.num[k], 1)), d.latMax[k])
	}
	fmt.Fprintf(&b, "  dma_bandwidth       %.1f MB/s (load+store bytes / core time)\n",
		float64(d.bytes[0]+d.bytes[1])*mhz/float64(max(cycles, 1)))
	fmt.Fprintf(&b, "  dma_blocked_cycles  %d (%.1f%%)\n", d.blocked, 100*float64(d.blocked)/float64(max(cycles, 1)))
	if d.t > cycles {
		fmt.Fprintf(&b, "  memory_drained_at   %d\n", d.t)
	}
	fmt.Fprintf(&b, "  wall_time           %.3f s\n", wall.Seconds())
	if code != 1 {
		return b.String(), fmt.Errorf("exit code %d", code)
	}
	return b.String(), nil
}

func parseDumps(s string) ([]dump, error) {
	var dumps []dump
	for _, f := range strings.Split(s, ",") {
		if f == "" {
			continue
		}
		parts := strings.SplitN(f, ":", 3)
		if len(parts) != 3 {
			return nil, fmt.Errorf("--rv_dump_mram %q: want off:len:file", f)
		}
		off, err1 := strconv.ParseInt(parts[0], 0, 64)
		n, err2 := strconv.ParseInt(parts[1], 0, 64)
		if err := errors.Join(err1, err2); err != nil {
			return nil, fmt.Errorf("--rv_dump_mram %q: %w", f, err)
		}
		dumps = append(dumps, dump{off, n, parts[2]})
	}
	return dumps, nil
}

// Run simulates every DPU (independent, same ELF) and returns the process exit status.
func Run(p *misc.CommandLineParser) int {
	n := int(p.IntParameter("num_channels") * p.IntParameter("num_ranks_per_channel") * p.IntParameter("num_dpus_per_rank"))
	dumps, err := parseDumps(p.StringParameter("rv_dump_mram"))
	if err != nil {
		fmt.Fprintln(os.Stderr, "rvpim:", err)
		return 2
	}
	var images [][]byte
	for _, f := range strings.Split(p.StringParameter("rv_mram_image"), ",") {
		if f == "" {
			continue
		}
		img, err := os.ReadFile(f)
		if err != nil {
			fmt.Fprintln(os.Stderr, "rvpim:", err)
			return 2
		}
		images = append(images, img)
	}
	if len(images) > 1 && len(images) != n {
		fmt.Fprintf(os.Stderr, "rvpim: %d MRAM images for %d DPUs (give one shared or one per DPU)\n", len(images), n)
		return 2
	}

	reports := make([]string, n)
	errs := make([]error, n)
	sem := make(chan struct{}, max(p.IntParameter("num_simulation_threads"), 1))
	var wg sync.WaitGroup
	for i := 0; i < n; i++ {
		var image []byte
		if len(images) > 0 {
			image = images[min(i, len(images)-1)]
		}
		wg.Add(1)
		go func(i int, image []byte) {
			defer wg.Done()
			sem <- struct{}{}
			defer func() { <-sem }()
			reports[i], errs[i] = simulate(i, n, p, image, dumps)
		}(i, image)
	}
	wg.Wait()

	status := 0
	for i := 0; i < n; i++ {
		fmt.Print(reports[i])
		if errs[i] != nil {
			fmt.Printf("DPU %d: FAIL: %v\n", i, errs[i])
			status = 1
		}
	}
	return status
}
