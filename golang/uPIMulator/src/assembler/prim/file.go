package prim

import (
	"errors"
	"os"
	"strings"
	"uPIMulator/src/abi/encoding"
	"uPIMulator/src/misc"
)

// File is a benchmark whose input is a prepared MRAM image per DPU (--mram_image a.bin, or
// a.bin,b.bin,... one per DPU), placed at DPU_MRAM_HEAP_POINTER. It has no host arguments and
// no output check: the results are read back with --dump_mram and checked by the caller.
type File struct {
	images [][]byte
}

func (this *File) Init(command_line_parser *misc.CommandLineParser) {
	num_dpus := int(command_line_parser.IntParameter("num_channels") *
		command_line_parser.IntParameter("num_ranks_per_channel") *
		command_line_parser.IntParameter("num_dpus_per_rank"))

	for _, f := range strings.Split(command_line_parser.StringParameter("mram_image"), ",") {
		if f == "" {
			continue
		}
		image, err := os.ReadFile(f)
		if err != nil {
			panic(err)
		}
		this.images = append(this.images, image)
	}
	if len(this.images) == 0 || (len(this.images) > 1 && len(this.images) != num_dpus) {
		panic(errors.New("--mram_image: give one image shared by all DPUs or one per DPU"))
	}
}

func (this *File) InputDpuHost(execution int, dpu_id int) map[string]*encoding.ByteStream {
	return make(map[string]*encoding.ByteStream, 0)
}

func (this *File) OutputDpuHost(execution int, dpu_id int) map[string]*encoding.ByteStream {
	return make(map[string]*encoding.ByteStream, 0)
}

func (this *File) InputDpuMramHeapPointerName(execution int, dpu_id int) (int64, *encoding.ByteStream) {
	byte_stream := new(encoding.ByteStream)
	byte_stream.Init()
	for _, b := range this.images[min(dpu_id, len(this.images)-1)] {
		byte_stream.Append(b)
	}
	return 0, byte_stream
}

func (this *File) OutputDpuMramHeapPointerName(execution int, dpu_id int) (int64, *encoding.ByteStream) {
	byte_stream := new(encoding.ByteStream)
	byte_stream.Init()
	return 0, byte_stream
}

func (this *File) NumExecutions() int {
	return 1
}
